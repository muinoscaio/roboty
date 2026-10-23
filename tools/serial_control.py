#!/usr/bin/env python3
"""Interactive USB control with serialized writes and fail-safe shutdown."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import queue
import re
import sys
import threading
import time
from typing import Callable

import serial


REFRESH_SECONDS = 0.20
IO_TIMEOUT_SECONDS = 0.20
_INTEGER = re.compile(r"[+-]?[0-9]+\Z")
_DIRECTIONS = {"FWD": "FWD", "F": "FWD", "BACK": "BACK", "B": "BACK",
               "LEFT": "LEFT", "L": "LEFT", "RIGHT": "RIGHT", "R": "RIGHT"}


@dataclass(frozen=True)
class Command:
    text: str
    motion: bool = False
    stop: bool = False


def _integer(token: str, minimum: int, maximum: int) -> int:
    if not _INTEGER.fullmatch(token):
        raise ValueError(f"Expected an integer between {minimum} and {maximum}.")
    value = int(token)
    if not minimum <= value <= maximum:
        raise ValueError(f"Expected an integer between {minimum} and {maximum}.")
    return value


def parse_command(line: str) -> Command:
    """Validate a single firmware command before it can become a repeat command."""
    if not line.isascii() or any(ord(c) < 32 and c != "\t" for c in line) or "\x7f" in line:
        raise ValueError("Commands must be one ASCII line without control characters.")
    if len(line) > 95:
        raise ValueError("Command exceeds the firmware's 95-character limit.")
    tokens = line.split()
    if not tokens:
        raise ValueError("Empty command.")
    name, arguments = tokens[0].upper(), tokens[1:]
    if name in _DIRECTIONS:
        if len(arguments) > 1:
            raise ValueError(f"Usage: {name} [0..100]")
        speed = f" {_integer(arguments[0], 0, 100)}" if arguments else ""
        return Command(_DIRECTIONS[name] + speed, motion=True)
    if name == "MOTOR":
        if len(arguments) != 2:
            raise ValueError("Usage: MOTOR <left -100..100> <right -100..100>")
        values = [_integer(value, -100, 100) for value in arguments]
        return Command(f"MOTOR {values[0]} {values[1]}", motion=True)
    if name == "DRIVE":
        if len(arguments) != 2:
            raise ValueError("Usage: DRIVE <linear -1..1> <angular -1..1>")
        try:
            values = [float(value) for value in arguments]
        except ValueError as exc:
            raise ValueError("DRIVE requires two finite numbers between -1 and 1.") from exc
        if any(not math.isfinite(value) or not -1 <= value <= 1 for value in values):
            raise ValueError("DRIVE requires two finite numbers between -1 and 1.")
        if any("_" in value for value in arguments):
            raise ValueError("Numeric separators are not supported by the firmware.")
        return Command("DRIVE " + " ".join(arguments), motion=True)
    if name in {"STOP", "S", "STATUS", "HELP", "?"} and not arguments:
        name = {"S": "STOP", "?": "HELP"}.get(name, name)
        return Command(name, stop=name == "STOP")
    if name == "TELEMETRY" and len(arguments) <= 1:
        if not arguments:
            return Command(name)
        interval = _integer(arguments[0], 0, 5000)
        if interval != 0 and interval < 50:
            raise ValueError("TELEMETRY interval must be 0 (off), or 50..5000 ms.")
        return Command(f"TELEMETRY {interval}")
    raise ValueError("Unknown command or wrong argument count; type HELP.")


class CommandRefresher:
    """A single lock orders motion changes, query writes and the final STOP."""

    def __init__(self, connection: serial.Serial, refresh_seconds: float = REFRESH_SECONDS) -> None:
        self.connection = connection
        self._refresh_seconds = refresh_seconds
        self._motion: str | None = None
        self._shutdown = threading.Event()
        self._lock = threading.Lock()
        self._error: Exception | None = None
        self._stop_sent = False
        self._started = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    @property
    def error(self) -> Exception | None:
        with self._lock:
            return self._error

    @property
    def stopped(self) -> bool:
        return self._shutdown.is_set()

    def start(self) -> None:
        with self._lock:
            if self._started or self._shutdown.is_set():
                raise RuntimeError("Command refresher cannot be restarted.")
            self._started = True
            self._thread.start()

    def _write_locked(self, command: str) -> None:
        payload = (command + "\n").encode("ascii")
        try:
            if self.connection.write(payload) != len(payload):
                raise serial.SerialTimeoutException("Incomplete serial write.")
        except (serial.SerialException, OSError) as exc:
            self._motion = None
            self._shutdown.set()
            if self._error is None:
                self._error = exc
            raise

    def submit(self, command: str) -> None:
        try:
            parsed = parse_command(command)
        except ValueError:
            self.halt()
            raise
        with self._lock:
            if self._shutdown.is_set():
                raise serial.SerialException("Command refresher is stopped.")
            if parsed.stop:
                self._motion = None
            elif parsed.motion:
                self._motion = parsed.text
            self._write_locked(parsed.text)

    def set_command(self, command: str) -> None:
        self.submit(command)

    def halt(self) -> None:
        """Stop current motion while allowing later, explicit motion commands."""
        with self._lock:
            self._motion = None
            if not self._shutdown.is_set():
                self._write_locked("STOP")

    def abort(self, error: Exception) -> None:
        with self._lock:
            if self._error is None:
                self._error = error
        self.stop()

    def stop(self) -> None:
        with self._lock:
            self._motion = None
            self._shutdown.set()
            if not self._stop_sent:
                self._stop_sent = True
                try:
                    self._write_locked("STOP")
                except (serial.SerialException, OSError):
                    pass
        if self._started and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._shutdown.wait(self._refresh_seconds):
            with self._lock:
                if self._shutdown.is_set():
                    return
                if self._motion is not None:
                    try:
                        self._write_locked(self._motion)
                    except (serial.SerialException, OSError):
                        return


def interactive_session(
    connection: serial.Serial,
    input_fn: Callable[[str], str] = input,
    output: Callable[[str], None] = print,
) -> int:
    """Exit on I/O errors even when the terminal is waiting for user input."""
    refresher = CommandRefresher(connection)
    finished = threading.Event()
    inputs: queue.Queue[str | Exception | None] = queue.Queue()

    def read_input() -> None:
        while not finished.is_set():
            try:
                inputs.put(input_fn("> "))
            except (EOFError, KeyboardInterrupt):
                inputs.put(None)
                return
            except Exception as exc:
                inputs.put(exc)
                return

    def read_serial() -> None:
        while not finished.is_set() and not refresher.stopped:
            try:
                line = connection.readline(4096).decode("utf-8", errors="replace").rstrip()
                if line:
                    output(f"< {line}")
                    if line.startswith(("ERR ", "WATCHDOG ")):
                        refresher.halt()
            except (serial.SerialException, OSError) as exc:
                refresher.abort(exc)
                return

    reader = threading.Thread(target=read_serial, daemon=True)
    try:
        refresher.start()
        refresher.submit("STOP")
        reader.start()
        threading.Thread(target=read_input, daemon=True).start()
        while not refresher.stopped:
            try:
                command = inputs.get(timeout=0.05)
            except queue.Empty:
                continue
            if command is None:
                break
            if isinstance(command, Exception):
                refresher.abort(command)
                break
            if not command.strip():
                continue
            if command.strip().upper() in {"QUIT", "EXIT"}:
                break
            try:
                refresher.submit(command)
            except ValueError as exc:
                output(f"Invalid command; motors stopped: {exc}")
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as exc:
        refresher.abort(exc)
    finally:
        finished.set()
        refresher.stop()
        if reader.ident is not None:
            reader.join(timeout=1.0)
    if refresher.error is not None:
        output(f"Connection stopped: {refresher.error}")
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Roboty USB serial controller")
    parser.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    try:
        with serial.Serial(args.port, args.baud, timeout=IO_TIMEOUT_SECONDS,
                           write_timeout=IO_TIMEOUT_SECONDS) as connection:
            time.sleep(1.0)
            print("Connected. Enter a firmware command; EXIT or Ctrl+C quits.")
            print("Motion repeats until STOP, an invalid command, or an I/O error.")
            print("STATUS, HELP and TELEMETRY queries keep the current motion active.")
            print("Examples: FWD 40 | LEFT 35 | DRIVE 0.5 0.2 | TELEMETRY 200 | STOP")
            return interactive_session(connection)
    except (serial.SerialException, OSError) as exc:
        print(f"Serial connection failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
