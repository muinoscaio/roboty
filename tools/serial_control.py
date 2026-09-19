#!/usr/bin/env python3
"""Simple host controller for Roboty M0 over USB serial."""

from __future__ import annotations

import argparse
import threading
import time

import serial


REFRESH_SECONDS = 0.20


class CommandRefresher:
    def __init__(self, connection: serial.Serial) -> None:
        self.connection = connection
        self._command = "STOP"
        self._running = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set_command(self, command: str) -> None:
        with self._lock:
            self._command = command

    def stop(self) -> None:
        self._running = False
        self.set_command("STOP")
        self.send_once("STOP")

    def send_once(self, command: str) -> None:
        self.connection.write((command.strip() + "\n").encode("utf-8"))
        self.connection.flush()

    def _run(self) -> None:
        while self._running:
            with self._lock:
                command = self._command
            if command != "STOP":
                self.send_once(command)
            time.sleep(REFRESH_SECONDS)


def reader(connection: serial.Serial) -> None:
    while connection.is_open:
        line = connection.readline().decode("utf-8", errors="replace").rstrip()
        if line:
            print(f"< {line}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Roboty M0 serial controller")
    parser.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as connection:
        time.sleep(1.0)
        print("Connected. Enter a firmware command.")
        print("Motion commands are refreshed automatically until STOP is entered.")
        print("Examples: FWD 40 | LEFT 35 | DRIVE 0.5 0.2 | STOP | HELP")

        threading.Thread(target=reader, args=(connection,), daemon=True).start()

        refresher = CommandRefresher(connection)
        refresher.start()

        try:
            while True:
                command = input("> ").strip()
                if not command:
                    continue

                upper = command.upper()
                if upper in {"QUIT", "EXIT"}:
                    break

                if upper.startswith(("FWD", "F ", "BACK", "B ", "LEFT", "L ", "RIGHT", "R ", "DRIVE", "MOTOR")):
                    refresher.set_command(command)
                else:
                    refresher.set_command("STOP")
                    refresher.send_once(command)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            refresher.stop()


if __name__ == "__main__":
    main()
