#!/usr/bin/env python3
"""Capture Roboty encoder/control telemetry to CSV without sending motion commands."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable, TextIO

import serial

BASE_FIELDS = ("ms", "encoders", "calibrated", "left_ticks", "right_ticks",
               "left_delta", "right_delta", "left_rpm", "right_rpm",
               "left_invalid", "right_invalid", "sample_ms", "left_pwm",
               "right_pwm", "motion_active")
CONTROL_FIELDS = ("schema_version", "sample_seq", "sample_at_ms", "control_mode",
                  "control_fault", "left_target_rpm", "right_target_rpm")
FIELDS = BASE_FIELDS + CONTROL_FIELDS
_UINT32_FIELDS = ("ms", "left_invalid", "right_invalid", "sample_ms")
_INT64_FIELDS = ("left_ticks", "right_ticks", "left_delta", "right_delta")
FAULTS = {"none", "disabled", "configuration", "target_range", "feedback_unavailable",
          "stale_feedback", "invalid_feedback", "encoder_transition", "no_motion",
          "sample_timing", "numeric"}


def _finite_number(value: object) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def parse_telemetry_line(line: str | bytes) -> dict | None:
    """Validate M1 or M2 frames. Missing M1 control fields become blank CSV cells."""
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="replace")
    if not line.lstrip().startswith("{"):
        return None
    try:
        record = json.loads(line)
    except (ValueError, TypeError, RecursionError):
        return None
    if not isinstance(record, dict) or record.get("type") != "telemetry":
        return None
    if any(field not in record for field in BASE_FIELDS):
        return None
    if any(type(record[field]) is not bool for field in ("encoders", "calibrated", "motion_active")):
        return None
    for fields, minimum, maximum in (
        (_UINT32_FIELDS, 0, 2**32 - 1), (_INT64_FIELDS, -(2**63), 2**63 - 1),
        (("left_pwm", "right_pwm"), -100, 100),
    ):
        if any(type(record[field]) is not int or not minimum <= record[field] <= maximum for field in fields):
            return None
    if record["calibrated"] and not record["encoders"]:
        return None
    for field in ("left_rpm", "right_rpm"):
        value = record[field]
        if value is not None and (not _finite_number(value) or not record["calibrated"] or record["sample_ms"] == 0):
            return None
    version = record.get("schema_version", 1)
    if type(version) is not int or version not in (1, 2):
        return None
    if version == 2:
        if any(field not in record for field in CONTROL_FIELDS):
            return None
        if any(type(record[field]) is not int or not 0 <= record[field] < 2**32
               for field in ("sample_seq", "sample_at_ms")):
            return None
        if not isinstance(record["control_mode"], str) or record["control_mode"] not in {"stopped", "open_loop", "velocity"}:
            return None
        if not isinstance(record["control_fault"], str) or record["control_fault"] not in FAULTS:
            return None
        if any(not _finite_number(record[field]) for field in ("left_target_rpm", "right_target_rpm")):
            return None
        if record["control_mode"] == "velocity" and (
            not record["encoders"] or not record["calibrated"] or record["sample_ms"] == 0 or
            record["left_rpm"] is None or record["right_rpm"] is None or
            not record["motion_active"] or record["control_fault"] != "none"
        ):
            return None
        if record["control_fault"] != "none" and (
            record["control_mode"] != "stopped" or record["motion_active"] or
            record["left_pwm"] != 0 or record["right_pwm"] != 0
        ):
            return None
    else:
        record.update({field: None for field in CONTROL_FIELDS})
        record["schema_version"] = 1
    return {field: record[field] for field in FIELDS}


def _send(connection: serial.Serial, command: str) -> None:
    payload = (command + "\n").encode("ascii")
    if connection.write(payload) != len(payload):
        raise serial.SerialTimeoutException("Incomplete serial write.")


def capture(connection: serial.Serial, output: TextIO, interval_ms: int = 200,
            duration: float | None = None, clock: Callable[[], float] = time.monotonic) -> int:
    if type(interval_ms) is not int or not 50 <= interval_ms <= 5000:
        raise ValueError("Telemetry interval must be between 50 and 5000 ms.")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("Capture duration must be a finite positive number.")
    writer = csv.DictWriter(output, fieldnames=FIELDS)
    writer.writeheader()
    output.flush()
    rows = 0
    started = clock()
    try:
        _send(connection, f"TELEMETRY {interval_ms}")
        while duration is None or clock() - started < duration:
            line = connection.readline(4096)
            if line.startswith(b"ERR "):
                raise RuntimeError(line.decode("utf-8", errors="replace").strip())
            record = parse_telemetry_line(line)
            if record is not None:
                writer.writerow(record)
                output.flush()
                rows += 1
    finally:
        already_failing = sys.exc_info()[0] is not None
        try:
            _send(connection, "TELEMETRY 0")
        except (serial.SerialException, OSError):
            if not already_failing:
                raise
    return rows


def _positive_duration(value: str) -> float:
    try:
        seconds = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Duration must be a number.") from exc
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("Duration must be a finite positive number.")
    return seconds


def _interval(value: str) -> int:
    try:
        interval = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Interval must be an integer.") from exc
    if not 50 <= interval <= 5000:
        raise argparse.ArgumentTypeError("Interval must be between 50 and 5000 ms.")
    return interval


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--output", required=True, type=Path, help="Destination CSV file")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=_interval, default=200, metavar="MS")
    parser.add_argument("--duration", type=_positive_duration, metavar="SECONDS",
                        help="Stop after this duration; otherwise capture until Ctrl+C")
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=0.2) as connection:
            time.sleep(1.0)
            with args.output.open("w", newline="", encoding="utf-8") as output:
                print(f"Capturing telemetry to {args.output}; Ctrl+C stops capture.")
                rows = capture(connection, output, args.interval, args.duration)
                print(f"Saved {rows} telemetry records.")
    except KeyboardInterrupt:
        print("Capture stopped; received records are saved.")
    except (serial.SerialException, OSError, RuntimeError) as exc:
        print(f"Telemetry capture failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
