from __future__ import annotations

import csv
import io
import json
import pathlib
import sys
import unittest

import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from telemetry_capture import FIELDS, capture, parse_telemetry_line, _positive_duration


def frame(**overrides):
    data = {"type": "telemetry", "ms": 200, "encoders": True, "calibrated": True,
            "left_ticks": 80, "right_ticks": -40, "left_delta": 40, "right_delta": -20,
            "left_rpm": 60.0, "right_rpm": -30.0, "left_invalid": 0, "right_invalid": 1,
            "sample_ms": 100, "left_pwm": 30, "right_pwm": -20, "motion_active": True}
    data.update(overrides)
    return (json.dumps(data) + "\n").encode("ascii")


class FakeCaptureSerial:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def readline(self, size=-1):
        line = next(self.lines, b"")
        if isinstance(line, BaseException):
            raise line
        return line


class ParserTests(unittest.TestCase):
    def test_valid_signed_values_and_unavailable_rpm(self) -> None:
        parsed = parse_telemetry_line(frame())
        self.assertEqual(parsed["right_ticks"], -40)
        self.assertEqual(parsed["right_rpm"], -30.0)
        for overrides in (
            {"encoders": False, "calibrated": False, "left_rpm": None, "right_rpm": None},
            {"calibrated": False, "left_rpm": None, "right_rpm": None},
            {"sample_ms": 0, "left_rpm": None, "right_rpm": None},
        ):
            with self.subTest(overrides=overrides):
                self.assertIsNotNone(parse_telemetry_line(frame(**overrides)))

    def test_boot_text_malformed_json_and_invalid_fields_are_ignored(self) -> None:
        for line in (b"Roboty ready\n", b"left=0% right=0%\n", b"{truncated\n",
                     b"[]", b'{"type":"telemetry"}', b'{"type":"other"}',
                     frame(left_rpm=float("nan")), frame(right_rpm=float("inf")),
                     frame(left_ticks=True), frame(ms=-1), frame(right_invalid=-1),
                     frame(left_pwm=101), frame(encoders="true"),
                     frame(left_rpm=True), frame(left_rpm=10**1000), frame(left_ticks=2**63),
                     frame(calibrated=False), frame(sample_ms=0),
                     frame(encoders=False)):
            with self.subTest(line=line):
                self.assertIsNone(parse_telemetry_line(line))


class CaptureTests(unittest.TestCase):
    def test_mixed_serial_output_becomes_csv_without_motion_commands(self) -> None:
        port = FakeCaptureSerial([b"Roboty M1 ready\n", b"OK telemetry=200ms\n",
                                  frame(), frame(ms=300, left_ticks=120), b"{partial"])
        clock_values = iter([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        output = io.StringIO()
        rows = capture(port, output, duration=0.6, clock=lambda: next(clock_values))
        self.assertEqual(rows, 2)
        records = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(tuple(records[0]), FIELDS)
        self.assertEqual(records[0]["right_rpm"], "-30.0")
        self.assertEqual(records[1]["left_ticks"], "120")
        self.assertEqual(port.writes, [b"TELEMETRY 200\n", b"TELEMETRY 0\n"])

    def test_null_rpm_is_blank_in_csv(self) -> None:
        port = FakeCaptureSerial([frame(calibrated=False, left_rpm=None, right_rpm=None)])
        values = iter([0, 0.1, 0.2])
        output = io.StringIO()
        self.assertEqual(capture(port, output, duration=0.2, clock=lambda: next(values)), 1)
        row = next(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(row["left_rpm"], "")
        self.assertEqual(row["calibrated"], "False")

    def test_interrupt_disconnect_or_firmware_error_disable_stream(self) -> None:
        for failure in (KeyboardInterrupt(), serial.SerialException("Unplugged"),
                        b"ERR usage: TELEMETRY [0|50..5000]\n"):
            with self.subTest(failure=failure):
                port = FakeCaptureSerial([failure])
                expected = type(failure) if isinstance(failure, BaseException) else RuntimeError
                with self.assertRaises(expected):
                    capture(port, io.StringIO())
                self.assertEqual(port.writes[-1], b"TELEMETRY 0\n")
                self.assertTrue(all(data.startswith(b"TELEMETRY ") for data in port.writes))

    def test_partial_start_write_reports_failure_and_attempts_cleanup(self) -> None:
        class PartialPort(FakeCaptureSerial):
            def write(self, data):
                super().write(data)
                return 1 if data == b"TELEMETRY 200\n" else len(data)

        port = PartialPort([])
        with self.assertRaises(serial.SerialTimeoutException):
            capture(port, io.StringIO())
        self.assertEqual(port.writes, [b"TELEMETRY 200\n", b"TELEMETRY 0\n"])

    def test_invalid_options_do_not_write_to_serial(self) -> None:
        for kwargs in ({"interval_ms": 49}, {"interval_ms": 5001},
                       {"duration": 0}, {"duration": -1}, {"duration": float("inf")},
                       {"duration": float("nan")}):
            with self.subTest(kwargs=kwargs):
                port = FakeCaptureSerial([])
                with self.assertRaises(ValueError):
                    capture(port, io.StringIO(), **kwargs)
                self.assertEqual(port.writes, [])

    def test_duration_argument_rejects_nonfinite_and_nonpositive_values(self) -> None:
        import argparse
        for value in ("nan", "inf", "-inf", "0", "-1", "text"):
            with self.subTest(value=value):
                with self.assertRaises(argparse.ArgumentTypeError):
                    _positive_duration(value)
        self.assertEqual(_positive_duration("1.5"), 1.5)


if __name__ == "__main__":
    unittest.main()
