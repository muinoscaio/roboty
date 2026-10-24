from __future__ import annotations
import csv
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from serial_control import CommandRefresher, interactive_session, main, parse_command
from telemetry_capture import CONTROL_FIELDS, FIELDS, parse_telemetry_line


def frame(**changes):
    data = {"type": "telemetry", "ms": 200, "encoders": True, "calibrated": True,
            "left_ticks": 20, "right_ticks": -20, "left_delta": 10, "right_delta": -10,
            "left_rpm": 20.0, "right_rpm": -20.0, "left_invalid": 0, "right_invalid": 0,
            "sample_ms": 100, "left_pwm": 15, "right_pwm": -15, "motion_active": True,
            "schema_version": 2, "sample_seq": 2, "sample_at_ms": 200,
            "control_mode": "velocity", "control_fault": "none",
            "left_target_rpm": 20, "right_target_rpm": -20}
    data.update(changes)
    return (json.dumps(data) + "\n").encode("ascii")


class Port:
    def __init__(self):
        self.writes = []
        self.lines = queue.Queue()
        self.changed = threading.Condition()

    def write(self, payload):
        with self.changed:
            self.writes.append(payload.decode("ascii").strip())
            self.changed.notify_all()
        return len(payload)

    def readline(self, size=-1):
        try:
            return self.lines.get(timeout=.01)
        except queue.Empty:
            return b""

    def wait(self, predicate, timeout=1):
        with self.changed:
            return self.changed.wait_for(lambda: predicate(self.writes), timeout)


class ParsingTests(unittest.TestCase):
    def test_rpm_validation_and_acknowledgement(self):
        self.assertEqual(parse_command("rpm 20.5 -15").text, "RPM 20.5 -15")
        self.assertTrue(parse_command("RPM 0 0").motion)
        self.assertTrue(parse_command("CLEARFAULT").stop)
        self.assertFalse(parse_command("CLEARFAULT").motion)
        for command in ("RPM", "RPM 2", "RPM 2 3 4", "RPM nan 0", "RPM inf 0",
                        "RPM 1_0 0", "RPM 1e999 0", "CLEARFAULT 1"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                parse_command(command)

    def test_m1_frames_remain_readable_without_fabricated_targets(self):
        old = json.loads(frame())
        for field in CONTROL_FIELDS:
            del old[field]
        row = parse_telemetry_line(json.dumps(old))
        self.assertEqual(row["schema_version"], 1)
        self.assertIsNone(row["left_target_rpm"])
        self.assertIsNone(row["sample_seq"])
        self.assertEqual(tuple(row), FIELDS)

    def test_v2_targets_and_sample_identity_preserved(self):
        row = parse_telemetry_line(frame())
        self.assertEqual(row["right_target_rpm"], -20)
        self.assertEqual(row["sample_seq"], 2)
        self.assertEqual(row["control_mode"], "velocity")
        self.assertIsNotNone(parse_telemetry_line(frame(control_mode="stopped", control_fault="no_motion",
                                                     left_pwm=0, right_pwm=0, motion_active=False,
                                                     left_target_rpm=0, right_target_rpm=0)))

    def test_invalid_v2_fields_and_inconsistent_faults_rejected(self):
        for values in ({"schema_version": True}, {"schema_version": 3}, {"sample_seq": -1},
                       {"sample_at_ms": 2**32}, {"sample_seq": True}, {"control_mode": []},
                       {"control_fault": {}}, {"left_target_rpm": float("nan")},
                       {"right_target_rpm": 10**1000}, {"left_target_rpm": True},
                       {"control_mode": "flying"}, {"control_fault": "no_motion"},
                       {"left_rpm": None}, {"motion_active": False}):
            with self.subTest(values=values):
                self.assertIsNone(parse_telemetry_line(frame(**values)))
        missing = json.loads(frame()); del missing["sample_seq"]
        self.assertIsNone(parse_telemetry_line(json.dumps(missing)))


class SessionTests(unittest.TestCase):
    def start(self, log=None, interval=100):
        port = Port(); commands = queue.Queue(); result = []
        seen_frame = threading.Event()
        def output(line):
            if '"type": "telemetry"' in line:
                seen_frame.set()
        worker = threading.Thread(target=lambda: result.append(interactive_session(
            port, lambda _: commands.get(timeout=3), output,
            telemetry_output=log, telemetry_interval=interval)))
        worker.start()
        self.addCleanup(lambda: commands.put("EXIT"))
        self.addCleanup(lambda: worker.join(2))
        self.assertTrue(port.wait(lambda writes: "STOP" in writes))
        return port, commands, result, worker, seen_frame

    def test_control_and_csv_share_one_connection_and_exit_stops(self):
        log = io.StringIO()
        port, commands, result, worker, seen = self.start(log)
        self.assertTrue(port.wait(lambda w: "TELEMETRY 100" in w))
        commands.put("RPM 20 -20")
        self.assertTrue(port.wait(lambda w: "RPM 20 -20" in w))
        port.lines.put(frame())
        self.assertTrue(seen.wait(1))
        commands.put("EXIT"); worker.join(1)
        self.assertFalse(worker.is_alive()); self.assertEqual(result, [0])
        rows = list(csv.DictReader(io.StringIO(log.getvalue())))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["right_target_rpm"], "-20")
        self.assertEqual(port.writes[-1], "STOP")
        self.assertIn("TELEMETRY 0", port.writes)

    def test_firmware_restart_cancels_motion_refresh(self):
        port, commands, result, worker, _ = self.start()
        commands.put("RPM 20 20")
        self.assertTrue(port.wait(lambda w: "RPM 20 20" in w))
        port.lines.put(b"Roboty M2 ready; open-loop commands retained; RPM control is opt-in.\n")
        self.assertTrue(port.wait(lambda w: len(w) >= 3 and w[-1] == "STOP"))
        count = len(port.writes)
        self.assertFalse(port.wait(lambda w: len(w) > count, .25))
        commands.put("EXIT"); worker.join(1)
        self.assertEqual(result, [0])

    def test_fault_telemetry_cancels_motion_even_without_err_line(self):
        port, commands, result, worker, _ = self.start()
        commands.put("RPM 20 20")
        self.assertTrue(port.wait(lambda w: "RPM 20 20" in w))
        port.lines.put(frame(control_mode="stopped", control_fault="stale_feedback", motion_active=False,
                             left_pwm=0, right_pwm=0, left_target_rpm=0, right_target_rpm=0))
        self.assertTrue(port.wait(lambda w: len(w) >= 3 and w[-1] == "STOP"))
        count = len(port.writes)
        self.assertFalse(port.wait(lambda w: len(w) > count, .25))
        commands.put("EXIT"); worker.join(1); self.assertEqual(result, [0])

    def test_logging_error_stops_even_with_blocked_terminal(self):
        class BrokenLog(io.StringIO):
            def write(self, data):
                if not data.startswith("ms,"):
                    raise OSError("Disk full")
                return super().write(data)
        port, commands, result, worker, _ = self.start(BrokenLog())
        commands.put("RPM 20 20")
        self.assertTrue(port.wait(lambda w: "RPM 20 20" in w))
        port.lines.put(frame()); worker.join(1)
        self.assertFalse(worker.is_alive()); self.assertEqual(result, [1])
        self.assertEqual(port.writes[-1], "STOP")
        count = len(port.writes)
        self.assertFalse(port.wait(lambda w: len(w) > count, .25))
        commands.put("EXIT")

    def test_bad_intervals_do_not_write(self):
        for interval in (49, 5001, True):
            port = Port()
            with self.assertRaises(ValueError):
                interactive_session(port, telemetry_interval=interval)
            self.assertEqual(port.writes, [])
        for refresh in (0, -.1, .5, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                CommandRefresher(Port(), refresh)

    def test_existing_log_is_not_overwritten_or_connected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run.csv"; path.write_text("keep me")
            with patch("serial_control.serial.Serial", create=True) as connection:
                self.assertEqual(main(["COM5", "--telemetry-csv", str(path)]), 1)
                connection.assert_not_called()
            self.assertEqual(path.read_text(), "keep me")


if __name__ == "__main__":
    unittest.main()
