from __future__ import annotations

import pathlib
import queue
import sys
import threading
import time
import unittest

import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from serial_control import CommandRefresher, interactive_session, parse_command


class FakeSerial:
    def __init__(self) -> None:
        self.writes: list[str] = []
        self.condition = threading.Condition()
        self.lines: queue.Queue[bytes | Exception] = queue.Queue()

    def write(self, data: bytes) -> int:
        with self.condition:
            self.writes.append(data.decode("ascii").rstrip("\n"))
            self.condition.notify_all()
        return len(data)

    def readline(self, size: int = -1) -> bytes:
        try:
            line = self.lines.get(timeout=0.01)
        except queue.Empty:
            return b""
        if isinstance(line, Exception):
            raise line
        return line

    def wait_for(self, predicate, timeout: float = 1.0) -> bool:
        with self.condition:
            return self.condition.wait_for(lambda: predicate(self.writes), timeout)


class ProtocolTests(unittest.TestCase):
    def test_exact_direction_aliases_and_default_speed(self) -> None:
        for alias, canonical in (("F", "FWD"), ("B", "BACK"), ("L", "LEFT"), ("R", "RIGHT")):
            with self.subTest(alias=alias):
                self.assertEqual(parse_command(alias.lower()).text, canonical)
                self.assertTrue(parse_command(alias).motion)
                self.assertEqual(parse_command(alias + " 25").text, canonical + " 25")

    def test_rejects_dangerous_or_ambiguous_commands(self) -> None:
        for command in ("FWDFAST", "FWD garbage", "FWD -1", "FWD 101",
                        "FWD 20 extra", "STOP 20", "MOTOR 20", "MOTOR 0 0 extra",
                        "MOTOR 4294967296 0", "DRIVE nan 0", "DRIVE 0 inf",
                        "DRIVE 1.1 0", "DRIVE 0 0 extra", "DRIVE 0_1 0",
                        "STATUS extra", "FWD 20\nMOTOR 100 100", "FWD 20\r",
                        "FWD \x0030", "FWD " + "0" * 96, "ＦＷＤ 20",
                        "TELEMETRY 1", "TELEMETRY 5001", "TELEMETRY -1"):
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    parse_command(command)

    def test_telemetry_queries_are_not_motion(self) -> None:
        for command in ("STATUS", "HELP", "?", "TELEMETRY", "TELEMETRY 0",
                        "TELEMETRY 50", "TELEMETRY 5000"):
            with self.subTest(command=command):
                self.assertFalse(parse_command(command).motion)
        self.assertTrue(parse_command("S").stop)


class RefresherTests(unittest.TestCase):
    def test_queries_preserve_motion_and_invalid_commands_stop_it(self) -> None:
        port = FakeSerial()
        refresher = CommandRefresher(port, refresh_seconds=0.01)
        self.addCleanup(refresher.stop)
        refresher.start()
        refresher.submit("F 25")
        self.assertTrue(port.wait_for(lambda writes: writes.count("FWD 25") >= 2))
        refresher.submit("STATUS")
        self.assertTrue(port.wait_for(lambda writes: "FWD 25" in writes[writes.index("STATUS") + 1:]))
        with self.assertRaises(ValueError):
            refresher.submit("FWD nonsense")
        stopped_at = len(port.writes)
        self.assertEqual(port.writes[-1], "STOP")
        self.assertFalse(port.wait_for(lambda writes: len(writes) > stopped_at, timeout=0.04))
        refresher.submit("B")
        self.assertEqual(port.writes[-1], "BACK")

    def test_stop_cannot_be_overtaken_by_an_in_flight_refresh(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        class BlockingPort(FakeSerial):
            def write(self, data: bytes) -> int:
                if data.startswith(b"FWD") and self.writes.count("FWD 30") == 1:
                    entered.set()
                    if not release.wait(1.0):
                        raise TimeoutError("Test did not release in-flight write.")
                return super().write(data)

        port = BlockingPort()
        refresher = CommandRefresher(port, refresh_seconds=0.01)
        self.addCleanup(refresher.stop)
        refresher.start()
        refresher.submit("FWD 30")
        self.assertTrue(entered.wait(1.0))
        stop_thread = threading.Thread(target=lambda: refresher.submit("STOP"))
        stop_thread.start()
        release.set()
        stop_thread.join(timeout=1.0)
        self.assertFalse(stop_thread.is_alive())
        stop_index = port.writes.index("STOP")
        self.assertFalse(port.wait_for(lambda writes: len(writes) > stop_index + 1, timeout=0.04))
        self.assertEqual(port.writes[-1], "STOP")
        refresher.stop()
        writes_after_shutdown = list(port.writes)
        refresher.stop()
        self.assertEqual(port.writes, writes_after_shutdown)

    def test_failed_or_partial_write_disables_refresh(self) -> None:
        for partial in (False, True):
            with self.subTest(partial=partial):
                class FailingPort(FakeSerial):
                    def write(self, data: bytes) -> int:
                        super().write(data)
                        if data.startswith(b"MOTOR"):
                            if partial:
                                return 2
                            raise serial.SerialException("USB disconnected")
                        return len(data)

                port = FailingPort()
                refresher = CommandRefresher(port, refresh_seconds=0.01)
                refresher.start()
                with self.assertRaises(serial.SerialException):
                    refresher.submit("MOTOR 20 20")
                refresher.stop()
                writes_after_shutdown = list(port.writes)
                self.assertTrue(refresher.stopped)
                self.assertIsNotNone(refresher.error)
                self.assertFalse(port.wait_for(lambda writes: writes != writes_after_shutdown, timeout=0.04))
                self.assertEqual(port.writes[-1], "STOP")

    def test_abort_after_read_failure_ends_motion(self) -> None:
        port = FakeSerial()
        refresher = CommandRefresher(port, refresh_seconds=0.01)
        refresher.start()
        refresher.submit("LEFT 20")
        refresher.abort(serial.SerialException("Read failure"))
        self.assertTrue(refresher.stopped)
        self.assertEqual(port.writes[-1], "STOP")
        with self.assertRaises(serial.SerialException):
            refresher.submit("FWD 100")


class SessionTests(unittest.TestCase):
    def test_eof_and_ctrl_c_send_final_stop(self) -> None:
        for end in (EOFError, KeyboardInterrupt):
            with self.subTest(end=end):
                port = FakeSerial()
                first = True

                def terminal(prompt):
                    nonlocal first
                    if first:
                        first = False
                        return "FWD 20"
                    raise end()

                self.assertEqual(interactive_session(port, terminal, lambda _: None), 0)
                self.assertIn("FWD 20", port.writes)
                self.assertEqual(port.writes[-1], "STOP")

    def test_read_failure_exits_while_terminal_is_blocked(self) -> None:
        port = FakeSerial()
        release_terminal = threading.Event()
        result = []

        def terminal(prompt):
            release_terminal.wait(2.0)
            raise EOFError()

        worker = threading.Thread(target=lambda: result.append(
            interactive_session(port, terminal, lambda _: None)))
        worker.start()
        self.assertTrue(port.wait_for(lambda writes: "STOP" in writes))
        port.lines.put(serial.SerialException("Read disconnected"))
        worker.join(timeout=1.0)
        release_terminal.set()
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [1])
        self.assertEqual(port.writes[-1], "STOP")

    def test_firmware_error_stops_refresh_before_next_input(self) -> None:
        port = FakeSerial()
        commands: queue.Queue[str] = queue.Queue()
        result = []
        commands.put("FWD 35")
        worker = threading.Thread(target=lambda: result.append(
            interactive_session(port, lambda _: commands.get(timeout=2.0), lambda _: None)))
        worker.start()
        self.assertTrue(port.wait_for(lambda writes: "FWD 35" in writes))
        port.lines.put(b"ERR command rejected; motors stopped\n")
        self.assertTrue(port.wait_for(lambda writes: len(writes) >= 3 and writes[-1] == "STOP"))
        stopped_at = len(port.writes)
        self.assertFalse(port.wait_for(lambda writes: len(writes) > stopped_at, timeout=0.25))
        commands.put("EXIT")
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [0])
        self.assertEqual(port.writes[-1], "STOP")


if __name__ == "__main__":
    unittest.main()
