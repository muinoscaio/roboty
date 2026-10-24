#!/usr/bin/env python3
"""Run M2 control math and real serial/driver integration without hardware."""
from __future__ import annotations
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/esp32"


def main() -> None:
    flags = shlex.split(os.environ.get("CXX", "g++")) + [
        "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(ROOT / "tests/native/fakes"), "-I", str(FIRMWARE / "include")]
    if os.environ.get("SANITIZE") == "1":
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-no-pie"]
    suites = {
        "velocity": [ROOT / "tests/native/test_velocity.cpp"],
        "velocity_serial": [ROOT / "tests/native/test_velocity_serial.cpp",
                            FIRMWARE / "src/comms/SerialCommandInterface.cpp",
                            FIRMWARE / "src/control/DifferentialDrive.cpp",
                            FIRMWARE / "src/drivers/MotorDriver.cpp"]}
    with tempfile.TemporaryDirectory(prefix="roboty-m2-") as folder:
        for name, sources in suites.items():
            binary = Path(folder) / name
            subprocess.run(flags + list(map(str, sources)) + ["-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
        output = subprocess.check_output([str(Path(folder) / "velocity_serial"), "--emit-telemetry"], text=True)
        records = [json.loads(line) for line in output.splitlines()]
        assert len(records) == 4 and all(row["schema_version"] == 2 for row in records)
        assert records[0]["control_mode"] == "stopped"
        assert records[1]["control_mode"] == "velocity" and records[1]["motion_active"]
        assert records[1]["left_target_rpm"] == 20 and records[1]["right_target_rpm"] == -15
        assert records[1]["sample_seq"] == 2 and records[1]["sample_at_ms"] == 200
        assert records[2]["control_fault"] == "encoder_transition"
        assert records[2]["left_pwm"] == records[2]["right_pwm"] == 0
        assert records[3]["control_fault"] == "none" and not records[3]["motion_active"]
        print("PASS M2 real firmware JSON round-trip (4 frames)")


if __name__ == "__main__":
    main()
