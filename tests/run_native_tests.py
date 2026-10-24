#!/usr/bin/env python3
"""Compile and run the real firmware logic on a host, without an ESP32."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "esp32"


def main() -> None:
    compiler = shlex.split(os.environ.get("CXX", "g++"))
    common = compiler + ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    if os.environ.get("SANITIZE") == "1":
        common += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    common += ["-I", str(ROOT / "tests/native/fakes"), "-I", str(FIRMWARE / "include")]
    suites = {
        "encoders": [ROOT / "tests/native/test_encoders.cpp"],
        "serial_commands": [
            ROOT / "tests/native/test_serial_commands.cpp",
            FIRMWARE / "src/comms/SerialCommandInterface.cpp",
            FIRMWARE / "src/control/DifferentialDrive.cpp",
            FIRMWARE / "src/drivers/MotorDriver.cpp",
        ],
    }
    with tempfile.TemporaryDirectory(prefix="roboty-tests-") as directory:
        for name, sources in suites.items():
            binary = Path(directory) / name
            subprocess.run(common + [str(source) for source in sources] + ["-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
        output = subprocess.check_output([str(Path(directory) / "serial_commands"), "--emit-telemetry"], text=True)
        records = [json.loads(line) for line in output.splitlines()]
        assert len(records) == 3
        assert records[0]["encoders"] is False and records[0]["left_rpm"] is None
        assert records[1]["left_ticks"] == (1 << 63) - 1
        assert records[1]["right_ticks"] == -(1 << 63)
        assert records[1]["left_rpm"] == 60.25 and records[1]["right_rpm"] == -60.25
        assert records[2]["left_rpm"] == 1e200
        print("PASS firmware JSON telemetry round-trip (3 frames)")


if __name__ == "__main__":
    main()
