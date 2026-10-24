#!/usr/bin/env python3
"""Compile default and opt-in firmware; never upload or modify real configuration."""
from __future__ import annotations
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/esp32"


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    if text.count(old) != 1:
        raise RuntimeError(f"Expected one test configuration marker in {path.name}: {old}")
    path.write_text(text.replace(old, new), encoding="utf-8")


def main() -> None:
    subprocess.run(["pio", "run", "-d", str(FIRMWARE)], check=True)
    with tempfile.TemporaryDirectory(prefix="roboty-enabled-build-") as folder:
        target = Path(folder) / "esp32"
        shutil.copytree(FIRMWARE, target, ignore=shutil.ignore_patterns(".pio", ".venv", "__pycache__"))
        robot = target / "include/config/RobotConfig.h"
        replace_once(robot, "ENCODERS_ENABLED = false", "ENCODERS_ENABLED = true")
        for name, value in (("LEFT_ENCODER_A_PIN", 21), ("LEFT_ENCODER_B_PIN", 22),
                            ("RIGHT_ENCODER_A_PIN", 34), ("RIGHT_ENCODER_B_PIN", 35)):
            replace_once(robot, f"{name} = -1", f"{name} = {value}")
        for wheel in ("LEFT", "RIGHT"):
            replace_once(robot, f"{wheel}_ENCODER_COUNTS_PER_REVOLUTION = 0.0F",
                         f"{wheel}_ENCODER_COUNTS_PER_REVOLUTION = 1440.0F")
        control = target / "include/config/VelocityConfig.h"
        replace_once(control, "CLOSED_LOOP_ENABLED = false", "CLOSED_LOOP_ENABLED = true")
        for name, value in (("VELOCITY_MAX_TARGET_RPM", "100.0"),
                            ("VELOCITY_MAX_FEEDBACK_RPM", "200.0"),
                            ("VELOCITY_MAX_PWM_PERCENT", "40.0")):
            replace_once(control, f"{name} = 0.0", f"{name} = {value}")
        for wheel in ("LEFT", "RIGHT"):
            replace_once(control, f"{wheel}_VELOCITY_PID{{0.0, 0.0, 0.0, 0.0, 0.02}}",
                         f"{wheel}_VELOCITY_PID{{0.5, 0.1, 0.01, 0.1, 0.02}}")
        print("Compiling synthetic opt-in configuration. These values are NOT hardware calibration.", flush=True)
        subprocess.run(["pio", "run", "-d", str(target)], check=True)
    print("PASS default and opt-in ESP32 builds; repository configuration unchanged")


if __name__ == "__main__":
    main()
