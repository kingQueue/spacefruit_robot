"""Compile and execute the actual portable C controller on Windows or Linux."""
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / "build" / "tests"
out.mkdir(parents=True, exist_ok=True)
exe = out / ("robot_control_test.exe" if os.name == "nt" else "robot_control_test")
subprocess.run([os.environ.get("CC", "gcc"), "-std=c99", "-Wall", "-Wextra",
                "-Werror", "-pedantic", "-I", str(root / "apps/robot_app/fsw/inc"),
                str(root / "apps/robot_app/fsw/src/robot_control.c"),
                str(root / "tests/test_robot_control.c"), "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
