"""Portable C++ tests, without needing CMake or FreeRTOS on Windows."""
import os
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
out = root / "build/tests"
out.mkdir(parents=True, exist_ok=True)
exe = out / ("test_robot_control.exe" if os.name == "nt" else "test_robot_control")
subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra",
                "-Werror", "-pedantic", "-I", str(root / "include"),
                str(root / "src/robot_control.cpp"), str(root / "tests/test_robot_control.cpp"),
                "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)
