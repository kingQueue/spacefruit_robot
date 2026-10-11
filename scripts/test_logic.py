"""Portable C++ tests, without needing CMake or FreeRTOS on Windows."""
import os
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
out = root / "build/tests"
out.mkdir(parents=True, exist_ok=True)
suites = {
    "test_robot_control": ["robot_control"],
    "test_navigation": ["navigation"],
    "test_motion_guard": ["motion_guard"],
    "test_motion_protocol": ["motion_guard", "motion_protocol"],
    "test_mission": ["mission"],
}
for name, sources in suites.items():
    exe = out / (name + (".exe" if os.name == "nt" else ""))
    subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Werror", "-pedantic", "-I", str(root / "include"),
                    *[str(root / f"src/{source}.cpp") for source in sources],
                    str(root / f"tests/{name}.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print(f"PASS {name}", flush=True)
