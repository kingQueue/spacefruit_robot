"""Start cFS in the required directory with its app and startup script."""
import os
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
runtime = root / "build/cfs/exe/cpu1"
if os.name == "nt":
    raise SystemExit("Run cFS on Linux; use scripts/test_logic.py on Windows.")
try:
    subprocess.run([str(runtime / "core-cpu1")], cwd=runtime, check=True)
except KeyboardInterrupt:
    pass
