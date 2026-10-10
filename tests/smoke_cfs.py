"""Boot real cFS and require a loaded app plus an ordered control sequence."""
from pathlib import Path
import subprocess
import tempfile
import time

root = Path(__file__).resolve().parents[1]
runtime = root / "build/cfs/exe/cpu1"
expected = ["ROBOT_APP ready:",
            "ROBOT_APP sample=1 moisture=20 pump=1",
            "ROBOT_APP sample=2 moisture=35 pump=1",
            "ROBOT_APP sample=3 moisture=50 pump=0"]
with tempfile.TemporaryFile(mode="w+b") as log:
    process = subprocess.Popen([str(runtime / "core-cpu1")], cwd=runtime,
                               stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            log.seek(0)
            output = log.read().decode("utf-8", errors="replace")
            positions = [output.find(marker) for marker in expected]
            if "OPERATIONAL" in output and all(p >= 0 for p in positions) and positions == sorted(positions):
                print("PASS: cFE operational; robot app loaded; dry/hold/wet sequence executed")
                break
            if process.poll() is not None:
                raise AssertionError("cFS exited before verification:\n" + output)
            time.sleep(0.25)
        else:
            raise AssertionError("cFS startup/control sequence timed out:\n" + output)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.seek(0)
        (root / "build/smoke.log").write_bytes(log.read())
