"""Verify real FreeRTOS tasks deliver exact messages, then terminate cleanly."""
import subprocess
import sys
exe = sys.argv[1]
run = subprocess.run([exe, "--samples", "8"], capture_output=True, text=True, timeout=10)
assert run.returncode == 0, run.stderr + run.stdout
cycle = [(20, 1, 1), (35, 1, 1), (50, 0, 1), (101, 0, 0)]
expected = [f"sample={i+1} moisture={m} pump={p} valid={v}"
            for i, (m, p, v) in enumerate(cycle * 2)]
assert run.stdout.splitlines() == expected, run.stdout
for args in [["--samples", "0"], ["--samples", "-1"], ["--samples", "bad"], ["--unknown"]]:
    bad = subprocess.run([exe, *args], capture_output=True, timeout=3)
    assert bad.returncode == 2, args
print("PASS: real FreeRTOS sensor/control/logger queues, ordering, shutdown, CLI")
