"""Verify the full mission executes through real FreeRTOS tasks and queues."""
import subprocess
import sys

binary = sys.argv[1]
refused = subprocess.run([binary], capture_output=True, text=True, timeout=5)
assert refused.returncode == 2
run = subprocess.run([binary, "--simulate"], capture_output=True, text=True, timeout=10)
assert run.returncode == 0, run.stdout + run.stderr
actions = [line.split()[1].split("=")[1] for line in run.stdout.splitlines() if line.startswith("SIM action=")]
assert actions == [
    "map_plot", "read_environment",
    "navigate", "plant_seed", "navigate", "plant_seed",
    "navigate", "inspect_weeds", "remove_weeds", "inspect_health",
    "navigate", "inspect_weeds", "remove_weeds", "inspect_health",
    "navigate", "inspect_readiness", "harvest", "navigate", "plant_seed",
], actions
assert "SIM complete growing=2 harvested_radish=1 checked=2 replanted=Carrot" in run.stdout
print("PASS: actual FreeRTOS mission task/queue workflow and explicit simulation gate")
