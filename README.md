# Spacefruit robot — cFS C starter

A Raspberry Pi 5 development mission using NASA cFS v7.0.1, pinned as a
Git submodule. `apps/robot_app` is editable source owned by this repository.
The sample simulates moisture readings and reports a pump **request**; it
does not connect to sensors, drive GPIO, or actuate a pump.

## Raspberry Pi setup and first run

Use Raspberry Pi OS **64-bit**. In a terminal on your Pi:

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3
git clone --recurse-submodules https://github.com/kingQueue/spacefruit_robot.git
cd spacefruit_robot
python3 scripts/test_logic.py
python3 scripts/build.py
python3 tests/smoke_cfs.py
python3 scripts/run.py
```

Stop the foreground run with Ctrl+C. Run only one cFS instance at a time.
If you already cloned without dependencies:

```bash
git submodule update --init --recursive
```

Why these steps:

1. APT installs GCC/Make, CMake, Git and Python. Python scripts coordinate
   the build and tests; the controller and cFS application are written in C.
2. Recursive cloning retrieves the pinned framework and its pinned components.
3. The logic test compiles the real controller with strict warnings and checks
   threshold boundaries, hysteresis, invalid-input fail-off, counters and reset.
4. The build script combines upstream configuration defaults with our mission
   files, configures native Linux compilation and stages cFS and the app.
   Native builds produce ARM64 code when run on your ARM64 Pi; no cross compiler
   or `rpi_linux` target is needed.
5. The smoke test starts the real framework and requires OPERATIONAL startup,
   successful app loading, and the first three expected controller outputs.
6. The run script starts the executable from the required runtime directory.

Expected output includes:

```text
ROBOT_APP ready: simulated moisture control
ROBOT_APP sample=1 moisture=20 pump=1
ROBOT_APP sample=2 moisture=35 pump=1
ROBOT_APP sample=3 moisture=50 pump=0
```

## Edit your C app

* `apps/robot_app/fsw/src/robot_app.c`: cFS entry point, simulated inputs,
  periodic task and diagnostic output.
* `apps/robot_app/fsw/src/robot_control.c`: portable controller logic.
* `apps/robot_app/fsw/inc/robot_control.h`: state and threshold configuration.
* `tests/test_robot_control.c`: controller behavior tests.
* `mission/`: app selection and startup registration.

Below 30% requests pumping; at 45% or above stops requesting it. Between
thresholds the previous state is retained, avoiding rapid on/off switching.
Readings above 100% clear the request and count as invalid.

After editing, stop any running instance and repeat:

```bash
python3 scripts/test_logic.py
python3 scripts/build.py
python3 tests/smoke_cfs.py
python3 scripts/run.py
```

Changing behavior intentionally may require updating the smoke test's expected
sequence. Build output lives under `build/` and is not committed. Modify tracked
`mission/` files, not the generated `build/robot_defs` copy. Use `BUILD_JOBS=1
python3 scripts/build.py` on a memory-constrained machine.

## Development on Windows

With GCC and Python installed, `python scripts/test_logic.py` runs locally.
The cFS Linux PSP needs Linux: use WSL, a Linux host, or the Pi for build/run.
GitHub Actions builds and boots the same mission on Ubuntu after each push.
It does not test physical Pi hardware, GPIO or timing guarantees.

## Growing this into a robot

Replace simulated inputs with a sensor adapter and keep the control logic
independently testable. Add a hardware output adapter, watchdog, command handling
and cFS Software Bus telemetry as needed. This minimal mission starts only
`robot_app`; it does not yet include network commanding or a ground-system GUI.
Linux cFS development does not turn Raspberry Pi OS into a hard real-time OS.
The sample decision rule is not a complete hardware control/safety system.

NASA source and its licenses remain in `vendor/cFS` and its submodules.
See [NASA cFS](https://github.com/nasa/cFS) for upstream documentation.
