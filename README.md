# Spacefruit robot — C++ FreeRTOS starter

A small robot development sample using **FreeRTOS Kernel V11.2.0** and its
POSIX/Linux port, pinned to commit `0adc196d4bd52a2d91102b525b0aafc1e14a2386`.
Application code is C++17; the upstream FreeRTOS kernel is C.
There are no NASA framework components in the current source tree.

This runs actual FreeRTOS tasks, a preemptive priority scheduler and queues
**inside a Linux process** on Raspberry Pi OS. It is a development/simulation
port, not a bare-metal Pi 5 operating system or a hard real-time guarantee.
Linux still controls hardware and process scheduling.

## Build and run on Raspberry Pi 5

Use 64-bit Raspberry Pi OS. Install tools and clone:

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3
git clone --recurse-submodules https://github.com/kingQueue/spacefruit_robot.git
cd spacefruit_robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/robot_demo
```

Stop with Ctrl+C. For a finite run:

```bash
./build/robot_demo --samples 8
```

If updating an existing clone, stop the old runtime, then:

```bash
git pull --ff-only
git submodule update --init --recursive
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Old build outputs or an orphaned vendor directory can remain in an existing
checkout after an update; they are not part of the new tracked source.
A fresh recursive clone avoids them.

Why each step:
1. The tools provide C/C++ compilers, CMake, Git and the integration test runner.
2. Recursive cloning retrieves the exact pinned RTOS dependency.
3. CMake configures the native Linux build: ARM64 on your Pi, x86-64 on a PC.
4. Compilation builds the kernel and your C++ application.
5. CTest checks controller behavior and executes the real scheduler/task pipeline.
6. The executable starts three FreeRTOS tasks and feeds simulated readings.

## Tasks and data flow

```text
sensor (priority 1, every 200 ms)
    → bounded sensor queue
control (priority 2, waits for readings)
    → bounded report queue
logger (priority 3, waits for reports)
    → console
```

Higher numbers mean higher FreeRTOS priorities. Tasks block on queues rather than
busy-waiting. The sensor uses `xTaskDelayUntil` for periodic scheduling.
Only the logger performs console I/O, following the POSIX port's libc-lock guidance.
Queue messages are plain structs copied by value; do not enqueue owning C++ objects
such as `std::string` directly.

The controller requests a pump below 30% moisture, turns the request off at 45%,
and retains its prior state between those thresholds. Readings above 100% are
invalid and clear the request. No physical sensors or pump are connected.

Example output:
```text
sample=1 moisture=20 pump=1 valid=1
sample=2 moisture=35 pump=1 valid=1
sample=3 moisture=50 pump=0 valid=1
sample=4 moisture=101 pump=0 valid=0
```

## Edit and test

- `src/main.cpp`: task entry points, priorities, period, simulated inputs, queues.
- `src/robot_control.cpp`: hardware-independent decision logic.
- `include/robot_control.hpp`: thresholds and controller state.
- `config/FreeRTOSConfig.h`: kernel tick, task limits, heap and enabled APIs.
- `tests/test_robot_control.cpp`: boundaries, hysteresis, invalid values, reset.
- `tests/test_runtime.py`: actual queued task messages, ordering and finite shutdown.

After editing, rebuild and rerun CTest before running the demo. Intentionally
changing the demo sequence may require changing integration-test expectations.
GitHub Actions builds and runs both tests on Linux x86-64 and ARM64.
ARM64 CI checks the CPU architecture, not physical Pi sensors or timing.

## Windows development

With G++ and Python available:
```powershell
python scripts/test_logic.py
```

The full RTOS demo needs Linux (Pi, WSL or a Linux development host).
With CMake you can build just portable tests on other platforms using
`-DROBOT_BUILD_RTOS=OFF`.

## Hardware next steps

The SpaceFruit garden workflow now has a C++ mission controller in
`include/mission.hpp` / `src/mission.cpp`. It mirrors setup, editable-plan
confirmation, exact seed loading, planting, care checks, readiness-gated
harvesting and empty-cell replanting. Each hardware job has an ID and deadline;
unverified results, timeouts and unsafe sensing stop the mission. See
`docs/app-workflow.md` for the observed app sequence and integration contract.

Run the full flow through two real FreeRTOS tasks and queues:

```bash
./build/mission_demo --simulate
```

This explicit simulation plants Radish/Onion, checks both, harvests Radish and
replants its cell with Carrot. It does not operate hardware. There is no implicit
simulation fallback when physical hardware is unavailable.

`firmware/` contains an RP2040 target for the four TT/L298N motor channels and
two VL53L1X sensors with command expiry, arming and a watchdog. Its driver loop
uses Arduino; it is separate from the Pi FreeRTOS mission runtime. See
`firmware/README.md` for proposed wiring and build instructions. Physical camera
localization and the app-to-robot transport still need integration. The gripper
is confirmed, but its servo calibration/arm geometry are unknown; seed and weed
tools have not been designed, so those physical actions remain unavailable.

Add sensor and output adapters while retaining independently tested logic.
The sample is not a complete actuator control system. For strict timing or
bare-metal operation, use an RTOS-supported hardware target and its hardware port;
the Linux POSIX port does not supply Pi GPIO drivers.

FreeRTOS remains under its upstream MIT license in
`vendor/FreeRTOS-Kernel/LICENSE.md`. Upstream port implementation:
https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.2.0/portable/ThirdParty/GCC/Posix
