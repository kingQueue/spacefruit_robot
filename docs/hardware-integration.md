# Hardware integration status

The owner's purchased parts override the older SpaceFruit planning documents.

## Selected hardware

- Pi 5, 8 GB, with Geekworm X1202 UPS.
- YD-RP2040, 4 MB flash.
- Four DIANN TT motors, 200 RPM, 3–6 V, 1:48 gearbox, wheels and two L298N boards.
  No encoders were supplied.
- Camera Module 3 Wide for navigation and Standard for arm inspection, with
  Pi 5 adapter cables.
- Two Adafruit VL53L1X sensors, intended front/rear.
- Four arm servos confirmed by the owner. PCA9685 and BME280 are provisional
  carryovers from earlier discussions. Servo models/calibration remain unknown.

Do not follow the previous Motoron, Pololu encoder-motor, BNO085 or alternative
UPS design as the wiring specification for these purchased parts.

## Software implemented locally

The Linux C++ FreeRTOS sample still uses simulated moisture readings. It is not
RP2040 firmware. The portable Navigator implements waypoint steering, freshness
checks, stopping-distance limits, arrival, emergency stops and clearance-gated
turns. Dimensions/braking defaults are provisional.

MotionGuard requires explicit arming, fresh valid front/rear ranges and a clear
emergency input. It limits PWM, stops for obstacles, rejects replayed sequences
and expires commands after 100–500 ms. Timeout requires rearming.

MotionProtocol accepts CRC16-CCITT checksummed ASCII arm, drive and stop frames.
Initial CRC is FFFF; checksum is four uppercase hex digits after '*'. Example
bodies are 'A 1', 'D 2 40 40 200', 'S 3'. The serial transport must provide a
bounded NUL-terminated frame without CR/LF. Bad frames disarm. A checksum-valid
stop stops even when its sequence is old. CRC does not authenticate a sender.

The RP2040 firmware now connects the motion protocol to proposed motor GPIO and ToF wiring. The mission controller and camera/app adapters are not connected to physical hardware yet.
Run python scripts/test_logic.py for all five portable C++ suites. CMake/CTest
also runs them and the Linux FreeRTOS pipeline.

## Physical integration requirements

Navigation must work without markers. No encoder feedback exists, so motor
runtime cannot be treated as measured travel. Camera localization needs measured
metric scale, calibration, confidence and drift bounds. Tracking loss must stop
motion. Front/rear single-point ranges do not establish lateral turn clearance.

Before hardware operation, measure wheel track, motor signs, PWM/speed/braking,
camera intrinsics/extrinsics, traversable space, arm geometry, servo pulse bounds
and tool transforms. Four servos do not define a seed dispenser, weed tool or
harvester. Those mechanisms and operation verification still need specification.

Random simulated app decisions cannot verify weed removal, plant health or
harvest readiness. Use appropriate real models or explicit operator review.
Command acknowledgment alone does not prove seed release or crop collection.
Update profiles and inventory only after verified operation results.

There is no GPS or soil-moisture sensor in the supplied hardware. BME280 measures
air temperature/humidity/pressure. Geographic recommendations require an
operator-supplied location or another positioning source.

The C++ mission sequence and initial MCU motor/ToF drivers are implemented. Camera localization, app transport and physical tool execution remain unfinished. Unit tests cannot replace physical bench verification.
