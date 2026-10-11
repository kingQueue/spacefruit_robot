# RP2040 motor and ToF firmware

This target controls the four purchased TT motors through two L298N boards and
reads the two Adafruit VL53L1X sensors. It uses the same C++ motion guard/protocol
as the native tests. It is a cooperative Arduino driver loop with a hardware
watchdog; the Pi mission demo uses FreeRTOS. This is not a claim that Arduino
firmware is running the FreeRTOS scheduler.

Proposed GPIO wiring (verify YD-RP2040 breakout labels before assembly):

| Device | GPIO |
|---|---|
| I2C SDA/SCL | GP0 / GP1 |
| Left front EN/IN1/IN2 | GP2 / GP3 / GP4 |
| Left rear EN/IN1/IN2 | GP5 / GP6 / GP7 |
| Right front EN/IN1/IN2 | GP8 / GP9 / GP10 |
| Right rear EN/IN1/IN2 | GP11 / GP12 / GP13 |
| Front/rear ToF XSHUT | GP14 / GP15 |
| Physical motion-enable input | GP18, closed to GND permits arming |

Remove L298N EN jumpers before connecting PWM. Use a common ground and a motor
supply appropriate for the 3–6 V TT motors; the UPS's 6–18 V input specification
does not make that voltage safe for the motors. Motors are not powered by Pi GPIO.
Provide hardware pull-downs on EN so channels remain disabled during MCU reset.
The driver does not implement wheel velocity feedback: these motors have no
encoders. PWM values are not metres per second; a Pi adapter must calibrate the
mapping and use camera pose feedback.

Both ToF sensors are held in shutdown, then assigned addresses 0x30 and 0x31.
Missing/invalid/stale ranges prevent arming. GP18 high disables motion. USB loss
expires the last command. Watchdog reset handles a stalled MCU loop; startup
sets outputs off. Front/rear ranges do not verify lateral turning clearance.

Install PlatformIO, then from the repository root:

```bash
pio run -d firmware
```

Use the board's documented BOOTSEL/upload procedure and the resulting firmware
artifact. The conservative Pico flash layout uses 2 MB of the purchased 4 MB;
confirm the board variant, USB and pinout before flashing. No physical board was
bench tested here. Initial motor direction tests need wheels raised and low PWM.

USB commands are newline-delimited `A seq*CRC`, `D seq left right ttl*CRC` and
`S seq*CRC`. Telemetry is `T ack front_mm rear_mm front_age rear_age armed fault*CRC`.
CRC is CCITT-FALSE, initial FFFF, four uppercase hexadecimal digits. PWM magnitude
is limited to 100/255; command TTL is 100–500 ms. The Pi must keep refreshing safe
commands and stop on lost camera localization. ACK is the highest parsed command sequence, including commands rejected by a
safety gate; inspect armed/fault to determine execution. Reconnect chooses ACK+1;
reset disarms and restarts ACK at zero.

Arm, gripper, seed dispenser and weed-removal actions are not implemented by this
firmware. The owner confirmed a gripper but has not designed the other tools.
Those jobs must report unavailable, not simulated success, in hardware mode.
