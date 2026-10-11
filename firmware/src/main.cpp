#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <hardware/watchdog.h>
#include "motion_protocol.hpp"

namespace {
// Proposed GPIO wiring: each row is EN, IN1, IN2 for one L298N channel.
constexpr unsigned motor_pins[4][3] = {{2,3,4},{5,6,7},{8,9,10},{11,12,13}};
constexpr unsigned enable_input=18; // Pulled high = disabled; closed to GND = enabled.
Adafruit_VL53L1X front(14), rear(15);
robot::MotionProtocol protocol;
robot::MotionSensors sensors;
std::uint32_t front_time=0, rear_time=0, telemetry_time=0;
bool sensors_ready=false, have_front=false, have_rear=false;
char frame[96];
unsigned used=0;
bool discard=false;

void motor(unsigned index,int pwm) {
    const auto* pins=motor_pins[index];
    // Disable channel while changing direction to avoid a transient reverse command.
    analogWrite(pins[0],0);
    digitalWrite(pins[1],pwm>0 ? HIGH : LOW);
    digitalWrite(pins[2],pwm<0 ? HIGH : LOW);
    analogWrite(pins[0],pwm<0 ? -pwm : pwm);
}
void apply(robot::MotionOutput output) {
    motor(0,output.left_pwm); motor(1,output.left_pwm);
    motor(2,output.right_pwm); motor(3,output.right_pwm);
}
void invalidate_command(std::uint32_t now) {
    char bad[]="invalid";
    (void)protocol.receive(bad,now,sensors);
    apply(protocol.poll(now,sensors));
}
void read_ranges(std::uint32_t now) {
    if (sensors_ready && front.dataReady()) {
        sensors.front_mm=front.distance();
        have_front=sensors.front_mm>=30;
        front_time=now;
        if (!front.clearInterrupt()) have_front=false;
    }
    if (sensors_ready && rear.dataReady()) {
        sensors.rear_mm=rear.distance();
        have_rear=sensors.rear_mm>=30;
        rear_time=now;
        if (!rear.clearInterrupt()) have_rear=false;
    }
    sensors.front_age_ms=have_front ? now-front_time : 1000;
    sensors.rear_age_ms=have_rear ? now-rear_time : 1000;
    sensors.emergency=digitalRead(enable_input)!=LOW;
}
void serial_input(std::uint32_t now) {
    // A bounded per-loop budget keeps sensor and watchdog updates running under flood.
    for (unsigned budget=0;budget<32 && Serial.available();++budget) {
        const int ch=Serial.read();
        if (ch=='\r') continue;
        if (ch=='\n') {
            if (!discard && used) {
                frame[used]='\0';
                (void)protocol.receive(frame,now,sensors);
            }
            used=0; discard=false;
        } else if (ch==0 || ch<32 || ch>126 || used==sizeof(frame)-1) {
            discard=true; used=0; invalidate_command(now);
        } else if (!discard) frame[used++]=static_cast<char>(ch);
    }
}
void telemetry(std::uint32_t now,robot::MotionOutput output) {
    if (now-telemetry_time<100 || Serial.availableForWrite()<96) return;
    telemetry_time=now;
    char body[80], line[96];
    const int count=snprintf(body,sizeof(body),"T %lu %d %d %lu %lu %u %u",
        static_cast<unsigned long>(protocol.acknowledged_sequence()),sensors.front_mm,sensors.rear_mm,
        static_cast<unsigned long>(sensors.front_age_ms),static_cast<unsigned long>(sensors.rear_age_ms),
        output.armed?1u:0u,static_cast<unsigned>(output.fault));
    if (count<0 || static_cast<unsigned>(count)>=sizeof(body)) return;
    const auto checksum=robot::crc16(body,static_cast<unsigned>(count));
    const int length=snprintf(line,sizeof(line),"%s*%04X\n",body,checksum);
    if (length>0 && static_cast<unsigned>(length)<sizeof(line)) Serial.write(line,length);
}
}
void setup() {
    analogWriteFreq(1000);
    analogWriteRange(255);
    for (const auto& pins:motor_pins) for (auto pin:pins) {pinMode(pin,OUTPUT);digitalWrite(pin,LOW);}
    apply({});
    pinMode(enable_input,INPUT_PULLUP);
    pinMode(14,OUTPUT);pinMode(15,OUTPUT);digitalWrite(14,LOW);digitalWrite(15,LOW);
    Serial.begin(115200); // USB CDC; do not wait for the host to open it.
    Wire.setSDA(0);Wire.setSCL(1);Wire.begin();
    watchdog_enable(1000,true);
    sensors_ready=front.begin(0x30,&Wire) && rear.begin(0x31,&Wire)
        && front.setTimingBudget(50) && rear.setTimingBudget(50)
        && front.startRanging() && rear.startRanging();
}
void loop() {
    const auto now=static_cast<std::uint32_t>(millis());
    read_ranges(now);
    serial_input(now);
    const auto output=protocol.poll(now,sensors);
    apply(output);
    telemetry(now,output);
    watchdog_update();
}
