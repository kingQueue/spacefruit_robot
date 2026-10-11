#include "motion_guard.hpp"
#include <cstring>
namespace robot {
MotionFault MotionGuard::check(const MotionSensors& s) const noexcept {
    if (s.emergency || s.bumper) return MotionFault::emergency;
    if (s.front_mm < 30 || s.rear_mm < 30 || s.front_mm > 4000 || s.rear_mm > 4000 ||
        s.front_age_ms > sensor_max_age_ms || s.rear_age_ms > sensor_max_age_ms) return MotionFault::sensor;
    return MotionFault::none;
}
bool MotionGuard::sequence(std::uint32_t seq) noexcept {
    if (seq == 0 || seq <= last_seq_) return false;
    last_seq_ = seq;
    return true;
}
void MotionGuard::stop(MotionFault fault) noexcept { output_ = {0, 0, false, fault}; }
bool MotionGuard::arm(std::uint32_t seq, std::uint32_t now, const MotionSensors& sensors) noexcept {
    const auto fault = check(sensors);
    if (!sequence(seq) || fault != MotionFault::none) { stop(fault == MotionFault::none ? MotionFault::command : fault); return false; }
    if (sensors.front_mm <= stop_mm || sensors.rear_mm <= stop_mm) { stop(MotionFault::obstacle); return false; }
    updated_ = now; ttl_ = 300; output_ = {0, 0, true, MotionFault::none}; return true;
}
bool MotionGuard::drive(std::uint32_t seq, int left, int right, std::uint32_t ttl,
                        std::uint32_t now, const MotionSensors& sensors) noexcept {
    (void)poll(now, sensors);
    if (!sequence(seq) || left < -max_pwm || left > max_pwm || right < -max_pwm || right > max_pwm ||
        ttl < 100 || ttl > 500) { stop(MotionFault::command); return false; }
    if (!output_.armed) return false;
    output_.left_pwm = left; output_.right_pwm = right; updated_ = now; ttl_ = ttl;
    (void)poll(now, sensors); return output_.armed;
}
MotionOutput MotionGuard::poll(std::uint32_t now, const MotionSensors& sensors) noexcept {
    const auto fault = check(sensors);
    if (fault != MotionFault::none) stop(fault);
    if (output_.armed && static_cast<std::uint32_t>(now - updated_) >= ttl_) stop(MotionFault::timeout);
    if (output_.armed && (((output_.left_pwm > 0 || output_.right_pwm > 0) && sensors.front_mm <= stop_mm) ||
                         ((output_.left_pwm < 0 || output_.right_pwm < 0) && sensors.rear_mm <= stop_mm))) stop(MotionFault::obstacle);
    return output_;
}
std::uint16_t crc16(const char* data, unsigned length) noexcept {
    std::uint16_t crc = 0xffff;
    for (unsigned i = 0; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(static_cast<unsigned char>(data[i])) << 8;
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) : static_cast<std::uint16_t>(crc << 1);
    }
    return crc;
}
bool validate_frame(char* line) noexcept {
    const auto length = std::strlen(line);
    if (length < 6 || length > 95 || line[length - 5] != '*') return false;
    unsigned value = 0;
    for (unsigned i = static_cast<unsigned>(length - 4); i < length; ++i) {
        const char c = line[i];
        unsigned digit;
        if (c >= '0' && c <= '9') digit = static_cast<unsigned>(c - '0');
        else if (c >= 'A' && c <= 'F') digit = static_cast<unsigned>(c - 'A' + 10);
        else return false;
        value = value * 16 + digit;
    }
    if (crc16(line, static_cast<unsigned>(length - 5)) != value) return false;
    line[length - 5] = '\0'; return true;
}
}
