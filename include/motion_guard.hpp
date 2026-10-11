#pragma once
#include <cstdint>

namespace robot {
enum class MotionFault { none, disarmed, sensor, obstacle, emergency, timeout, command };
struct MotionSensors {
    int front_mm = -1, rear_mm = -1;
    std::uint32_t front_age_ms = 1000, rear_age_ms = 1000;
    bool emergency = true, bumper = false;
};
struct MotionOutput { int left_pwm = 0, right_pwm = 0; bool armed = false; MotionFault fault = MotionFault::disarmed; };
class MotionGuard {
public:
    bool arm(std::uint32_t seq, std::uint32_t now, const MotionSensors& sensors) noexcept;
    bool drive(std::uint32_t seq, int left, int right, std::uint32_t ttl,
               std::uint32_t now, const MotionSensors& sensors) noexcept;
    void stop(MotionFault reason = MotionFault::disarmed) noexcept;
    MotionOutput poll(std::uint32_t now, const MotionSensors& sensors) noexcept;
    static constexpr int max_pwm = 100;
    static constexpr int stop_mm = 300;
    static constexpr std::uint32_t sensor_max_age_ms = 200;
private:
    MotionFault check(const MotionSensors& sensors) const noexcept;
    bool sequence(std::uint32_t seq) noexcept;
    MotionOutput output_;
    std::uint32_t last_seq_ = 0, updated_ = 0, ttl_ = 300;
};
std::uint16_t crc16(const char* data, unsigned length) noexcept;
// In-place checksum validation. On success, replaces '*' with '\0'.
bool validate_frame(char* line) noexcept;
}
