#include "motion_protocol.hpp"
#include <cstring>
#include <limits>

namespace robot {
namespace {
bool number(const char*& p, std::uint32_t& value) noexcept {
    if (*p < '0' || *p > '9') return false;
    value = 0;
    while (*p >= '0' && *p <= '9') {
        const auto digit = static_cast<std::uint32_t>(*p++ - '0');
        if (value > (std::numeric_limits<std::uint32_t>::max() - digit) / 10) return false;
        value = value * 10 + digit;
    }
    return true;
}
bool space(const char*& p) noexcept {
    if (*p != ' ') return false;
    ++p;
    return true;
}
bool pwm(const char*& p, int& value) noexcept {
    const bool negative = *p == '-';
    if (negative) ++p;
    std::uint32_t magnitude;
    if (!number(p, magnitude) || magnitude > MotionGuard::max_pwm) return false;
    value = negative ? -static_cast<int>(magnitude) : static_cast<int>(magnitude);
    return true;
}
}
bool MotionProtocol::receive(char* frame, std::uint32_t now, const MotionSensors& sensors) noexcept {
    if (!frame || !validate_frame(frame)) { guard_.stop(MotionFault::command); return false; }
    // A checksum-valid STOP always stops, including an old or malformed sequence.
    if (frame[0] == 'S' && (frame[1] == ' ' || frame[1] == '\0')) {
        guard_.stop();
        const char* p = frame + 1;
        std::uint32_t seq;
        if (!space(p) || !number(p, seq) || *p || seq <= acknowledged_) return false;
        acknowledged_ = seq;
        return true;
    }
    const char operation = frame[0];
    const char* p = frame + 1;
    std::uint32_t seq = 0, ttl = 0;
    int left = 0, right = 0;
    bool valid = space(p) && number(p, seq) && seq > acknowledged_;
    if (valid && operation == 'A') valid = *p == '\0' && guard_.arm(seq, now, sensors);
    else if (valid && operation == 'D') {
        valid = space(p) && pwm(p, left) && space(p) && pwm(p, right)
            && space(p) && number(p, ttl) && *p == '\0'
            && guard_.drive(seq, left, right, ttl, now, sensors);
    } else valid = false;
    if (!valid) { guard_.stop(MotionFault::command); return false; }
    acknowledged_ = seq;
    return true;
}
MotionOutput MotionProtocol::poll(std::uint32_t now, const MotionSensors& sensors) noexcept {
    return guard_.poll(now, sensors);
}
}
