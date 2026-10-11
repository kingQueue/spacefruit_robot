#pragma once
#include <cstdint>
#include "motion_guard.hpp"

namespace robot {
// Only complete, checked frames may update the guard. Invalid input disarms it.
class MotionProtocol {
public:
    bool receive(char* frame, std::uint32_t now, const MotionSensors& sensors) noexcept;
    MotionOutput poll(std::uint32_t now, const MotionSensors& sensors) noexcept;
    // High-water mark of parsed commands, not proof of actuator success.
    std::uint32_t acknowledged_sequence() const noexcept { return acknowledged_; }
private:
    MotionGuard guard_;
    std::uint32_t acknowledged_ = 0;
};
}
