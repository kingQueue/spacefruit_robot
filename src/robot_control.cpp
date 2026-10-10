#include "robot_control.hpp"
namespace robot {
Decision Controller::update(unsigned moisture) noexcept {
    ++samples_;
    const bool valid = moisture <= 100;
    if (!valid) { ++invalid_; pump_ = false; }
    else if (moisture < dry_percent) pump_ = true;
    else if (moisture >= wet_percent) pump_ = false;
    return {moisture, pump_, valid};
}
}
