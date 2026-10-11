#include "navigation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
namespace {
constexpr double pi = 3.14159265358979323846;
bool positive(double v) { return std::isfinite(v) && v > 0; }
double wrap(double angle) { return std::remainder(angle, 2 * pi); }
bool fresh(std::uint64_t stamp, std::uint64_t now, std::uint64_t age) {
    return stamp <= now && now - stamp <= age;
}
bool pose_valid(const Pose& p) {
    return p.valid && std::isfinite(p.x_m) && std::isfinite(p.y_m) && std::isfinite(p.yaw_rad);
}
bool range_valid(const Range& r, std::uint64_t now, std::uint64_t age) {
    return r.valid && positive(r.distance_m) && fresh(r.timestamp_ms, now, age);
}
}
Navigator::Navigator(NavigationConfig config) : config_(config) {
    if (!positive(config.track_m) || !positive(config.max_wheel_mps) ||
        !positive(config.position_tolerance_m) || !positive(config.heading_tolerance_rad) ||
        config.heading_tolerance_rad >= pi || !positive(config.stop_distance_m) ||
        !positive(config.brake_deceleration_mps2) || !positive(config.heading_gain) ||
        config.max_age_ms == 0) throw std::invalid_argument("Invalid navigation calibration");
}
DriveCommand Navigator::update(const Waypoint& goal, const NavigationInput& input,
                               std::uint64_t now) const noexcept {
    if (input.emergency_stop || input.bumper_pressed) return {};
    if (!std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) || !pose_valid(input.pose) ||
        !fresh(input.pose.timestamp_ms, now, config_.max_age_ms) ||
        !range_valid(input.front, now, config_.max_age_ms) ||
        !range_valid(input.rear, now, config_.max_age_ms))
        return {0, 0, NavigationStatus::invalid_data};
    const double dx = goal.x_m - input.pose.x_m;
    const double dy = goal.y_m - input.pose.y_m;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) return {0, 0, NavigationStatus::invalid_data};
    if (distance <= config_.position_tolerance_m) return {0, 0, NavigationStatus::arrived};
    const double heading_error = wrap(std::atan2(dy, dx) - input.pose.yaw_rad);
    const double angular = config_.heading_gain * heading_error;
    double left, right;
    // Rotate first. Never reverse into unobserved space to reach a waypoint.
    if (std::abs(heading_error) > config_.heading_tolerance_rad) {
        if (!input.turning_clearance) return {0, 0, NavigationStatus::no_turn_clearance};
        if (std::min(input.front.distance_m, input.rear.distance_m) <= config_.stop_distance_m)
            return {0, 0, NavigationStatus::obstacle};
        left = -angular * config_.track_m / 2;
        right = -left;
    } else {
        // Bound speed by stopping distance and worst-case sensor age.
        const double clearance = input.front.distance_m - config_.stop_distance_m;
        if (clearance <= 0) return {0, 0, NavigationStatus::obstacle};
        const double reaction = static_cast<double>(config_.max_age_ms) / 1000;
        const double a = config_.brake_deceleration_mps2;
        const double safe_speed = std::sqrt(a * a * reaction * reaction + 2 * a * clearance) - a * reaction;
        const double speed = std::min({config_.max_wheel_mps, distance, safe_speed});
        // Steer only with a map that establishes side clearance.
        const double correction = input.turning_clearance ? angular * config_.track_m / 2 : 0;
        left = speed - correction;
        right = speed + correction;
    }
    const double scale = std::max({1.0, std::abs(left) / config_.max_wheel_mps,
                                  std::abs(right) / config_.max_wheel_mps});
    return {left / scale, right / scale, NavigationStatus::moving};
}
Pose integrate_odometry(const Pose& previous, double left, double right,
                        double track, std::uint64_t now) {
    if (!pose_valid(previous) || !std::isfinite(left) || !std::isfinite(right) ||
        !positive(track) || now < previous.timestamp_ms)
        throw std::invalid_argument("Invalid odometry input");
    const double angle = (right - left) / track;
    const double distance = (right + left) / 2;
    if (!std::isfinite(angle) || !std::isfinite(distance))
        throw std::invalid_argument("Odometry overflow");
    const double half = angle / 2;
    const double sinc = std::abs(half) < 1e-8 ? 1.0 : std::sin(half) / half;
    Pose result{previous.x_m + distance * sinc * std::cos(previous.yaw_rad + half),
                previous.y_m + distance * sinc * std::sin(previous.yaw_rad + half),
                wrap(previous.yaw_rad + angle), now, true};
    if (!pose_valid(result)) throw std::invalid_argument("Odometry overflow");
    return result;
}
}
