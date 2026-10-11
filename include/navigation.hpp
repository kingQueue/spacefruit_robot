#pragma once
#include <cstdint>

namespace robot {
struct Pose {
    double x_m = 0, y_m = 0, yaw_rad = 0;
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};
struct Range {
    double distance_m = 0;
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};
struct NavigationInput {
    Pose pose;
    Range front, rear;
    bool emergency_stop = false;
    bool bumper_pressed = false;
    // Must be supplied by a validated local map/planner. Front/rear ToF
    // alone cannot establish clearance around the sides during a turn.
    bool turning_clearance = false;
};
struct Waypoint { double x_m, y_m; };
enum class NavigationStatus { moving, arrived, stopped, invalid_data, obstacle, no_turn_clearance };
struct DriveCommand {
    double left_mps = 0, right_mps = 0;
    NavigationStatus status = NavigationStatus::stopped;
};
struct NavigationConfig {
    double track_m = 0.10; // Provisional: measure wheel contact spacing before hardware use.
    double max_wheel_mps = 0.10;
    double position_tolerance_m = 0.05;
    double heading_tolerance_rad = 0.10;
    double stop_distance_m = 0.20;
    double brake_deceleration_mps2 = 0.20; // Provisional: measure loaded chassis.
    double heading_gain = 1.0;
    std::uint64_t max_age_ms = 250;
};
class Navigator {
public:
    explicit Navigator(NavigationConfig config);
    DriveCommand update(const Waypoint& goal, const NavigationInput& input,
                        std::uint64_t now_ms) const noexcept;
private:
    NavigationConfig config_;
};
// A pure odometry primitive; distances must be signed wheel travel in metres.
Pose integrate_odometry(const Pose& previous, double left_m, double right_m,
                        double track_m, std::uint64_t now_ms);
}
