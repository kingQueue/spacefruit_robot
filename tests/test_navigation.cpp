#include "navigation.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
bool near(double a, double b) { return std::abs(a - b) < 1e-8; }
int main() {
    using namespace robot;
    Navigator nav{NavigationConfig{}};
    NavigationInput in{{0, 0, 0, 1000, true}, {1, 1000, true}, {1, 1000, true}};
    Waypoint goal{1, 0};
    auto out = nav.update(goal, in, 1000);
    CHECK(out.status == NavigationStatus::moving && out.left_mps > 0);
    CHECK(near(out.left_mps, out.right_mps));
    CHECK(nav.update({0, 0}, in, 1000).status == NavigationStatus::arrived);
    in.emergency_stop = true;
    CHECK(nav.update(goal, in, 1000).left_mps == 0);
    in.emergency_stop = false; in.bumper_pressed = true;
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::stopped);
    in.bumper_pressed = false;
    CHECK(nav.update(goal, in, 1251).status == NavigationStatus::invalid_data);
    CHECK(nav.update(goal, in, 999).status == NavigationStatus::invalid_data);
    in.front.valid = false;
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::invalid_data);
    in.front.valid = true; in.rear.valid = false;
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::invalid_data);
    in.rear.valid = true; in.front.distance_m = 0.20;
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::obstacle);
    in.front.distance_m = 0.201;
    out = nav.update(goal, in, 1000);
    CHECK(out.left_mps > 0 && out.left_mps < 0.01);
    in.front.distance_m = 1;
    CHECK(nav.update({0, 1}, in, 1000).status == NavigationStatus::no_turn_clearance);
    in.turning_clearance = true;
    out = nav.update({0, 1}, in, 1000);
    CHECK(out.left_mps < 0 && out.right_mps > 0);
    CHECK(std::abs(out.left_mps) <= 0.1 && std::abs(out.right_mps) <= 0.1);
    in.rear.distance_m = 0.1;
    CHECK(nav.update({0, 1}, in, 1000).status == NavigationStatus::obstacle);
    in.rear.distance_m = 1;
    in.pose.x_m = std::numeric_limits<double>::quiet_NaN();
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::invalid_data);
    in.pose.x_m = 0;
    CHECK(nav.update({std::numeric_limits<double>::infinity(), 0}, in, 1000).status == NavigationStatus::invalid_data);
    in.front.distance_m = -1;
    CHECK(nav.update(goal, in, 1000).status == NavigationStatus::invalid_data);
    Pose origin{0, 0, 0, 1000, true};
    auto pose = integrate_odometry(origin, 0.1, 0.1, 0.1, 1100);
    CHECK(near(pose.x_m, 0.1) && near(pose.y_m, 0) && near(pose.yaw_rad, 0));
    pose = integrate_odometry(origin, -0.05, 0.05, 0.1, 1100);
    CHECK(near(pose.x_m, 0) && near(pose.y_m, 0) && near(pose.yaw_rad, 1));
    pose = integrate_odometry(origin, 0, 0.1, 0.1, 1100);
    CHECK(near(pose.x_m, 0.05 * std::sin(1.0)));
    CHECK(near(pose.y_m, 0.05 * (1 - std::cos(1.0))));
    bool threw = false;
    try { (void)integrate_odometry(origin, 0, 0, 0, 1000); } catch(const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { NavigationConfig c; c.max_age_ms = 0; Navigator bad(c); } catch(const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    puts("PASS: navigation validation, stops, waypoint tracking, braking, turns, odometry");
}
