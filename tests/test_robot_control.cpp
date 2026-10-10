#include "robot_control.hpp"
#include <cstdio>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)
int main() {
    robot::Controller c;
    CHECK(c.samples() == 0 && c.invalid_samples() == 0);
    CHECK(!c.update(robot::dry_percent).pump_requested);
    CHECK(c.update(robot::dry_percent - 1).pump_requested);
    CHECK(c.update(robot::wet_percent - 1).pump_requested);
    CHECK(!c.update(robot::wet_percent).pump_requested);
    CHECK(c.update(0).pump_requested);
    const auto invalid = c.update(101);
    CHECK(!invalid.valid && !invalid.pump_requested);
    CHECK(c.invalid_samples() == 1);
    CHECK(c.update(100).valid);
    CHECK(!c.update(35).pump_requested);
    CHECK(c.samples() == 8);
    c.reset();
    CHECK(c.samples() == 0 && c.invalid_samples() == 0);
    CHECK(!c.update(35).pump_requested);
    std::puts("PASS: C++ thresholds, hysteresis, invalid input, counters, reset");
}
