#include "robot_control.h"
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); return 1; \
} } while (0)

int main(void)
{
    RobotControl state;
    RobotControl_Init(&state);
    CHECK(!state.pump_requested && state.samples == 0u);
    CHECK(RobotControl_Update(&state, ROBOT_DRY_PERCENT));
    CHECK(!state.pump_requested);
    CHECK(RobotControl_Update(&state, ROBOT_DRY_PERCENT - 1u));
    CHECK(state.pump_requested);
    CHECK(RobotControl_Update(&state, ROBOT_WET_PERCENT - 1u));
    CHECK(state.pump_requested); /* Hysteresis retains state in the middle. */
    CHECK(RobotControl_Update(&state, ROBOT_WET_PERCENT));
    CHECK(!state.pump_requested);
    CHECK(RobotControl_Update(&state, 0u));
    CHECK(state.pump_requested);
    CHECK(!RobotControl_Update(&state, 101u));
    CHECK(!state.pump_requested && state.invalid_samples == 1u);
    CHECK(RobotControl_Update(&state, 100u));
    CHECK(!state.pump_requested && state.samples == 7u);
    RobotControl_Init(&state);
    CHECK(state.samples == 0u && state.invalid_samples == 0u);
    puts("PASS: thresholds, hysteresis, invalid sensor fail-off, counters, reset");
    return 0;
}
