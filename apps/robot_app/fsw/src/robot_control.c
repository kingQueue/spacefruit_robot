#include "robot_control.h"

void RobotControl_Init(RobotControl *state)
{
    state->pump_requested = false;
    state->samples = 0;
    state->invalid_samples = 0;
}

bool RobotControl_Update(RobotControl *state, unsigned int moisture_percent)
{
    ++state->samples;
    if (moisture_percent > 100u)
    {
        ++state->invalid_samples;
        state->pump_requested = false;
        return false;
    }
    if (moisture_percent < ROBOT_DRY_PERCENT)
        state->pump_requested = true;
    else if (moisture_percent >= ROBOT_WET_PERCENT)
        state->pump_requested = false;
    return true;
}
