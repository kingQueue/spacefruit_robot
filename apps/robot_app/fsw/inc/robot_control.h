#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

/* Percentage thresholds: edit these to change the sample's behavior. */
#define ROBOT_DRY_PERCENT 30u
#define ROBOT_WET_PERCENT 45u

typedef struct
{
    bool pump_requested;
    uint32_t samples;
    uint32_t invalid_samples;
} RobotControl;

void RobotControl_Init(RobotControl *state);
bool RobotControl_Update(RobotControl *state, unsigned int moisture_percent);
#endif
