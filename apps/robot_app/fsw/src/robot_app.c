#include "cfe.h"
#include "robot_control.h"

void ROBOT_APP_Main(void);

/* A cFS task using simulated sensor input. No physical output is driven. */
void ROBOT_APP_Main(void)
{
    uint32 run_status = CFE_ES_RunStatus_APP_RUN;
    RobotControl state;
    const unsigned int moisture[] = {20u, 35u, 50u};
    unsigned int index = 0;
    RobotControl_Init(&state);
    if (CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY) != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("ROBOT_APP event registration failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }
    CFE_ES_WriteToSysLog("ROBOT_APP ready: simulated moisture control\n");
    while (CFE_ES_RunLoop(&run_status))
    {
        unsigned int value = moisture[index];
        (void)RobotControl_Update(&state, value);
        CFE_ES_WriteToSysLog("ROBOT_APP sample=%lu moisture=%u pump=%u\n",
                            (unsigned long)state.samples, value,
                            state.pump_requested ? 1u : 0u);
        index = (index + 1u) % 3u;
        OS_TaskDelay(1000);
    }
    CFE_ES_ExitApp(run_status);
}
