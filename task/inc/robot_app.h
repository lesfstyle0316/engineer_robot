#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "robot_core.h"
#include "bsp_io.h"
/* 可在调试器中实时更改模式；变化会停机并要求回中500ms。 */
extern volatile uint32_t g_robot_requested_profile;
extern volatile uint32_t g_robot_emergency_stop;
typedef struct {
    uint32_t heartbeat_ms, iterations, can_send_failures, max_cycle_us;
    RobotInput input;
    RobotCore controller;
    BspCanStatistics can[2];
} RobotDebug;
extern volatile RobotDebug g_robot_debug;
void RobotApp_CreateTasks(void);
#ifdef __cplusplus
}
#endif
#endif
