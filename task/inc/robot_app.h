#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "robot_core.h"
#include "bsp_io.h"
/* 可在调试器中实时更改模式；变化会停机并要求回中500ms。 */
extern volatile uint32_t g_robot_emergency_stop;
typedef struct {
    uint32_t heartbeat_ms, iterations, can_send_failures, max_cycle_us;
    uint32_t rc_age_ms; /* 距最新有效遥控帧的时间，ms */
    bool rc_online;     /* 有效且未达到60ms超时；不能只看rc.valid */
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
