#ifndef DRV_PID_H
#define DRV_PID_H
#ifdef __cplusplus
extern "C" {
#endif
#include "robot_config.h"
typedef struct {
    float integral;
} Pid;
float Control_Clamp(float x, float low, float high);
float Control_Slew(float current, float target, float maximum_step);
float Pid_Update(Pid *p, const PidConfig *c, float error, float dt);
void Pid_Reset(Pid *p);
#ifdef __cplusplus
}
#endif
#endif
