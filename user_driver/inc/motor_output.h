#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H
#ifdef __cplusplus
extern "C" {
#endif
#include "drv_motor.h"

enum { MOTOR_OUTPUT_MAX_FRAMES = 3 };
typedef struct {
    bool preloaded[DM_COUNT];
} MotorOutput;

unsigned MotorOutput_BuildFrames(MotorOutput *output, const DmFeedback dm[DM_COUNT],
                                 const int16_t dji_command[DJI_COUNT], const float dm_target[DM_COUNT],
                                 bool enabled, bool gimbal_enabled, uint32_t tick,
                                 MotorFrame frames[MOTOR_OUTPUT_MAX_FRAMES]);
#ifdef __cplusplus
}
#endif
#endif
