#ifndef MOTOR_SYSTEM_H
#define MOTOR_SYSTEM_H
#include "drv_motor.h"
#ifdef __cplusplus
extern "C" {
#endif
bool MotorSystem_Init(void);
void MotorSystem_CopyFeedback(DjiFeedback dji[DJI_COUNT], DmFeedback dm[DM_COUNT], PowerTelemetry *power);
bool MotorSystem_SendFrame(const MotorFrame *frame, uint32_t now);
#ifdef __cplusplus
}
#endif
#endif
