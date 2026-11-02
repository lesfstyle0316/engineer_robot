#ifndef BSP_IO_H
#define BSP_IO_H
#include "drv_motor.h"
#include "drv_rc.h"
typedef struct {RcState rc;DjiFeedback dji[DJI_COUNT];DmFeedback dm[DM_COUNT];PowerTelemetry power;uint8_t bus_fault_mask;} BspSnapshot;
bool BSP_IO_Init(void);
void BSP_IO_Snapshot(BspSnapshot *out);
void BSP_Dbus_Service(void);
bool BSP_CAN_Send(const MotorFrame *frame);
void BSP_CAN_AbortAll(void);
void BSP_CAN_AbortBus(uint8_t bus);
/* 预留硬件接口：在 bsp_board.c 中实现接线后返回实际状态。 */
bool BSP_UserEmergencyStop(void);
uint32_t BSP_UserLimitMask(void);
void BSP_StatusLed(bool armed,bool rc_ok,uint8_t profile,uint32_t now);
void BSP_FatalStop(void);
#endif
