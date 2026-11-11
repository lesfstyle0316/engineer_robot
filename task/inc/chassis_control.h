#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "robot_core.h"

void Chassis_WheelStep(const RcState *rc, const DjiFeedback dji[4], Pid speed_pid[4], int16_t command[4]);
void Chassis_LiftStep(RobotCore *controller, const RobotInput *input);
#endif
