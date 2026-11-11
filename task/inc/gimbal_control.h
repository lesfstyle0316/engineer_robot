#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "robot_core.h"

void Gimbal_Step(RobotCore *controller, const RobotInput *input, uint32_t now);
#endif
