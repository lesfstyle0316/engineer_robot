// 电机公共接口模板作者 Tessia。
#include "motor_driver.h"

float motor_dev::motor_get_speed() const {
    if (direction_ == -1.0f)
        return -motor_get_speed_forward();
    return motor_get_speed_forward();
}

float motor_dev::motor_get_rounds() const {
    if (direction_ == -1.0f)
        return -motor_get_rounds_forward();
    return motor_get_rounds_forward();
}
