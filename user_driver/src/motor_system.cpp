#include "DJI_motor_driver.h"
#include "DM_motor_driver.h"
#include "motor_system.h"
#include <new>
namespace {
class robot_motor_devices {
  public:
    can_network &net;
    DJI_motor_group chassis_group, arm_current_group, yaw_voltage_group;
    DJI_motor wheel_lf, wheel_rf, wheel_lr, wheel_rr, yaw, small_pitch, gripper;
    DM_motor lift_left, lift_right, big_pitch;
    PowerTelemetry power{};
    can_device_receive power_rx;
    robot_motor_devices()
        : net(robot_can_network()), chassis_group(net, 1, 0x200), arm_current_group(net, 2, 0x200),
          yaw_voltage_group(net, 2, 0x1ff), wheel_lf(net, chassis_group, g_dji_config[DJI_LF], 1, DJI_M3508),
          wheel_rf(net, chassis_group, g_dji_config[DJI_RF], 2, DJI_M3508),
          wheel_lr(net, chassis_group, g_dji_config[DJI_LR], 3, DJI_M3508),
          wheel_rr(net, chassis_group, g_dji_config[DJI_RR], 4, DJI_M3508),
          yaw(net, yaw_voltage_group, g_dji_config[DJI_YAW], ROBOT_YAW_MOTOR_ID, DJI_GM6020),
          small_pitch(net, arm_current_group, g_dji_config[DJI_SMALL_PITCH], ROBOT_SMALL_PITCH_MOTOR_ID, DJI_M2006),
          gripper(net, arm_current_group, g_dji_config[DJI_GRIP], ROBOT_GRIP_MOTOR_ID, DJI_M2006),
          lift_left(net, g_dm_config[DM_LIFT_L]), lift_right(net, g_dm_config[DM_LIFT_R]),
          big_pitch(net, g_dm_config[DM_BIG_PITCH]),
          power_rx(net, ROBOT_POWER_CAN_BUS, ROBOT_POWER_CAN_ID, power_update, this, 4, power_update) {}
    static void power_update(can_device_receive *rx, const MotorFrame &frame, uint32_t now) {
        auto *self = static_cast<robot_motor_devices *>(rx->context());
        (void)PowerTelemetry_Decode(&self->power, frame.data, frame.dlc, ROBOT_POWER_BYTE_ORDER, now);
    }
    bool valid() const {
        return wheel_lf.valid() && wheel_rf.valid() && wheel_rr.valid() && wheel_lr.valid() && yaw.valid() &&
               small_pitch.valid() && gripper.valid() && lift_left.valid() && lift_right.valid() &&
               big_pitch.valid() && power_rx.valid();
    }
};
robot_motor_devices &devices() {
    /* 固定存储 + 原位构造，不申请堆，也不注册退出析构；只在HAL启动CAN前调用首次初始化。 */
    alignas(robot_motor_devices) static unsigned char storage[sizeof(robot_motor_devices)];
    static robot_motor_devices *instance = nullptr;
    if (!instance)
        instance = new (storage) robot_motor_devices;
    return *instance;
}
} // namespace
extern "C" bool MotorSystem_Init(void) {
    return devices().valid();
}
extern "C" void MotorSystem_CopyFeedback(DjiFeedback d[DJI_COUNT], DmFeedback m[DM_COUNT],
                                         PowerTelemetry *p) {
    auto &s = devices();
    d[DJI_LF] = s.wheel_lf.feedback();
    d[DJI_RF] = s.wheel_rf.feedback();
    d[DJI_LR] = s.wheel_lr.feedback();
    d[DJI_RR] = s.wheel_rr.feedback();
    d[DJI_YAW] = s.yaw.feedback();
    d[DJI_SMALL_PITCH] = s.small_pitch.feedback();
    d[DJI_GRIP] = s.gripper.feedback();
    m[DM_LIFT_L] = s.lift_left.feedback();
    m[DM_LIFT_R] = s.lift_right.feedback();
    m[DM_BIG_PITCH] = s.big_pitch.feedback();
    *p = s.power;
}
extern "C" bool MotorSystem_SendFrame(const MotorFrame *f, uint32_t now) {
    if (!f)
        return false;
    auto &s = devices();
    if (f->bus == 1 && f->id == 0x200)
        return s.chassis_group.apply_frame(*f, now);
    if (f->bus == 2 && f->id == 0x200)
        return s.arm_current_group.apply_frame(*f, now);
    if (f->bus == 2 && f->id == 0x1ff)
        return s.yaw_voltage_group.apply_frame(*f, now);
    if (f->bus == 2) {
        if (f->id == s.lift_left.command_id())
            return s.lift_left.apply_frame(*f, now);
        if (f->id == s.lift_right.command_id())
            return s.lift_right.apply_frame(*f, now);
        if (f->id == s.big_pitch.command_id())
            return s.big_pitch.apply_frame(*f, now);
    }
    return false;
}
