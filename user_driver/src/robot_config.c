#include "robot_config.h"
/* PID 与电流限制是低功率起调值，不是经过负载实测的最终值。
 * direction 表示电机正向对车辆/关节正向的映射。四轮方向为起调值，待逐轮实测。 */
#define WHEEL(bus_, id_, dir_) \
    {bus_, id_, dir_, ROBOT_M3508_P19_REDUCTION, {4.0f, 0.3f, 1200.0f, 3000.0f}, 3000.0f}
const DjiConfig g_dji_config[DJI_COUNT] = {
    [DJI_LF] = WHEEL(1, 0x201, 1.0f),
    [DJI_RF] = WHEEL(1, 0x202, -1.0f),
    [DJI_LR] = WHEEL(1, 0x203, 1.0f),
    [DJI_RR] = WHEEL(1, 0x204, -1.0f),
    {2, 0x205, 1.0f, 1.0f, {60.0f, 5.0f, 800.0f, 4000.0f}, 4000.0f}, /* GM6020 ID1: 电压值 */
    {2, 0x201, 1.0f, 36.0f, {3.0f, 0.2f, 500.0f, 1800.0f}, 1800.0f}, /* M2006/C610 ID1 */
    {2, 0x202, 1.0f, 36.0f, {3.0f, 0.2f, 500.0f, 1800.0f}, 1800.0f}  /* M2006/C610 ID2 */
};
/* 后腿使用 MIT 位置保持，大 pitch 使用位置速度 PV 模式。
 * 下列映射范围是协议配置示例，不能据电机型号推断实际 EEPROM 值。
 * 必须使用达妙助手读取并核对。代码不自动写电机参数、不自动保存零点。
 * 后腿镜像安装默认右侧反向；MIT 增益为台架起调值，需空载确认。 */
const DmConfig g_dm_config[DM_COUNT] = {
    /* 左侧按减速前转子接法，机构转 10° 对应协议角度变化 1°；右侧接减速输出法兰。 */
    [DM_LIFT_L] = {0x02, 0x03, 12.5f, 45.0f, 18.0f, 1.0f, 0.10f, 0.90f, DM_MODE_MIT, 10.0f, 0.5f},
    [DM_LIFT_R] = {0x01, 0x11, ROBOT_RIGHT_LIFT_PMAX_RAD, 45.0f, 18.0f, -1.0f, 1.0f,
                   500.0f * ROBOT_DEG2RAD, DM_MODE_MIT, 16.0f, 0.5f},
    [DM_BIG_PITCH] = {0x03, 0x303, 12.5f, 45.0f, 18.0f, 1.0f, 1.0f, 0.30f, DM_MODE_PV, 0.0f, 0.0f},
};
/* boot_joint_deg 是上电机械姿态对应的关节角，不是自动回零动作。
 * yaw 总是以上电位置为 0。pitch 需手动置于这里声明的姿态再上电。
 * 后腿在机械最低位上电，以该位置为 0°。全行程的 CAN 角度增量为左 +360°、右 -3600°。 */
#define LIFT_MAX_TRAVEL_DEG 3600.0f
const JointConfig g_joint_config[JOINT_COUNT] = {
    [JOINT_YAW] = {-180.0f, 180.0f, 0.0f, 30.0f, 4.0f, 40.0f},
    [JOINT_SMALL_PITCH] = {-60.0f, 45.0f, 0.0f, 15.0f, 4.0f, 25.0f},
    [JOINT_GRIP] = {0.0f, 60.0f, 0.0f, 15.0f, 4.0f, 20.0f},
    [JOINT_LIFT_L] = {0.0f, LIFT_MAX_TRAVEL_DEG, 0.0f, ROBOT_LIFT_SYNC_SPEED_DEG_S, 0.0f, 0.0f},
    [JOINT_LIFT_R] = {0.0f, LIFT_MAX_TRAVEL_DEG, 0.0f, ROBOT_LIFT_SYNC_SPEED_DEG_S, 0.0f, 0.0f},
    [JOINT_BIG_PITCH] = {0.0f, 90.0f, 0.0f, 12.0f, 0.0f, 20.0f},
};

/* 四轮总功率 F(k)=R*sum(I^2)*k^2+K*sum(I*n)*k+L。
 * R/K/L沿用所给算法的教学示例；必须用四轮支路实测标定。
 * g_power_config不包含升降电机或机械臂参数。 */
const PowerConfig g_power_config = {.resistance_ohm = 0.10f,
                                    .mechanical_w_per_a_rpm = 0.0012f,
                                    .baseline_w = 5.0f,
                                    .rule_limit_w = 65.0f,
                                    .safety_margin_w = 5.0f,
                                    .bench_limit_w = 45.0f,
                                    .buffer_target_j = 20.0f,
                                    .recharge_time_s = 1.0f,
                                    .reserve_up_tau_s = 0.05f,
                                    .reserve_down_tau_s = 1.0f,
                                    .k_recovery_per_s = 2.0f,
                                    .referee_timeout_ms = 200u, /* 规则未规定反馈周期，需按实测调整 */
                                    .cutoff_hold_ms = 5000u,
                                    .require_referee = (ROBOT_POWER_REQUIRE_REFEREE != 0)};
