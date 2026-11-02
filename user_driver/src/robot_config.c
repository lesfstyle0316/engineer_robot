#include "robot_config.h"
/* PID 与电流限制是低功率起调值，不是经过负载实测的最终值。
 * direction 表示电机正向对车辆/关节正向的映射。右轮默认反向。 */
#define WHEEL(bus_,id_,dir_) {bus_,id_,dir_,3591.0f/187.0f,{4.0f,0.3f,1200.0f,3000.0f},3000.0f}
const DjiConfig g_dji_config[DJI_COUNT] = {
    WHEEL(1,0x201, 1.0f), WHEEL(1,0x202,-1.0f),
    WHEEL(1,0x203,-1.0f), WHEEL(1,0x204, 1.0f),
    {2,0x205,1.0f,1.0f, {60.0f,5.0f,800.0f,4000.0f},4000.0f}, /* GM6020 ID1: 电压值 */
    {2,0x201,1.0f,36.0f,{3.0f,0.2f,500.0f,1800.0f},1800.0f}, /* M2006/C610 ID1 */
    {2,0x202,1.0f,36.0f,{3.0f,0.2f,500.0f,1800.0f},1800.0f}  /* M2006/C610 ID2 */
};
/* 使用位置速度 PV 模式：命令 ID=0x100+motor_id；使能/失能也用此 ID。
 * 下列映射范围是协议配置示例，不能据电机型号推断实际 EEPROM 值。
 * 必须使用达妙助手读取并核对。代码不自动写电机参数、不自动保存零点。
 * 后腿镜像安装默认右侧反向；需空载确认。 */
const DmConfig g_dm_config[DM_COUNT] = {
    {1,0x301,12.5f,45.0f,18.0f, 1.0f,1.0f,0.35f},
    {2,0x302,12.5f,45.0f,18.0f,-1.0f,1.0f,0.35f},
    {3,0x303,12.5f,45.0f,18.0f, 1.0f,1.0f,0.30f}
};
/* boot_joint_deg 是上电机械姿态对应的关节角，不是自动回零动作。
 * yaw 总是以上电位置为 0。pitch 需手动置于这里声明的姿态再上电。
 * 后腿行程、夹爪行程尚未知；以下 ±30 / 0..60 只是待替换的配置值。
 * 改好机械参数后才能将 ROBOT_CONFIG_CONFIRMED 设为 1。 */
const JointConfig g_joint_config[JOINT_COUNT] = {
    {-180.0f,180.0f,0.0f,30.0f,4.0f,40.0f},
    {-60.0f,45.0f,0.0f,15.0f,4.0f,25.0f},
    {0.0f,60.0f,0.0f,15.0f,4.0f,20.0f},
    {-30.0f,30.0f,0.0f,8.0f,0.0f,15.0f},
    {-30.0f,30.0f,0.0f,8.0f,0.0f,15.0f},
    {0.0f,90.0f,0.0f,12.0f,0.0f,20.0f}
};

/* 四轮总功率 F(k)=R*sum(I^2)*k^2+K*sum(I*n)*k+L。
 * R/K/L沿用所给算法的教学示例；必须用四轮支路实测标定。
 * g_power_config不包含升降电机或机械臂参数。 */
const PowerConfig g_power_config = {
    .resistance_ohm=0.10f,
    .mechanical_w_per_a_rpm=0.0012f,
    .baseline_w=5.0f,
    .rule_limit_w=65.0f,
    .safety_margin_w=5.0f,
    .bench_limit_w=45.0f,
    .buffer_target_j=20.0f,
    .recharge_time_s=1.0f,
    .reserve_up_tau_s=0.05f,
    .reserve_down_tau_s=1.0f,
    .k_recovery_per_s=2.0f,
    .referee_timeout_ms=200u, /* 规则未规定反馈周期，需按实测调整 */
    .cutoff_hold_ms=5000u,
    .require_referee=(ROBOT_POWER_REQUIRE_REFEREE!=0)
};
