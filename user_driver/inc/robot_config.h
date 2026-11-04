#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "drv_power.h"
/* 唯一的集中配置入口。先看 README.md。单位写在字段名或注释中。 */
#define ROBOT_PROFILE_RC_ONLY 0u
#define ROBOT_PROFILE_CHASSIS 1u /* 四轮 + 同步后腿 */
#define ROBOT_PROFILE_ARM 2u
#define ROBOT_PROFILE_FULL 3u
#ifndef ROBOT_DEFAULT_PROFILE
#define ROBOT_DEFAULT_PROFILE ROBOT_PROFILE_RC_ONLY
#endif
/* 机械尚未完成：核对方向、传动比、零位、限位及驱动参数后改为 1。
 * RC_ONLY 不受此开关影响。此开关不是急停开关。 */
#ifndef ROBOT_CONFIG_CONFIRMED
#define ROBOT_CONFIG_CONFIRMED 0
#endif
/* CAN对象层容量与最长待发/邮箱驻留时间；两路分别分配固定存储。 */
#define ROBOT_CAN_MAX_RX_PER_BUS 16u
#define ROBOT_CAN_TX_QUEUE_CAPACITY 16u
#define ROBOT_CAN_MAX_TX_AGE_MS 5u
#define ROBOT_RC_TIMEOUT_MS 60u
#define ROBOT_MOTOR_TIMEOUT_MS 50u
#define ROBOT_NEUTRAL_DWELL_MS 500u
#define ROBOT_DJI_PERIOD_MS 2u /* 500Hz 控制，反馈 1000Hz */
#define ROBOT_DM_PERIOD_MS 5u  /* 200Hz 控制 + 200Hz 回复 */
#define ROBOT_CONTROL_DT 0.001f
#define ROBOT_MAX_TEMP_C 80u
#define ROBOT_RC_DEADBAND 20
#define ROBOT_RC_LEFT_SWITCH 0u  /* 协议 S1 */
#define ROBOT_RC_RIGHT_SWITCH 1u /* 协议 S2 */
#define ROBOT_RC_FORWARD_SIGN 1.0f
#define ROBOT_RC_TURN_SIGN 1.0f
#define ROBOT_RC_YAW_SIGN 1.0f /* 推右 = 正关节方向，可在此取反 */
#define ROBOT_RC_PITCH_SIGN 1.0f
#define ROBOT_RC_GRIP_SIGN 1.0f     /* 推右 = 夹紧 */
#define ROBOT_WHEEL_MAX_RPM 1500.0f /* 转子 rpm；无轮径假设 */
#define ROBOT_WHEEL_ACCEL_RPM_S 2500.0f
/* RoboSister电源管理模块：CAN2 / 标准帧0x31 / DLC4 / 大端int16。
 * 只从CAN接收功率与缓冲，不向裁判模块发送查询或配置命令。
 * 比赛必须保持REQUIRE_REFEREE=1；无裁判台架可设0，仍按45W模型限功率。 */
#define ROBOT_POWER_CAN_BUS 2u
#define ROBOT_POWER_CAN_ID 0x31u
#define ROBOT_POWER_BYTE_ORDER POWER_ORDER_BIG_ENDIAN
#define ROBOT_POWER_REQUIRE_REFEREE 1
#define ROBOT_LIFT_MAX_SKEW_DEG 5.0f
#define ROBOT_GRIP_CONTACT_CURRENT 1200.0f /* C610 反馈原始值，待实测 */
#define ROBOT_GRIP_HOLD_CURRENT 700.0f
#define ROBOT_GRIP_STALL_SPEED_DEG_S 3.0f
#define ROBOT_GRIP_CONTACT_MS 150u
#define ROBOT_DM_ENABLE_TIMEOUT_MS 200u
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_DEG2RAD (ROBOT_PI / 180.0f)
#define ROBOT_RAD2DEG (180.0f / ROBOT_PI)
enum { DJI_LF, DJI_RF, DJI_RR, DJI_LR, DJI_YAW, DJI_SMALL_PITCH, DJI_GRIP, DJI_COUNT };
enum { DM_LIFT_L, DM_LIFT_R, DM_BIG_PITCH, DM_COUNT };
enum { JOINT_YAW, JOINT_SMALL_PITCH, JOINT_GRIP, JOINT_LIFT_L, JOINT_LIFT_R, JOINT_BIG_PITCH, JOINT_COUNT };
typedef struct {
    float kp, ki, integral_limit, output_limit;
} PidConfig;
typedef struct {
    uint8_t bus;
    uint16_t feedback_id;
    float direction, reduction;
    PidConfig speed_pid;
    float max_command; /* C620 ±16384 / C610 ±10000 / GM6020 ±30000 */
} DjiConfig;
typedef struct {
    uint16_t motor_id, master_id;
    float p_max, v_max, t_max; /* 必须与达妙助手中实际值完全相同 */
    float direction, external_reduction;
    float velocity_limit_rad_s; /* 电机输出轴；位置速度模式使用 */
} DmConfig;
typedef struct {
    float min_deg, max_deg, boot_joint_deg, slew_deg_s;
    float position_kp, max_speed_deg_s;
} JointConfig;
extern const DjiConfig g_dji_config[DJI_COUNT];
extern const DmConfig g_dm_config[DM_COUNT];
extern const JointConfig g_joint_config[JOINT_COUNT];
#ifdef __cplusplus
}
#endif
#endif
