#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "drv_power.h"
/* 唯一的集中配置入口。先看 README.md。单位写在字段名或注释中。 */
#define ROBOT_PROFILE_CHASSIS 1u /* 四轮 + 同步后腿 */
#define ROBOT_PROFILE_FULL 3u
/* 当前上电默认：底盘调试（四轮 + 双后腿）。 */
#ifndef ROBOT_DEFAULT_PROFILE
#define ROBOT_DEFAULT_PROFILE ROBOT_PROFILE_CHASSIS
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
#define ROBOT_RC_DEADBAND 20
#define ROBOT_RC_LEFT_SWITCH 0u  /* 协议 S1 */
#define ROBOT_RC_RIGHT_SWITCH 1u /* 协议 S2 */
#define ROBOT_RC_FORWARD_SIGN 1.0f
#define ROBOT_RC_TURN_SIGN 1.0f
#define ROBOT_RC_YAW_SIGN 1.0f /* 推右 = 正关节方向，可在此取反 */
#define ROBOT_RC_PITCH_SIGN 1.0f
#define ROBOT_RC_GRIP_SIGN 1.0f     /* 推右 = 夹紧 */
#define ROBOT_M3508_P19_REDUCTION (3591.0f / 187.0f)
#define ROBOT_WHEEL_MAX_OUTPUT_RPM (1500.0f / ROBOT_M3508_P19_REDUCTION) /* 轮端约 78.1 rpm */
/* RoboSister电源管理模块：CAN2 / 标准帧0x31 / DLC4 / 大端int16。
 * 只从CAN接收功率与缓冲，不向裁判模块发送查询或配置命令。
 * 当前为无裁判台架调试，按45W模型限功率；比赛前改回1。 */
#define ROBOT_POWER_CAN_BUS 2u
#define ROBOT_POWER_CAN_ID 0x31u
#define ROBOT_POWER_BYTE_ORDER POWER_ORDER_BIG_ENDIAN
#define ROBOT_POWER_REQUIRE_REFEREE 0
/* 同步阈值使用换算后的机构角度。 */
#define ROBOT_LIFT_MAX_SKEW_DEG 5.0f
#define ROBOT_LIFT_RESUME_SKEW_DEG 3.0f
#define ROBOT_LIFT_MAX_LEAD_DEG 18.0f
/* 联动使用同一个机构角速度，当前对应左 CAN 50°/s、右 CAN 500°/s。 */
#define ROBOT_LIFT_SYNC_SPEED_DEG_S 500.0f
/* 必须与右侧达妙驱动器读回的 PMAX 一致，单位 rad。 */
#ifndef ROBOT_RIGHT_LIFT_PMAX_RAD
#define ROBOT_RIGHT_LIFT_PMAX_RAD 12.5f
#endif
#define ROBOT_GRIP_CONTACT_CURRENT 1200.0f /* C610 反馈原始值，待实测 */
#define ROBOT_GRIP_HOLD_CURRENT 700.0f
#define ROBOT_GRIP_STALL_SPEED_DEG_S 3.0f
#define ROBOT_GRIP_CONTACT_MS 150u
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_DEG2RAD (ROBOT_PI / 180.0f)
#define ROBOT_RAD2DEG (180.0f / ROBOT_PI)
enum { DJI_LF, DJI_RF, DJI_LR, DJI_RR, DJI_YAW, DJI_SMALL_PITCH, DJI_GRIP, DJI_COUNT };
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
typedef enum { DM_MODE_PV, DM_MODE_MIT } DmMode;
typedef struct {
    uint16_t motor_id, master_id;
    float p_max, v_max, t_max; /* 必须与达妙助手中实际值完全相同 */
    float direction;
    float external_reduction; /* 达妙协议角度增量 / 机构角度增量的绝对值 */
    float velocity_limit_rad_s; /* 输出轴，PV 命令限速或 MIT 目标速度上限 */
    DmMode mode;
    float mit_kp, mit_kd;
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
