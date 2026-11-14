#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "drv_rc.h"
#include "drv_pid.h"
#include "motor_output.h"
enum {
    ROBOT_STOP_RC = 1u,
    ROBOT_STOP_FEEDBACK = 2u,
    ROBOT_STOP_DM_ERROR = 8u,
    ROBOT_STOP_BUS = 16u,
    ROBOT_STOP_ESTOP = 32u,
    ROBOT_STOP_POSITION_LOST = 128u,
    ROBOT_STOP_LIMIT = 256u,
    ROBOT_STOP_NEUTRAL_REQUIRED = 1024u,
};
enum {
    MOTOR_FEEDBACK_NOT_SEEN = 0u,
    MOTOR_FEEDBACK_OK = 1u,
    MOTOR_FEEDBACK_TIMEOUT = 2u,
    MOTOR_FEEDBACK_WAIT_ZERO = 3u,
    MOTOR_FEEDBACK_INVALID_POSITION = 4u,
    MOTOR_FEEDBACK_POSITION_LOST = 5u,
    MOTOR_FEEDBACK_NOT_INSTALLED = 6u,
};
typedef struct {
    RcState rc;
    DjiFeedback dji[DJI_COUNT];
    DmFeedback dm[DM_COUNT];
    PowerTelemetry power;
    uint8_t bus_fault_mask;
    uint32_t hardware_limit_mask; /* 任一非0位触发全局停机，不自动回零 */
    bool emergency_stop;
} RobotInput;
typedef struct {
    float previous_rpm, acceleration;
    bool sampled;
} JointSpeedPd;
typedef struct {
    bool armed, neutral_tracking, grip_hold;
    bool dm_origin_valid[DM_COUNT];
    uint8_t lift_wait_mask;
    uint32_t stop_reason, neutral_since_ms;
    uint32_t last_bad_frames, step_count, grip_contact_since_ms;
    bool grip_contact_tracking;
    float dm_boot_pos[DM_COUNT], joint_measured[JOINT_COUNT], joint_target[JOINT_COUNT];
    float lift_skew_deg; /* 机构角度，左减右 */
    float lift_direction;
    Pid speed_pid[DJI_COUNT];
    PowerLimiter power;
    int16_t dji_command[DJI_COUNT];
    float dm_target[DM_COUNT];
    bool dm_active_seen[DM_COUNT];
    MotorOutput motor_output;
    uint8_t dji_feedback_status[DJI_COUNT];
    uint8_t dm_feedback_status[DM_COUNT];
    float big_pitch_gravity_ff_nm;
    JointSpeedPd arm_m2006_speed_pd[JOINT_GRIP - JOINT_SMALL_PITCH + 1];
    float small_pitch_gravity_ff_command; /* C610 原生方向，电流指令原始单位 */
} RobotCore;
typedef struct {
    bool abort_pending, abort_wheel_pending, armed;
    uint32_t reason;
    unsigned count;
    MotorFrame frames[MOTOR_OUTPUT_MAX_FRAMES];
} RobotOutput;
void RobotCore_Init(RobotCore *c);
void RobotCore_Step(RobotCore *c, const RobotInput *in, uint32_t now, RobotOutput *out);
#ifdef __cplusplus
}
#endif
#endif
