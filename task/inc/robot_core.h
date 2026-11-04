#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "drv_rc.h"
#include "drv_pid.h"
#include "drv_motor.h"
enum {
    ROBOT_STOP_RC = 1u,
    ROBOT_STOP_FEEDBACK = 2u,
    ROBOT_STOP_TEMPERATURE = 4u,
    ROBOT_STOP_DM_ERROR = 8u,
    ROBOT_STOP_BUS = 16u,
    ROBOT_STOP_ESTOP = 32u,
    ROBOT_STOP_CONFIG = 64u,
    ROBOT_STOP_POSITION_LOST = 128u,
    ROBOT_STOP_LIMIT = 256u,
    ROBOT_STOP_LIFT_SKEW = 512u,
    ROBOT_STOP_NEUTRAL_REQUIRED = 1024u,
    ROBOT_STOP_TIMING = 2048u
};
typedef struct {
    RcState rc;
    DjiFeedback dji[DJI_COUNT];
    DmFeedback dm[DM_COUNT];
    PowerTelemetry power;
    uint8_t profile, bus_fault_mask;
    uint32_t hardware_limit_mask; /* 任一非0位触发全局停机，不自动回零 */
    bool config_confirmed, emergency_stop, deadline_fault;
} RobotInput;
typedef struct {
    bool armed, neutral_tracking, initialized, grip_hold;
    bool dm_origin_valid[DM_COUNT];
    uint8_t profile;
    uint32_t stop_reason, neutral_since_ms, last_ms;
    uint32_t last_bad_frames, step_count, grip_contact_since_ms;
    bool grip_contact_tracking;
    float dm_boot_pos[DM_COUNT], joint_measured[JOINT_COUNT], joint_target[JOINT_COUNT];
    float wheel_reference[4];
    Pid speed_pid[DJI_COUNT];
    PowerLimiter power;
    int16_t dji_command[DJI_COUNT];
    float dm_target[DM_COUNT];
    uint32_t dm_enable_since[DM_COUNT];
    bool dm_enable_waiting[DM_COUNT], dm_active_seen[DM_COUNT], dm_preloaded[DM_COUNT];
} RobotCore;
typedef struct {
    bool abort_pending, abort_wheel_pending, armed;
    uint32_t reason;
    unsigned count;
    MotorFrame frames[6];
} RobotOutput;
void RobotCore_Init(RobotCore *c);
void RobotCore_Step(RobotCore *c, const RobotInput *in, uint32_t now, RobotOutput *out);
uint32_t RobotCore_ExpectedDji(uint8_t profile);
uint32_t RobotCore_ExpectedDm(uint8_t profile);
#ifdef __cplusplus
}
#endif
#endif
