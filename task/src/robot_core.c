#include "robot_core.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include <math.h>
#include <string.h>

void RobotCore_Init(RobotCore *c) {
    memset(c, 0, sizeof(*c));
    PowerLimiter_Init(&c->power);
}

static void measurements(RobotCore *c, const RobotInput *in) {
    for (unsigned i = JOINT_YAW; i <= JOINT_GRIP; i++) {
        unsigned mi = DJI_YAW + i;
        c->joint_measured[i] = Dji_JointDegrees(&in->dji[mi], &g_dji_config[mi]);
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        if (in->dm[i].seen && !c->dm_origin_valid[i] && isfinite(in->dm[i].position)) {
            c->dm_boot_pos[i] = in->dm[i].position;
            c->dm_origin_valid[i] = true;
        }
        const DmConfig *d = &g_dm_config[i];
        c->joint_measured[JOINT_LIFT_L + i] =
            (in->dm[i].position - c->dm_boot_pos[i]) * d->direction * ROBOT_RAD2DEG / d->external_reduction +
            g_joint_config[JOINT_LIFT_L + i].boot_joint_deg;
    }
    c->lift_skew_deg = c->joint_measured[JOINT_LIFT_L] - c->joint_measured[JOINT_LIFT_R];
}

static uint32_t stop_reason(const RobotCore *c, const RobotInput *in, uint32_t now) {
    if (!in->rc.valid || (uint32_t)(now - in->rc.stamp_ms) >= ROBOT_RC_TIMEOUT_MS ||
        in->rc.bad_frames != c->last_bad_frames)
        return ROBOT_STOP_RC;
    if (in->emergency_stop)
        return ROBOT_STOP_ESTOP;
    if (in->hardware_limit_mask)
        return ROBOT_STOP_LIMIT;
    if (in->bus_fault_mask)
        return ROBOT_STOP_BUS;
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        if (i == DJI_GRIP && !ROBOT_GRIP_INSTALLED)
            continue;
        const DjiFeedback *d = &in->dji[i];
        if (!d->seen || !d->zero_ready || (uint32_t)(now - d->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS)
            return ROBOT_STOP_FEEDBACK;
        if (i >= DJI_YAW && d->position_lost)
            return ROBOT_STOP_POSITION_LOST;
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        const DmFeedback *d = &in->dm[i];
        if (!d->seen || (uint32_t)(now - d->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS || !isfinite(d->position))
            return ROBOT_STOP_FEEDBACK;
        if (d->state > 1u || (d->state == 0u && c->dm_active_seen[i]))
            return ROBOT_STOP_DM_ERROR;
    }
    return 0u;
}

static void reset_control(RobotCore *c) {
    memset(c->speed_pid, 0, sizeof(c->speed_pid));
    memset(c->dji_command, 0, sizeof(c->dji_command));
    memset(c->dm_active_seen, 0, sizeof(c->dm_active_seen));
    memset(&c->motor_output, 0, sizeof(c->motor_output));
    PowerLimiter_ResetOutput(&c->power);
    c->grip_hold = false;
    c->grip_contact_tracking = false;
    c->lift_wait_mask = 0u;
    c->lift_direction = 0.0f;
    for (unsigned i = 0; i < JOINT_COUNT; i++) {
        const JointConfig *cfg = &g_joint_config[i];
        c->joint_target[i] = Control_Clamp(c->joint_measured[i], cfg->min_deg, cfg->max_deg);
    }
}

static void update_wheels(RobotCore *c, const RobotInput *in, uint32_t now) {
    if (!ROBOT_WHEEL_POWER_LIMIT_ENABLED) {
        Chassis_WheelStep(&in->rc, in->dji, c->speed_pid, c->dji_command);
        c->power.k_applied = 1.0f;
        return;
    }

    int16_t request[4], measured[4], rpm[4];
    float integral[4];
    for (unsigned i = 0; i < 4; i++) {
        integral[i] = c->speed_pid[i].integral;
        measured[i] = in->dji[i].current;
        rpm[i] = in->dji[i].rpm;
    }
    Chassis_WheelStep(&in->rc, in->dji, c->speed_pid, request);
    PowerLimiter_Apply(&c->power, &g_power_config, &in->power, request, measured, rpm, now,
                       ROBOT_CONTROL_DT, c->dji_command);
    /* 只撤销会继续推高受限电流的积分。功率模块不参与速度解算。 */
    for (unsigned i = 0; i < 4; i++)
        if ((request[i] - c->dji_command[i]) * g_dji_config[i].direction *
                (c->speed_pid[i].integral - integral[i]) > 0.0f)
            c->speed_pid[i].integral = integral[i];
    Chassis_ApplyRearBufferAssist(in, &c->power, request, c->dji_command);
}

static void update_dm_targets(RobotCore *c, const RobotInput *in) {
    for (unsigned i = 0; i < DM_COUNT; i++) {
        const DmConfig *motor = &g_dm_config[i];
        unsigned joint = JOINT_LIFT_L + i;
        float delta = c->joint_target[joint] - g_joint_config[joint].boot_joint_deg;
        float position = c->dm_boot_pos[i] + delta * ROBOT_DEG2RAD * motor->direction * motor->external_reduction;
        c->dm_target[i] = Control_Clamp(position, -motor->p_max, motor->p_max);
        if (in->dm[i].state == 1u)
            c->dm_active_seen[i] = true;
    }
}

void RobotCore_Step(RobotCore *c, const RobotInput *in, uint32_t now, RobotOutput *out) {
    memset(out, 0, sizeof(*out));
    bool was_armed = c->armed;
    float previous_power = c->power.k_applied;
    measurements(c, in);

    uint32_t reason = stop_reason(c, in, now);
    c->last_bad_frames = in->rc.bad_frames;
    if (reason) {
        c->armed = false;
        c->neutral_tracking = false;
        reset_control(c);
    } else if (!c->armed && Rc_IsNeutral(&in->rc)) {
        if (!c->neutral_tracking) {
            c->neutral_since_ms = now;
            c->neutral_tracking = true;
        }
        if ((uint32_t)(now - c->neutral_since_ms) >= ROBOT_NEUTRAL_DWELL_MS) {
            reset_control(c);
            c->armed = true;
        }
    } else if (!c->armed) {
        c->neutral_tracking = false;
    }

    if (c->armed) {
        update_wheels(c, in, now);
        Chassis_LiftStep(c, in);
        Gimbal_Step(c, in, now);
        update_dm_targets(c, in);
    }
    c->stop_reason = reason ? reason : (c->armed ? 0u : ROBOT_STOP_NEUTRAL_REQUIRED);
    out->abort_pending = was_armed && !c->armed;
    out->abort_wheel_pending =
        ROBOT_WHEEL_POWER_LIMIT_ENABLED && previous_power > 0.0f && c->power.k_applied == 0.0f;
    out->armed = c->armed;
    out->reason = c->stop_reason;
    out->count = MotorOutput_BuildFrames(&c->motor_output, in->dm, c->dji_command, c->dm_target,
                                         c->armed, c->step_count++, out->frames);
}
