#include "gimbal_control.h"
#include <math.h>

typedef struct {
    float small_pitch_deg;
    float big_pitch_deg[3];
    float current_command[3];
} SmallPitchGravityRow;

/* 电流为 C610 原生符号。31° 和 32° 的样本合并到 31.5° 组。 */
static const SmallPitchGravityRow small_pitch_gravity[] = {
    {22.0f, {25.0f, 39.0f, 64.0f}, {-1400.0f, -1000.0f, -1143.0f}},
    {31.5f, {25.0f, 46.0f, 64.0f}, {-2289.0f, -2427.0f, -2376.0f}},
    {35.0f, {25.0f, 42.0f, 71.0f}, {-1811.0f, -1811.0f, -1812.0f}},
};

static float interpolate(float x, float x0, float x1, float y0, float y1) {
    const float fraction = Control_Clamp((x - x0) / (x1 - x0), 0.0f, 1.0f);
    return y0 + fraction * (y1 - y0);
}

static float gravity_row_current(const SmallPitchGravityRow *row, float big_pitch_deg) {
    const unsigned lower = big_pitch_deg <= row->big_pitch_deg[1] ? 0u : 1u;
    const unsigned upper = lower + 1u;
    return interpolate(big_pitch_deg, row->big_pitch_deg[lower], row->big_pitch_deg[upper],
                       row->current_command[lower], row->current_command[upper]);
}

static void update_small_pitch_gravity(RobotCore *c) {
    const float small_deg = c->joint_measured[JOINT_SMALL_PITCH];
    const float big_deg = c->joint_measured[JOINT_BIG_PITCH];
    const unsigned lower = small_deg <= small_pitch_gravity[1].small_pitch_deg ? 0u : 1u;
    const SmallPitchGravityRow *left = &small_pitch_gravity[lower];
    const SmallPitchGravityRow *right = &small_pitch_gravity[lower + 1u];
    const float current = interpolate(small_deg, left->small_pitch_deg, right->small_pitch_deg,
                                      gravity_row_current(left, big_deg), gravity_row_current(right, big_deg));
    const float ramp_start_deg = small_pitch_gravity[0].small_pitch_deg - 10.0f;
    const float weight = Control_Clamp((small_deg - ramp_start_deg) / 10.0f, 0.0f, 1.0f);
    c->small_pitch_gravity_ff_command = Control_Slew(c->small_pitch_gravity_ff_command, weight * current,
                                                    4000.0f * ROBOT_CONTROL_DT);
}

static void advance(RobotCore *c, unsigned j, float input) {
    const JointConfig *cfg = &g_joint_config[j];
    c->joint_target[j] = Control_Clamp(c->joint_target[j] + input * cfg->slew_deg_s * ROBOT_CONTROL_DT,
                                       cfg->min_deg, cfg->max_deg);
}
static float joint_speed_target(const RobotCore *c, unsigned j, const JointConfig *cfg, float damping) {
    float v = (c->joint_target[j] - c->joint_measured[j]) * cfg->position_kp - damping;
    if (cfg->max_speed_deg_s > 0.0f)
        v = Control_Clamp(v, -cfg->max_speed_deg_s, cfg->max_speed_deg_s);
    if ((c->joint_measured[j] <= cfg->min_deg && v < 0) || (c->joint_measured[j] >= cfg->max_deg && v > 0))
        v = 0;
    return v;
}
static float m2006_speed_pd(JointSpeedPd *state, const PidConfig *cfg, float target_rpm, float actual_rpm) {
    if (state->sampled) {
        const float acceleration = (actual_rpm - state->previous_rpm) / ROBOT_CONTROL_DT;
        const float alpha = ROBOT_CONTROL_DT / (ROBOT_ARM_M2006_SPEED_D_FILTER_S + ROBOT_CONTROL_DT);
        state->acceleration += alpha * (acceleration - state->acceleration);
    }
    state->previous_rpm = actual_rpm;
    state->sampled = true;

    return cfg->kp * (target_rpm - actual_rpm) -
           ROBOT_ARM_M2006_SPEED_KD * state->acceleration;
}

static void apply_yaw_tuning(JointConfig *joint, PidConfig *pid) {
    YawTuning tuning = g_yaw_tuning;
    if (isfinite(tuning.position_kp) && tuning.position_kp >= 0.0f)
        joint->position_kp = tuning.position_kp;
    if (isfinite(tuning.speed_pid.kp) && tuning.speed_pid.kp >= 0.0f)
        pid->kp = tuning.speed_pid.kp;
    if (isfinite(tuning.speed_pid.ki) && tuning.speed_pid.ki >= 0.0f)
        pid->ki = tuning.speed_pid.ki;
    if (isfinite(tuning.speed_pid.output_limit) && tuning.speed_pid.output_limit >= 0.0f)
        pid->output_limit = fminf(tuning.speed_pid.output_limit, pid->output_limit);
    if (isfinite(tuning.speed_pid.integral_limit) && tuning.speed_pid.integral_limit >= 0.0f)
        pid->integral_limit = fminf(tuning.speed_pid.integral_limit, pid->output_limit);
    pid->integral_limit = fminf(pid->integral_limit, pid->output_limit);
}

static void update_big_pitch_gravity(RobotCore *c, const DmFeedback *feedback) {
    const DmConfig *motor = &g_dm_config[DM_BIG_PITCH];
    if (feedback->state != 1u || !c->dm_active_seen[DM_BIG_PITCH] || motor->mode != DM_MODE_MIT ||
        motor->mit_ff_limit_nm == 0.0f) {
        c->big_pitch_gravity_ff_nm = 0.0f;
        return;
    }

    const float angle = c->joint_measured[JOINT_BIG_PITCH];
    const float fitted_deg = Control_Clamp(angle, ROBOT_BIG_PITCH_GRAVITY_MIN_DEG,
                                           ROBOT_BIG_PITCH_GRAVITY_MAX_DEG);
    const float weight = fmaxf(0.0f, 1.0f - fabsf(angle - fitted_deg) / ROBOT_BIG_PITCH_GRAVITY_FADE_DEG);
    const float theta = fitted_deg * ROBOT_DEG2RAD;
    const float torque = weight * (ROBOT_BIG_PITCH_GRAVITY_COS_NM * cosf(theta) +
                                    ROBOT_BIG_PITCH_GRAVITY_SIN_NM * sinf(theta));
    const float target = Control_Clamp(torque, -motor->mit_ff_limit_nm, motor->mit_ff_limit_nm);
    c->big_pitch_gravity_ff_nm = Control_Slew(c->big_pitch_gravity_ff_nm, target,
                                             ROBOT_BIG_PITCH_GRAVITY_SLEW_NM_S * ROBOT_CONTROL_DT);
}

void Gimbal_Step(RobotCore *c, const RobotInput *in, uint32_t now) {
    update_big_pitch_gravity(c, &in->dm[DM_BIG_PITCH]);
    update_small_pitch_gravity(c);
    const RcState *r = &in->rc;
    uint8_t mode = r->sw[ROBOT_RC_RIGHT_SWITCH];
    float horizontal = Rc_Axis(r, 0), vertical = Rc_Axis(r, 1) * ROBOT_RC_PITCH_SIGN;
    if (mode == RC_UP) {
        advance(c, JOINT_YAW, horizontal * ROBOT_RC_YAW_SIGN);
        if (in->dm[DM_BIG_PITCH].state == 1u)
            advance(c, JOINT_BIG_PITCH, vertical);
    } else if (mode == RC_DOWN) {
        advance(c, JOINT_SMALL_PITCH, vertical);
        float close = ROBOT_GRIP_INSTALLED ? horizontal * ROBOT_RC_GRIP_SIGN : 0.0f;
        if (close < 0) {
            c->grip_hold = false;
            c->grip_contact_tracking = false;
        }
        if (!c->grip_hold)
            advance(c, JOINT_GRIP, close);
        float grip_speed = fabsf(in->dji[DJI_GRIP].rpm * 6.0f / g_dji_config[DJI_GRIP].reduction);
        float grip_current = in->dji[DJI_GRIP].current * g_dji_config[DJI_GRIP].direction;
        bool contact = close > 0 && grip_current >= ROBOT_GRIP_CONTACT_CURRENT &&
                       grip_speed <= ROBOT_GRIP_STALL_SPEED_DEG_S;
        if (contact) {
            if (!c->grip_contact_tracking) {
                c->grip_contact_tracking = true;
                c->grip_contact_since_ms = now;
            }
            if ((uint32_t)(now - c->grip_contact_since_ms) >= ROBOT_GRIP_CONTACT_MS) {
                c->grip_hold = true;
                c->joint_target[JOINT_GRIP] = c->joint_measured[JOINT_GRIP];
            }
        } else
            c->grip_contact_tracking = false;
    }
    for (unsigned j = 0; j < 3; j++) {
        unsigned i = DJI_YAW + j;
        if (i == DJI_GRIP && !ROBOT_GRIP_INSTALLED) {
            c->dji_command[i] = 0;
            continue;
        }
        const DjiConfig *cfg = &g_dji_config[i];
        if (j == JOINT_GRIP && c->grip_hold) {
            c->arm_m2006_speed_pd[JOINT_GRIP - JOINT_SMALL_PITCH] = (JointSpeedPd){0};
            c->dji_command[i] =
                (int16_t)(cfg->direction * Control_Clamp(ROBOT_GRIP_HOLD_CURRENT, 0, cfg->max_command));
        } else {
            JointConfig joint = g_joint_config[j];
            PidConfig pid = cfg->speed_pid;
            if (j == JOINT_YAW) {
                apply_yaw_tuning(&joint, &pid);
                c->speed_pid[i].integral = Control_Clamp(c->speed_pid[i].integral,
                                                        -pid.integral_limit, pid.integral_limit);
            }
            const float actual = in->dji[i].rpm * cfg->direction;
            /* 小 pitch 和夹爪的 D 项基于反馈。 */
            const float damping = j == JOINT_YAW
                                      ? 0.0f
                                      : ROBOT_ARM_M2006_POSITION_KD * (actual * 6.0f / cfg->reduction);
            const float rpm = joint_speed_target(c, j, &joint, damping) * cfg->reduction / 6.0f;
            float u;
            if (j == JOINT_YAW) {
                u = Pid_Update(&c->speed_pid[i], &pid, rpm - actual, ROBOT_CONTROL_DT);
            } else {
                u = m2006_speed_pd(&c->arm_m2006_speed_pd[j - JOINT_SMALL_PITCH], &pid, rpm, actual);
                if (j == JOINT_SMALL_PITCH)
                    u += cfg->direction * c->small_pitch_gravity_ff_command;
                u = Control_Clamp(u, -pid.output_limit, pid.output_limit);
            }
            /* 软限位处清积分并阻止电流继续向外；无机械制动保证。 */
            if ((c->joint_measured[j] <= g_joint_config[j].min_deg && u < 0) ||
                (c->joint_measured[j] >= g_joint_config[j].max_deg && u > 0)) {
                u = 0;
                Pid_Reset(&c->speed_pid[i]);
            }
            c->dji_command[i] =
                (int16_t)(cfg->direction * Control_Clamp(u, -cfg->max_command, cfg->max_command));
        }
    }
}
