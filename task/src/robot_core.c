#include "robot_core.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#define BIT(n) (1u << (n))
_Static_assert(ROBOT_DJI_PERIOD_MS >= 2u, "DJI command frequency must be below 1000Hz");
_Static_assert(ROBOT_DM_PERIOD_MS >= DM_COUNT, "DM slots must not overlap");
uint32_t RobotCore_ExpectedDji(uint8_t p) {
    return (p == ROBOT_PROFILE_CHASSIS ? 0x0fu : 0u) | (p == ROBOT_PROFILE_ARM ? 0x70u : 0u) |
           (p == ROBOT_PROFILE_FULL ? 0x7fu : 0u);
}
uint32_t RobotCore_ExpectedDm(uint8_t p) {
    return p == ROBOT_PROFILE_FULL ? 7u
                                   : (p == ROBOT_PROFILE_CHASSIS ? 3u : (p == ROBOT_PROFILE_ARM ? 4u : 0u));
}
void RobotCore_Init(RobotCore *c) {
    memset(c, 0, sizeof(*c));
    c->profile = 255u;
    PowerLimiter_Init(&c->power);
}
static bool config_valid(void) {
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        const DjiConfig *d = &g_dji_config[i];
        if (!isfinite(d->reduction) || d->reduction <= 0 || fabsf(d->direction) != 1.0f ||
            d->max_command <= 0 || !isfinite(d->max_command))
            return false;
        float protocol_limit = i < 4 ? 16384.0f : (i == DJI_YAW ? 30000.0f : 10000.0f);
        if (d->max_command > protocol_limit || !isfinite(d->speed_pid.kp) || !isfinite(d->speed_pid.ki) ||
            !isfinite(d->speed_pid.integral_limit) || d->speed_pid.integral_limit < 0 ||
            !isfinite(d->speed_pid.output_limit) || d->speed_pid.output_limit <= 0)
            return false;
        if (d->bus != (i < 4 ? 1 : 2))
            return false;
        for (unsigned j = 0; j < i; j++)
            if (d->bus == g_dji_config[j].bus && d->feedback_id == g_dji_config[j].feedback_id)
                return false;
    }
    /* DJI 分组槽位和解析 ID 是固定协议；更改 ID 时需同时改帧编组。 */
    const uint16_t ids[DJI_COUNT] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x201, 0x202};
    for (unsigned i = 0; i < DJI_COUNT; i++)
        if (g_dji_config[i].feedback_id != ids[i])
            return false;
    for (unsigned i = 0; i < DM_COUNT; i++) {
        const DmConfig *d = &g_dm_config[i];
        if (d->motor_id < 1 || d->motor_id > 15 || d->master_id > 0x7ffu || d->master_id < 0x300u ||
            fabsf(d->direction) != 1.0f || !isfinite(d->external_reduction) || d->external_reduction <= 0 ||
            !isfinite(d->p_max) || d->p_max <= 0 || !isfinite(d->v_max) || d->v_max <= 0 ||
            !isfinite(d->t_max) || d->t_max <= 0 || !isfinite(d->velocity_limit_rad_s) ||
            d->velocity_limit_rad_s <= 0)
            return false;
        for (unsigned j = 0; j < i; j++)
            if (d->motor_id == g_dm_config[j].motor_id || d->master_id == g_dm_config[j].master_id)
                return false;
    }
    for (unsigned i = 0; i < JOINT_COUNT; i++) {
        const JointConfig *j = &g_joint_config[i];
        if (!isfinite(j->min_deg) || !isfinite(j->max_deg) || !isfinite(j->boot_joint_deg) ||
            !isfinite(j->slew_deg_s) || j->min_deg >= j->max_deg || j->slew_deg_s <= 0 ||
            j->boot_joint_deg < j->min_deg || j->boot_joint_deg > j->max_deg || !isfinite(j->position_kp) ||
            j->position_kp < 0 || !isfinite(j->max_speed_deg_s) || j->max_speed_deg_s <= 0)
            return false;
    }
    return true;
}
static void measurements(RobotCore *c, const RobotInput *in) {
    for (unsigned i = 0; i < 3; i++) {
        unsigned mi = DJI_YAW + i;
        c->joint_measured[i] =
            Dji_JointDegrees(&in->dji[mi], &g_dji_config[mi]) + g_joint_config[i].boot_joint_deg;
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        if (in->dm[i].seen && !c->dm_origin_valid[i]) {
            c->dm_boot_pos[i] = in->dm[i].position;
            c->dm_origin_valid[i] = true;
        }
        const DmConfig *d = &g_dm_config[i];
        c->joint_measured[JOINT_LIFT_L + i] =
            (in->dm[i].position - c->dm_boot_pos[i]) * d->direction * ROBOT_RAD2DEG / d->external_reduction +
            g_joint_config[JOINT_LIFT_L + i].boot_joint_deg;
    }
}
static uint32_t health(RobotCore *c, const RobotInput *in, uint32_t now, uint32_t djmask, uint32_t dmmask) {
    uint32_t why = 0;
    if (!in->rc.valid || (uint32_t)(now - in->rc.stamp_ms) >= ROBOT_RC_TIMEOUT_MS ||
        in->rc.bad_frames != c->last_bad_frames)
        why |= ROBOT_STOP_RC;
    c->last_bad_frames = in->rc.bad_frames;
    if (in->profile > ROBOT_PROFILE_FULL || !config_valid() ||
        (in->profile != ROBOT_PROFILE_RC_ONLY && !in->config_confirmed))
        why |= ROBOT_STOP_CONFIG;
    if (in->emergency_stop)
        why |= ROBOT_STOP_ESTOP;
    if (in->hardware_limit_mask)
        why |= ROBOT_STOP_LIMIT;
    if (in->deadline_fault || (c->initialized && (uint32_t)(now - c->last_ms) > 5u))
        why |= ROBOT_STOP_TIMING;
    uint8_t buses = (djmask & 15u) ? 1u : 0u;
    if ((djmask & 0x70u) || dmmask)
        buses |= 2u;
    if (in->bus_fault_mask & buses)
        why |= ROBOT_STOP_BUS;
    for (unsigned i = 0; i < DJI_COUNT; i++)
        if (djmask & BIT(i)) {
            const DjiFeedback *d = &in->dji[i];
            if (!d->seen || (uint32_t)(now - d->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS)
                why |= ROBOT_STOP_FEEDBACK;
            if (i <= DJI_YAW && d->temperature >= ROBOT_MAX_TEMP_C)
                why |= ROBOT_STOP_TEMPERATURE;
            /* 多圈角度离线期间不可恢复：pitch/夹爪/yaw 需检查姿态后重启标定。
             * 四轮速度控制不依赖累计位置，因此不受此永久锁影响。 */
            if (i >= DJI_YAW && d->position_lost)
                why |= ROBOT_STOP_POSITION_LOST;
        }
    for (unsigned i = 0; i < DM_COUNT; i++)
        if (dmmask & BIT(i)) {
            const DmFeedback *d = &in->dm[i];
            if (!d->seen || (uint32_t)(now - d->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS)
                why |= ROBOT_STOP_FEEDBACK;
            if ((d->state != 0u && d->state != 1u) || (d->state == 0u && c->dm_active_seen[i]))
                why |= ROBOT_STOP_DM_ERROR;
            if (d->mos_temperature >= ROBOT_MAX_TEMP_C || d->rotor_temperature >= ROBOT_MAX_TEMP_C)
                why |= ROBOT_STOP_TEMPERATURE;
            if (c->dm_enable_waiting[i] &&
                (uint32_t)(now - c->dm_enable_since[i]) >= ROBOT_DM_ENABLE_TIMEOUT_MS && d->state != 1u)
                why |= ROBOT_STOP_DM_ERROR;
        }
    for (unsigned j = 0; j < JOINT_COUNT; j++) {
        bool used = j < 3 ? ((djmask & BIT(DJI_YAW + j)) != 0) : ((dmmask & BIT(j - JOINT_LIFT_L)) != 0);
        if (used &&
            (!isfinite(c->joint_measured[j]) || c->joint_measured[j] < g_joint_config[j].min_deg - 2.0f ||
             c->joint_measured[j] > g_joint_config[j].max_deg + 2.0f))
            why |= ROBOT_STOP_LIMIT;
    }
    if ((dmmask & 3u) == 3u &&
        fabsf(c->joint_measured[JOINT_LIFT_L] - c->joint_measured[JOINT_LIFT_R]) > ROBOT_LIFT_MAX_SKEW_DEG)
        why |= ROBOT_STOP_LIFT_SKEW;
    return why;
}
static void reset_control(RobotCore *c) {
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        Pid_Reset(&c->speed_pid[i]);
        c->dji_command[i] = 0;
    }
    memset(c->wheel_reference, 0, sizeof(c->wheel_reference));
    PowerLimiter_ResetOutput(&c->power);
    c->grip_hold = false;
    c->grip_contact_tracking = false;
    for (unsigned j = 0; j < JOINT_COUNT; j++)
        c->joint_target[j] =
            Control_Clamp(c->joint_measured[j], g_joint_config[j].min_deg, g_joint_config[j].max_deg);
    memset(c->dm_enable_waiting, 0, sizeof(c->dm_enable_waiting));
    memset(c->dm_active_seen, 0, sizeof(c->dm_active_seen));
    memset(c->dm_preloaded, 0, sizeof(c->dm_preloaded));
}
static void advance(RobotCore *c, unsigned j, float input) {
    const JointConfig *cfg = &g_joint_config[j];
    c->joint_target[j] = Control_Clamp(c->joint_target[j] + input * cfg->slew_deg_s * ROBOT_CONTROL_DT,
                                       cfg->min_deg, cfg->max_deg);
}
static float joint_speed_target(const RobotCore *c, unsigned j) {
    const JointConfig *cfg = &g_joint_config[j];
    float v = Control_Clamp((c->joint_target[j] - c->joint_measured[j]) * cfg->position_kp,
                            -cfg->max_speed_deg_s, cfg->max_speed_deg_s);
    if ((c->joint_measured[j] <= cfg->min_deg && v < 0) || (c->joint_measured[j] >= cfg->max_deg && v > 0))
        v = 0;
    return v;
}
static void controls(RobotCore *c, const RobotInput *in, uint32_t now, uint32_t djmask, uint32_t dmmask) {
    const RcState *r = &in->rc;
    for (unsigned i = 0; i < DM_COUNT; i++)
        if ((dmmask & BIT(i)) && in->dm[i].state == 1u)
            c->dm_active_seen[i] = true;
    if (djmask & 15u) {
        float f = Rc_Axis(r, 3) * ROBOT_RC_FORWARD_SIGN, t = Rc_Axis(r, 2) * ROBOT_RC_TURN_SIGN;
        float left = f + t, right = f - t, scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
        float demand[4] = {left / scale, right / scale, right / scale, left / scale};
        float old_integral[4], error[4], requested_logical[4];
        int16_t requested[4], measured[4], rpm[4], limited[4];
        for (unsigned i = 0; i < 4; i++) {
            c->wheel_reference[i] = Control_Slew(c->wheel_reference[i], demand[i] * ROBOT_WHEEL_MAX_RPM,
                                                 ROBOT_WHEEL_ACCEL_RPM_S * ROBOT_CONTROL_DT);
            const DjiConfig *cfg = &g_dji_config[i];
            float actual = in->dji[i].rpm * cfg->direction;
            old_integral[i] = c->speed_pid[i].integral;
            error[i] = c->wheel_reference[i] - actual;
            float u = Pid_Update(&c->speed_pid[i], &cfg->speed_pid, error[i], ROBOT_CONTROL_DT);
            requested_logical[i] = u;
            requested[i] = (int16_t)(cfg->direction * Control_Clamp(u, -cfg->max_command, cfg->max_command));
            measured[i] = in->dji[i].current;
            rpm[i] = in->dji[i].rpm;
        }
        /* 只传入四轮数组：CAN2后腿和机械臂从未进入功率模型。 */
        PowerLimiter_Apply(&c->power, &g_power_config, &in->power, requested, measured, rpm, now,
                           ROBOT_CONTROL_DT, limited);
        for (unsigned i = 0; i < 4; i++) {
            const DjiConfig *cfg = &g_dji_config[i];
            c->dji_command[i] = limited[i];
            float applied_logical = (float)limited[i] * cfg->direction;
            bool constrained =
                c->power.k_applied < 0.999999f || fabsf(requested_logical[i]) > cfg->max_command;
            /* 功率约束在PI之后：冻结继续向受限方向累积的积分，允许卸载积分。
             * 正常的不足1 LSB量化差不触发冻结。 */
            if (constrained && (requested_logical[i] - applied_logical) * error[i] > 0.0f)
                c->speed_pid[i].integral = old_integral[i];
        }
    }
    if (djmask & 0x70u) {
        uint8_t mode = r->sw[ROBOT_RC_RIGHT_SWITCH];
        float horizontal = Rc_Axis(r, 0), vertical = Rc_Axis(r, 1) * ROBOT_RC_PITCH_SIGN;
        if (mode == RC_UP) {
            advance(c, JOINT_YAW, horizontal * ROBOT_RC_YAW_SIGN);
            if (in->dm[DM_BIG_PITCH].state == 1u)
                advance(c, JOINT_BIG_PITCH, vertical);
        } else if (mode == RC_DOWN) {
            advance(c, JOINT_SMALL_PITCH, vertical);
            float close = horizontal * ROBOT_RC_GRIP_SIGN;
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
            const DjiConfig *cfg = &g_dji_config[i];
            if (j == JOINT_GRIP && c->grip_hold) {
                Pid_Reset(&c->speed_pid[i]);
                c->dji_command[i] =
                    (int16_t)(cfg->direction * Control_Clamp(ROBOT_GRIP_HOLD_CURRENT, 0, cfg->max_command));
            } else {
                float rpm = joint_speed_target(c, j) * cfg->reduction / 6.0f;
                float actual = in->dji[i].rpm * cfg->direction;
                float u = Pid_Update(&c->speed_pid[i], &cfg->speed_pid, rpm - actual, ROBOT_CONTROL_DT);
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
    if ((dmmask & 3u) == 3u && in->dm[0].state == 1u && in->dm[1].state == 1u) {
        uint8_t sw = r->sw[ROBOT_RC_LEFT_SWITCH];
        float cmd = sw == RC_UP ? 1.0f : (sw == RC_DOWN ? -1.0f : 0.0f);
        float lo = fmaxf(g_joint_config[JOINT_LIFT_L].min_deg, g_joint_config[JOINT_LIFT_R].min_deg);
        float hi = fminf(g_joint_config[JOINT_LIFT_L].max_deg, g_joint_config[JOINT_LIFT_R].max_deg);
        float speed = fminf(g_joint_config[JOINT_LIFT_L].slew_deg_s, g_joint_config[JOINT_LIFT_R].slew_deg_s);
        float common = Control_Clamp((c->joint_target[JOINT_LIFT_L] + c->joint_target[JOINT_LIFT_R]) * 0.5f +
                                         cmd * speed * ROBOT_CONTROL_DT,
                                     lo, hi);
        if (cmd != 0.0f) {
            c->joint_target[JOINT_LIFT_L] = common;
            c->joint_target[JOINT_LIFT_R] = common;
        }
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        const DmConfig *cfg = &g_dm_config[i];
        unsigned j = JOINT_LIFT_L + i;
        c->dm_target[i] = c->dm_boot_pos[i] + (c->joint_target[j] - g_joint_config[j].boot_joint_deg) *
                                                  ROBOT_DEG2RAD * cfg->direction * cfg->external_reduction;
    }
}
static void frames(RobotCore *c, const RobotInput *in, uint32_t now, RobotOutput *o, uint32_t dmmask) {
    unsigned slot = c->step_count;
    if (slot % ROBOT_DJI_PERIOD_MS == 0u) {
        Dji_GroupFrame(&o->frames[o->count++], 1, 0x200, c->dji_command);
        int16_t v[4] = {c->dji_command[DJI_SMALL_PITCH], c->dji_command[DJI_GRIP], 0, 0};
        Dji_GroupFrame(&o->frames[o->count++], 2, 0x200, v);
    }
    if (slot % ROBOT_DJI_PERIOD_MS == 1u) {
        int16_t v[4] = {c->dji_command[DJI_YAW], 0, 0, 0};
        Dji_GroupFrame(&o->frames[o->count++], 2, 0x1ff, v);
    }
    for (unsigned i = 0; i < DM_COUNT; i++)
        if (slot % ROBOT_DM_PERIOD_MS == i) {
            bool active = c->armed && (dmmask & BIT(i));
            MotorFrame *f = &o->frames[o->count++];
            if (!active) {
                Dm_SpecialFrame(f, &g_dm_config[i], false);
                c->dm_enable_waiting[i] = false;
            } else if (in->dm[i].state != 1u) {
                if (!c->dm_enable_waiting[i]) {
                    c->dm_enable_since[i] = now;
                    c->dm_enable_waiting[i] = true;
                }
                /* 先用当前位置预装 PV 目标，再在下一个 5ms 时隙使能。
                 * 不发保存零点/清错命令；实际固件使能行为仍需空载验收。 */
                if (!c->dm_preloaded[i]) {
                    Dm_PositionFrame(f, &g_dm_config[i], in->dm[i].position,
                                     g_dm_config[i].velocity_limit_rad_s);
                    c->dm_preloaded[i] = true;
                } else
                    Dm_SpecialFrame(f, &g_dm_config[i], true);
            } else {
                c->dm_enable_waiting[i] = false;
                Dm_PositionFrame(f, &g_dm_config[i], c->dm_target[i], g_dm_config[i].velocity_limit_rad_s);
            }
        }
}
void RobotCore_Step(RobotCore *c, const RobotInput *in, uint32_t now, RobotOutput *o) {
    memset(o, 0, sizeof(*o));
    bool was_armed = c->armed;
    float prior_power_k = c->power.k_applied;
    uint32_t djmask = RobotCore_ExpectedDji(in->profile), dmmask = RobotCore_ExpectedDm(in->profile);
    measurements(c, in);
    uint32_t reason = health(c, in, now, djmask, dmmask);
    if (c->profile != in->profile) {
        c->armed = false;
        c->neutral_tracking = false;
        c->profile = in->profile;
        reset_control(c);
    }
    if (reason) {
        c->armed = false;
        c->neutral_tracking = false;
        reset_control(c);
    } else if (!c->armed) {
        if (Rc_IsNeutral(&in->rc)) {
            if (!c->neutral_tracking) {
                c->neutral_since_ms = now;
                c->neutral_tracking = true;
            }
            if ((uint32_t)(now - c->neutral_since_ms) >= ROBOT_NEUTRAL_DWELL_MS) {
                reset_control(c);
                c->armed = true;
                c->neutral_tracking = false;
            }
        } else
            c->neutral_tracking = false;
    }
    if (c->armed) {
        controls(c, in, now, djmask, dmmask);
        for (unsigned i = 0; i < DM_COUNT; i++)
            if ((dmmask & BIT(i)) &&
                (!isfinite(c->dm_target[i]) || fabsf(c->dm_target[i]) > g_dm_config[i].p_max))
                reason |= ROBOT_STOP_LIMIT;
        if (reason) {
            c->armed = false;
            reset_control(c);
        }
    } else
        memset(c->dji_command, 0, sizeof(c->dji_command));
    c->stop_reason = reason ? reason : (c->armed ? 0u : ROBOT_STOP_NEUTRAL_REQUIRED);
    o->abort_pending = was_armed && !c->armed;
    o->abort_wheel_pending = (djmask & 15u) && prior_power_k > 0.0f && c->power.k_applied == 0.0f;
    o->armed = c->armed;
    o->reason = c->stop_reason;
    frames(c, in, now, o, dmmask);
    ++c->step_count;
    c->last_ms = now;
    c->initialized = true;
}
