#include "gimbal_control.h"
#include <math.h>

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
void Gimbal_Step(RobotCore *c, const RobotInput *in, uint32_t now) {
    const RcState *r = &in->rc;
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
