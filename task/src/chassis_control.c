#include "chassis_control.h"
#include <math.h>

void Chassis_WheelStep(const RcState *rc, const DjiFeedback dji[4], Pid speed_pid[4], int16_t command[4]) {
    float forward = Rc_Axis(rc, 3) * ROBOT_RC_FORWARD_SIGN;
    float turn = Rc_Axis(rc, 2) * ROBOT_RC_TURN_SIGN;
    float left = forward + turn;
    float right = forward - turn;
    float scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
    const float demand[4] = {[DJI_LF] = left, [DJI_RF] = right, [DJI_LR] = left, [DJI_RR] = right};
    for (unsigned i = 0; i < 4; i++) {
        const DjiConfig *motor = &g_dji_config[i];
        float wheel_rpm = demand[i] * ROBOT_WHEEL_MAX_OUTPUT_RPM / scale;
        float rotor_target_rpm = wheel_rpm * motor->reduction;
        float error = rotor_target_rpm - dji[i].rpm * motor->direction;
        float output = Pid_Update(&speed_pid[i], &motor->speed_pid, error, ROBOT_CONTROL_DT);
        command[i] = (int16_t)(motor->direction * Control_Clamp(output, -motor->max_command, motor->max_command));
    }
}

static void lift_travel_bounds(const RobotCore *c, unsigned side, const JointConfig *joint,
                               float *lower, float *upper) {
    const DmConfig *motor = &g_dm_config[side];
    float deg_per_rad = ROBOT_RAD2DEG / motor->external_reduction;
    float center = joint->boot_joint_deg - c->dm_boot_pos[side] * motor->direction * deg_per_rad;
    /* 给运动边界留一个反馈 LSB，上电位置本身始终可作为返回目标。 */
    float half_range = motor->p_max * (1.0f - 2.0f / 65535.0f) * deg_per_rad;
    *lower = fmaxf(joint->min_deg, fminf(center - half_range, joint->boot_joint_deg));
    *upper = fminf(joint->max_deg, fmaxf(center + half_range, joint->boot_joint_deg));
}
static void paired_lift_travel_bounds(const RobotCore *c, const JointConfig *left, const JointConfig *right,
                                      float *lower, float *upper) {
    float right_lower, right_upper;
    lift_travel_bounds(c, DM_LIFT_L, left, lower, upper);
    lift_travel_bounds(c, DM_LIFT_R, right, &right_lower, &right_upper);
    *lower = fmaxf(*lower, right_lower);
    *upper = fminf(*upper, right_upper);
}
static float lift_speed_limit(unsigned side) {
    const DmConfig *motor = &g_dm_config[side];
    return motor->velocity_limit_rad_s * ROBOT_RAD2DEG / motor->external_reduction;
}
static void lift(RobotCore *c, unsigned side, float cmd, float speed, float lower, float upper) {
    if (cmd == 0.0f)
        return;
    unsigned axis = JOINT_LIFT_L + side;
    if (c->lift_wait_mask) {
        unsigned held_axis = side == DM_LIFT_L ? JOINT_LIFT_R : JOINT_LIFT_L;
        float held = Control_Clamp(c->joint_target[held_axis], lower, upper);
        if (cmd > 0.0f)
            upper = held;
        else
            lower = held;
    }
    float previous = Control_Clamp(c->joint_target[axis], lower, upper);
    float measured = c->joint_measured[axis];
    float lead = ROBOT_LIFT_MAX_LEAD_DEG;
    lower = fmaxf(lower, fminf(previous, measured - lead));
    upper = fminf(upper, fmaxf(previous, measured + lead));

    float target = Control_Clamp(previous + cmd * speed * ROBOT_CONTROL_DT, lower, upper);
    c->joint_target[axis] = target;
}
static void paired_lift(RobotCore *c, float cmd) {
    float lower, upper;
    paired_lift_travel_bounds(c, &g_joint_config[JOINT_LIFT_L], &g_joint_config[JOINT_LIFT_R],
                              &lower, &upper);
    float skew_in_travel_direction = cmd * c->lift_skew_deg;
    float skew_abs = fabsf(c->lift_skew_deg);
    uint8_t leader = skew_in_travel_direction > 0.0f ? (1u << DM_LIFT_L) : (1u << DM_LIFT_R);
    uint8_t wait = c->lift_wait_mask;
    if (cmd == 0.0f)
        wait = 0u;
    else if (skew_abs >= ROBOT_LIFT_MAX_SKEW_DEG)
        wait = leader;
    else if (skew_abs <= ROBOT_LIFT_RESUME_SKEW_DEG || wait != leader ||
             cmd != c->lift_direction)
        wait = 0u;

    if (wait && (wait != c->lift_wait_mask || cmd != c->lift_direction)) {
        unsigned axis = wait == (1u << DM_LIFT_L) ? JOINT_LIFT_L : JOINT_LIFT_R;
        c->joint_target[axis] = Control_Clamp(c->joint_measured[axis], lower, upper);
    }
    c->lift_wait_mask = wait;
    c->lift_direction = cmd;
    float speed = fminf(g_joint_config[JOINT_LIFT_L].slew_deg_s, g_joint_config[JOINT_LIFT_R].slew_deg_s);
    speed = fminf(speed, fminf(lift_speed_limit(DM_LIFT_L), lift_speed_limit(DM_LIFT_R)));
    for (unsigned side = DM_LIFT_L; side <= DM_LIFT_R; side++)
        lift(c, side, (wait & (1u << side)) ? 0.0f : cmd, speed, lower, upper);
}
void Chassis_LiftStep(RobotCore *c, const RobotInput *input) {
    bool ready = input->dm[DM_LIFT_L].state == 1u && input->dm[DM_LIFT_R].state == 1u;
    if (!ready || !c->dm_active_seen[DM_LIFT_L] || !c->dm_active_seen[DM_LIFT_R]) {
        /* 先使能的一侧保持，双侧就绪后从当时姿态开始。 */
        for (unsigned side = DM_LIFT_L; side <= DM_LIFT_R; side++)
            if (ready || !c->dm_active_seen[side]) {
                unsigned axis = JOINT_LIFT_L + side;
                const JointConfig *joint = &g_joint_config[axis];
                c->joint_target[axis] = Control_Clamp(c->joint_measured[axis], joint->min_deg, joint->max_deg);
            }
        return;
    }
    uint8_t sw = input->rc.sw[ROBOT_RC_LEFT_SWITCH];
    float direction = sw == RC_UP ? 1.0f : (sw == RC_DOWN ? -1.0f : 0.0f);
    paired_lift(c, direction);
}
