#include "motor_output.h"

_Static_assert(ROBOT_DJI_PERIOD_MS >= 2u, "DJI groups require separate slots");
_Static_assert(ROBOT_DM_PERIOD_MS >= DM_COUNT, "DM slots must not overlap");

unsigned MotorOutput_BuildFrames(MotorOutput *output, const DmFeedback dm[DM_COUNT],
                                 const int16_t dji_command[DJI_COUNT], const float dm_target[DM_COUNT],
                                 bool enabled, uint32_t tick,
                                 MotorFrame frames[MOTOR_OUTPUT_MAX_FRAMES]) {
    unsigned count = 0;
    if (tick % ROBOT_DJI_PERIOD_MS == 0u) {
        const int16_t zero[4] = {0};
        Dji_GroupFrame(&frames[count++], 1, 0x200, enabled ? dji_command : zero);
        int16_t arm[4] = {
            [ROBOT_SMALL_PITCH_MOTOR_ID - 1u] = enabled ? dji_command[DJI_SMALL_PITCH] : 0,
            [ROBOT_GRIP_MOTOR_ID - 1u] = enabled ? dji_command[DJI_GRIP] : 0,
        };
        Dji_GroupFrame(&frames[count++], 2, 0x200, arm);
    } else if (tick % ROBOT_DJI_PERIOD_MS == 1u) {
        int16_t yaw[4] = {
            [ROBOT_YAW_MOTOR_ID - 1u] = enabled ? dji_command[DJI_YAW] : 0,
        };
        Dji_GroupFrame(&frames[count++], 2, 0x1ff, yaw);
    }

    unsigned side = tick % ROBOT_DM_PERIOD_MS;
    if (side >= DM_COUNT)
        return count;

    MotorFrame *frame = &frames[count++];
    const DmConfig *motor = &g_dm_config[side];
    if (!enabled) {
        output->preloaded[side] = false;
        Dm_SpecialFrame(frame, motor, false);
    } else if (dm[side].state == 1u) {
        Dm_PositionFrame(frame, motor, dm_target[side], motor->velocity_limit_rad_s);
    } else if (!output->preloaded[side]) {
        /* 先预装当前位置，再发送使能帧。 */
        output->preloaded[side] = true;
        Dm_PositionFrame(frame, motor, dm[side].position, motor->velocity_limit_rad_s);
    } else {
        Dm_SpecialFrame(frame, motor, true);
    }
    return count;
}
