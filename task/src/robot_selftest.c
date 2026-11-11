/* Host regression tests; deliberately excluded from MCU target. */
#include "robot_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        ++checks;                                                                                            \
        if (!(x)) {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                             \
            exit(1);                                                                                         \
        }                                                                                                    \
    } while (0)
#define NEAR(a, b, e) CHECK(fabsf((a) - (b)) < (e))
static void pack_rc(uint8_t b[18], unsigned a, unsigned v, unsigned t, unsigned f, unsigned l, unsigned r) {
    memset(b, 0, 18);
    b[0] = (uint8_t)a;
    b[1] = (uint8_t)((a >> 8) | (v << 3));
    b[2] = (uint8_t)((v >> 5) | (t << 6));
    b[3] = (uint8_t)(t >> 2);
    b[4] = (uint8_t)((t >> 10) | (f << 1));
    b[5] = (uint8_t)((f >> 7) | (r << 4) | (l << 6));
}
static void test_protocols(void) {
    uint8_t b[18];
    RcState r = {0};
    pack_rc(b, 1684, 364, 1024, 1100, RC_UP, RC_DOWN);
    CHECK(Rc_Decode(&r, b, 18, 7));
    CHECK(r.channel[0] == 660 && r.channel[1] == -660 && r.channel[2] == 0 && r.channel[3] == 76);
    CHECK(r.sw[0] == RC_UP && r.sw[1] == RC_DOWN);
    NEAR(Rc_Axis(&r, 0), 1.0f, 0.001f);
    CHECK(!Rc_IsNeutral(&r));
    b[5] &= 0x3f;
    CHECK(!Rc_Decode(&r, b, 18, 14));
    CHECK(!r.valid && r.stamp_ms == 7 && r.bad_frames == 1);
    CHECK(!Rc_Decode(&r, b, 17, 21));
    pack_rc(b, 1024, 1024, 1024, 1024, RC_MID, RC_MID);
    CHECK(Rc_Decode(&r, b, 18, 28));
    CHECK(Rc_IsNeutral(&r));
    pack_rc(b, 0, 1024, 1024, 1024, RC_MID, RC_MID);
    CHECK(!Rc_Decode(&r, b, 18, 35));
    DjiFeedback d = {0};
    uint8_t x[8] = {0x1f, 0xfa, 0xff, 0x9c, 0xff, 0xff, 30, 0};
    CHECK(Dji_Decode(&d, x, 8, 1));
    CHECK(d.rpm == -100 && d.current == -1 && d.total_counts == 0);
    x[0] = 0;
    x[1] = 4;
    CHECK(Dji_Decode(&d, x, 8, 2));
    CHECK(d.total_counts == 10);
    x[0] = 0x1f;
    x[1] = 0xfa;
    CHECK(Dji_Decode(&d, x, 8, 3));
    CHECK(d.total_counts == 0);
    CHECK(Dji_Decode(&d, x, 8, 100));
    CHECK(d.position_lost);
    x[0] = 0x20;
    CHECK(!Dji_Decode(&d, x, 8, 101));
    MotorFrame f;
    int16_t v[4] = {1, -1, 16384, -30000};
    Dji_GroupFrame(&f, 2, 0x1ff, v);
    CHECK(f.bus == 2 && f.id == 0x1ff && f.dlc == 8 && f.data[0] == 0 && f.data[1] == 1 && f.data[2] == 255 &&
          f.data[3] == 255);
    Dm_PositionFrame(&f, &g_dm_config[DM_BIG_PITCH], 1.0f, 0.2f);
    const uint8_t pv[8] = {0, 0, 0x80, 0x3f, 0xcd, 0xcc, 0x4c, 0x3e};
    CHECK(!memcmp(f.data, pv, 8) && f.id == 0x103);
    DmConfig lift = g_dm_config[DM_LIFT_L];
    lift.mit_kp = 6.0f;
    lift.mit_kd = 0.4f;
    Dm_PositionFrame(&f, &lift, 0.0f, 0.0f);
    const uint8_t mit[8] = {0x80, 0, 0x80, 0, 0x31, 0x14, 0x88, 0};
    CHECK(f.bus == 2 && f.dlc == 8 && f.id == 0x02 && !memcmp(f.data, mit, 8));
    Dm_SpecialFrame(&f, &g_dm_config[DM_LIFT_L], true);
    CHECK(f.id == 0x02 && f.data[7] == 0xfc && f.data[0] == 0xff);
    Dm_SpecialFrame(&f, &g_dm_config[DM_LIFT_R], false);
    CHECK(f.id == 0x01 && f.data[7] == 0xfd);
    DmFeedback m = {0};
    const uint8_t feedback[8] = {0x12, 0x80, 0, 0x80, 0, 0, 30, 31};
    CHECK(Dm_Decode(&m, &g_dm_config[0], feedback, 8, 5));
    CHECK(m.state == 1 && m.rotor_temperature == 31);
    NEAR(m.position, 0, 0.001f);
    CHECK(!Dm_Decode(&m, &g_dm_config[1], feedback, 8, 5));
    Pid pid = {0};
    const PidConfig cfg = {5, 1, 10, 20};
    for (unsigned i = 0; i < 1000; i++)
        CHECK(Pid_Update(&pid, &cfg, 100, 0.001f) <= 20);
    NEAR(pid.integral, 0, 0.001f);
    CHECK(Pid_Update(&pid, &cfg, NAN, 0.001f) == 0);
    unsigned rng = 1;
    for (unsigned k = 0; k < 10000; k++) {
        for (unsigned j = 0; j < 18; j++) {
            rng = rng * 1664525u + 1013904223u;
            b[j] = (uint8_t)(rng >> 24);
        }
        if (Rc_Decode(&r, b, 18, k))
            for (unsigned j = 0; j < 4; j++)
                CHECK(abs(r.channel[j]) <= 660);
    }
    puts("PASS DBUS parsing, rejection, wraparound, CAN encodings, PID saturation, fuzz");
}
static RobotCore c;
static RobotInput in;
static RobotOutput out;
static uint32_t now;
static void fresh(void) {
    in.rc.stamp_ms = now;
    ++in.rc.sequence;
    in.power.stamp_ms = now;
    ++in.power.sequence;
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        in.dji[i].stamp_ms = now;
        ++in.dji[i].sequence;
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        in.dm[i].stamp_ms = now;
        ++in.dm[i].sequence;
    }
}
static void step(void) {
    fresh();
    RobotCore_Step(&c, &in, now, &out);
    ++now;
}
static void run(unsigned n) {
    while (n--)
        step();
}
static void fixture(uint8_t profile) {
    memset(&in, 0, sizeof(in));
    RobotCore_Init(&c);
    now = 0;
    in.profile = profile;
    in.config_confirmed = true;
    in.power.seen = true;
    in.power.valid = true;
    in.power.chassis_power_w = 5;
    in.power.buffer_j = 60;
    in.rc.valid = true;
    in.rc.sw[0] = RC_MID;
    in.rc.sw[1] = RC_MID;
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        in.dji[i].seen = true;
        in.dji[i].temperature = 25;
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        in.dm[i].seen = true;
        in.dm[i].state = 1;
        in.dm[i].mos_temperature = 25;
        in.dm[i].rotor_temperature = 25;
    }
}
static void arm(uint8_t profile) {
    fixture(profile);
    run(501);
    CHECK(c.armed);
}
static void safe_frames(void) {
    for (unsigned i = 0; i < DJI_COUNT; i++)
        CHECK(c.dji_command[i] == 0);
    for (unsigned k = 0; k < out.count; k++) {
        const MotorFrame *f = &out.frames[k];
        if (f->id == 0x200 || f->id == 0x1ff)
            for (unsigned j = 0; j < 8; j++)
                CHECK(f->data[j] == 0);
        else
            CHECK(f->data[7] == 0xfd);
    }
}
static void test_profiles_and_rates(void) {
    fixture(ROBOT_PROFILE_RC_ONLY);
    for (unsigned i = 0; i < DJI_COUNT; i++)
        in.dji[i].seen = false;
    for (unsigned i = 0; i < DM_COUNT; i++)
        in.dm[i].seen = false;
    in.config_confirmed = false;
    run(501);
    CHECK(c.armed);
    in.rc.channel[0] = 660;
    in.rc.channel[3] = 660;
    step();
    safe_frames();
    fixture(ROBOT_PROFILE_RC_ONLY);
    unsigned n[6] = {0};
    for (unsigned t = 0; t < 1000; t++) {
        step();
        for (unsigned i = 0; i < out.count; i++) {
            const MotorFrame *f = &out.frames[i];
            if (f->bus == 1)
                n[0]++;
            else if (f->id == 0x200)
                n[1]++;
            else if (f->id == 0x1ff)
                n[2]++;
            else {
                unsigned dm = 0;
                while (dm < DM_COUNT && f->id != Dm_CommandId(&g_dm_config[dm]))
                    ++dm;
                CHECK(dm < DM_COUNT);
                n[3 + dm]++;
            }
        }
    }
    CHECK(n[0] == 500 && n[1] == 500 && n[2] == 500 && n[3] == 200 && n[4] == 200 && n[5] == 200);
    fixture(ROBOT_PROFILE_CHASSIS);
    for (unsigned i = 4; i < DJI_COUNT; i++)
        in.dji[i].seen = false;
    in.dm[2].seen = false;
    run(501);
    CHECK(c.armed);
    in.rc.channel[3] = 660;
    run(500);
    for (unsigned i = DJI_LF; i <= DJI_RR; i++)
        CHECK(c.wheel_reference[i] > 0 && c.dji_command[i] * g_dji_config[i].direction > 0);
    for (unsigned i = 4; i < DJI_COUNT; i++)
        CHECK(c.dji_command[i] == 0);
    fixture(ROBOT_PROFILE_ARM);
    for (unsigned i = 0; i < 4; i++)
        in.dji[i].seen = false;
    in.dm[0].seen = false;
    in.dm[1].seen = false;
    run(501);
    CHECK(c.armed);
    in.rc.channel[3] = 660;
    step();
    for (unsigned i = 0; i < 4; i++)
        CHECK(c.dji_command[i] == 0);
    fixture(ROBOT_PROFILE_FULL);
    in.dji[3].seen = false;
    run(600);
    CHECK(!c.armed && c.stop_reason & ROBOT_STOP_FEEDBACK);
    puts("PASS four profiles, absent inactive hardware, 500Hz DJI / 200Hz per DM");
}
static void test_motion(void) {
    arm(ROBOT_PROFILE_FULL);
    in.rc.sw[1] = RC_UP;
    in.rc.channel[0] = 660;
    in.rc.channel[1] = 660;
    run(1000);
    NEAR(c.joint_target[JOINT_YAW], 30, 0.05f);
    NEAR(c.joint_target[JOINT_BIG_PITCH], 12, 0.05f);
    CHECK(c.joint_target[JOINT_SMALL_PITCH] == 0 && c.joint_target[JOINT_GRIP] == 0);
    float yaw = c.joint_target[0], pitch = c.joint_target[5];
    in.rc.sw[1] = RC_MID;
    run(100);
    NEAR(c.joint_target[0], yaw, 0.001f);
    NEAR(c.joint_target[5], pitch, 0.001f);
    in.rc.sw[1] = RC_DOWN;
    run(500);
    CHECK(c.joint_target[1] > 0 && c.joint_target[2] > 0);
    NEAR(c.joint_target[0], yaw, 0.001f);
    in.rc.channel[0] = 0;
    in.rc.channel[1] = 0;
    float small = c.joint_target[1], grip = c.joint_target[2];
    run(100);
    NEAR(c.joint_target[1], small, 0.001f);
    NEAR(c.joint_target[2], grip, 0.001f);
    in.rc.sw[0] = RC_UP;
    run(1000);
    NEAR(c.joint_target[3], c.joint_target[4], 0.001f);
    CHECK(c.dm_target[0] > 0 && c.dm_target[1] < 0);
    float lift = c.joint_target[3];
    in.rc.sw[0] = RC_MID;
    run(100);
    NEAR(c.joint_target[3], lift, 0.001f);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_DOWN;
    run(1200);
    NEAR(c.joint_target[JOINT_LIFT_L], 0.0f, 0.001f);
    NEAR(c.joint_target[JOINT_LIFT_R], 0.0f, 0.001f);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_MID;
    in.rc.sw[1] = RC_UP;
    in.rc.channel[0] = 660;
    run(7000);
    NEAR(c.joint_target[0], 180, 0.001f);
    in.rc.channel[0] = -660;
    run(13000);
    NEAR(c.joint_target[0], -180, 0.001f);
    arm(ROBOT_PROFILE_CHASSIS);
    in.rc.channel[2] = 660;
    run(100);
    CHECK(c.wheel_reference[DJI_LF] > 0 && c.wheel_reference[DJI_RF] < 0);
    CHECK(c.wheel_reference[DJI_LR] == c.wheel_reference[DJI_LF]);
    CHECK(c.wheel_reference[DJI_RR] == c.wheel_reference[DJI_RF]);
    puts("PASS differential steering, arm modes, neutral hold, mirrored synchronous lifts, yaw +/-180");
}
static void test_single_lift_travel(void) {
    if (ROBOT_CHASSIS_LIFT_TEST == ROBOT_LIFT_TEST_BOTH)
        return;
    unsigned active = ROBOT_CHASSIS_LIFT_TEST == ROBOT_LIFT_TEST_LEFT ? DM_LIFT_L : DM_LIFT_R;
    unsigned inactive = active == DM_LIFT_L ? DM_LIFT_R : DM_LIFT_L;
    unsigned axis = JOINT_LIFT_L + active;
    uint16_t inactive_id = Dm_CommandId(&g_dm_config[inactive]);
    float expected_speed = active == DM_LIFT_R ? 500.0f : 50.0f;
    fixture(ROBOT_PROFILE_CHASSIS);
    in.dm[inactive].seen = false;
    run(501);
    CHECK(c.armed);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_UP;
    step();
    NEAR(c.lift_test_speed_deg_s[active], expected_speed, 0.02f);
    run(1500);
    NEAR(c.joint_target[axis], ROBOT_LIFT_TEST_MAX_LEAD_DEG, 0.01f);
    CHECK(c.dm_target[active] * (active == DM_LIFT_L ? 1.0f : -1.0f) > 0.0f);

    for (unsigned i = 0; i < 75000; i++) {
        in.dm[active].position = c.dm_target[active];
        step();
        if (i == 500)
            NEAR(c.lift_test_speed_deg_s[active], expected_speed, 0.02f);
    }
    CHECK(c.armed);
    const DmConfig *motor = &g_dm_config[active];
    float end_deg = fminf(3600.0f, (motor->p_max * (1.0f - 2.0f / 65535.0f)) *
                                      ROBOT_RAD2DEG / motor->external_reduction);
    NEAR(c.joint_target[axis], end_deg, 0.01f);
    NEAR(c.joint_target[JOINT_LIFT_L + inactive], 0.0f, 0.001f);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_MID;
    for (unsigned i = 0; i < ROBOT_DM_PERIOD_MS; i++) {
        step();
        for (unsigned j = 0; j < out.count; j++)
            if (out.frames[j].bus == 2 && out.frames[j].id == inactive_id)
                CHECK(out.frames[j].data[0] == 0xff && out.frames[j].data[7] == 0xfd);
    }
    CHECK(c.lift_test_speed_deg_s[active] == 0.0f);
    NEAR(c.joint_target[axis], end_deg, 0.01f);
    puts("PASS single lift isolation, direction, target speed, tracking lead, travel limit and neutral hold");
}
static void test_paired_lift_catch_up(void) {
    if (ROBOT_CHASSIS_LIFT_TEST != ROBOT_LIFT_TEST_BOTH)
        return;
    fixture(ROBOT_PROFILE_CHASSIS);
    in.dm[DM_LIFT_L].state = 0;
    in.dm[DM_LIFT_R].state = 0;
    run(501);
    CHECK(c.armed);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_UP;
    in.dm[DM_LIFT_L].state = 1;
    in.dm[DM_LIFT_L].position = 0.2f * ROBOT_DEG2RAD;
    run(20);
    NEAR(c.joint_target[JOINT_LIFT_L], 2.0f, 0.001f);
    CHECK(c.lift_test_speed_deg_s[DM_LIFT_L] == 0.0f);
    in.dm[DM_LIFT_R].state = 1;
    in.dm[DM_LIFT_R].position = -2.0f * ROBOT_DEG2RAD;
    step();
    CHECK(c.armed);
    NEAR(c.joint_target[JOINT_LIFT_R], 2.0f, 0.001f);
    NEAR(c.joint_target[JOINT_LIFT_L], 2.0f, 0.001f);

    /* 两侧 CAN 增量相差十倍且方向相反，机构位移相同。 */
    in.dm[DM_LIFT_L].position = 6.0f * ROBOT_DEG2RAD;
    in.dm[DM_LIFT_R].position = -60.0f * ROBOT_DEG2RAD;
    step();
    CHECK(c.armed && c.lift_wait_mask == 0u);
    NEAR(c.lift_skew_deg, 0.0f, 0.001f);

    arm(ROBOT_PROFILE_CHASSIS);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_UP;
    step();
    NEAR(c.lift_test_speed_deg_s[DM_LIFT_L], 500.0f, 0.02f);
    NEAR(c.lift_test_speed_deg_s[DM_LIFT_R], 500.0f, 0.02f);
    NEAR(c.dm_target[DM_LIFT_L] / ROBOT_CONTROL_DT, 50.0f * ROBOT_DEG2RAD, 0.001f);
    NEAR(c.dm_target[DM_LIFT_R] / ROBOT_CONTROL_DT, -500.0f * ROBOT_DEG2RAD, 0.001f);
    run(5);
    CHECK(c.joint_target[JOINT_LIFT_L] > 0.0f);
    NEAR(c.joint_target[JOINT_LIFT_L], c.joint_target[JOINT_LIFT_R], 0.001f);

    in.dm[DM_LIFT_L].position = 0.525f * ROBOT_DEG2RAD;
    step();
    CHECK(c.armed && c.stop_reason == 0u && c.lift_wait_mask == ROBOT_LIFT_TEST_LEFT);
    NEAR(c.lift_skew_deg, 5.25f, 0.001f);
    float held = c.joint_target[JOINT_LIFT_L];
    float follower = c.joint_target[JOINT_LIFT_R];
    NEAR(held, 5.25f, 0.001f);
    run(1000);
    NEAR(c.joint_target[JOINT_LIFT_L], held, 0.001f);
    CHECK(c.joint_target[JOINT_LIFT_R] > follower);
    NEAR(c.joint_target[JOINT_LIFT_R], held, 0.001f);
    in.dm[DM_LIFT_R].position = -1.25f * ROBOT_DEG2RAD;
    step();
    CHECK(c.lift_wait_mask == ROBOT_LIFT_TEST_LEFT);
    in.dm[DM_LIFT_R].position = -2.5f * ROBOT_DEG2RAD;
    run(50);
    CHECK(c.armed && c.lift_wait_mask == 0u && c.joint_target[JOINT_LIFT_L] > held);

    in.dm[DM_LIFT_L].position = 1.2f * ROBOT_DEG2RAD;
    in.dm[DM_LIFT_R].position = -6.0f * ROBOT_DEG2RAD;
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_DOWN;
    step();
    CHECK(c.armed && c.lift_wait_mask == ROBOT_LIFT_TEST_RIGHT);
    NEAR(c.joint_target[JOINT_LIFT_R], 6.0f, 0.001f);
    follower = c.joint_target[JOINT_LIFT_L];
    run(500);
    CHECK(c.joint_target[JOINT_LIFT_L] <= follower && c.joint_target[JOINT_LIFT_L] >= 5.999f);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_MID;
    follower = c.joint_target[JOINT_LIFT_L];
    run(50);
    CHECK(c.lift_wait_mask == 0u);
    NEAR(c.joint_target[JOINT_LIFT_L], follower, 0.001f);
    NEAR(c.joint_target[JOINT_LIFT_R], 6.0f, 0.001f);

    fixture(ROBOT_PROFILE_CHASSIS);
    in.dm[DM_LIFT_L].position = -1.25f;
    in.dm[DM_LIFT_R].position = 1.75f;
    run(501);
    CHECK(c.armed);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_UP;
    for (unsigned i = 0; i < 75000; i++) {
        in.dm[DM_LIFT_L].position = c.dm_target[DM_LIFT_L];
        in.dm[DM_LIFT_R].position = c.dm_target[DM_LIFT_R];
        step();
    }
    CHECK(c.armed && c.lift_wait_mask == 0u);
    float right_span_rad = 1.75f + g_dm_config[DM_LIFT_R].p_max * (1.0f - 2.0f / 65535.0f);
    float end_deg = fminf(3600.0f, right_span_rad * ROBOT_RAD2DEG);
    NEAR(c.joint_target[JOINT_LIFT_L], end_deg, 0.01f);
    NEAR(c.joint_target[JOINT_LIFT_R], end_deg, 0.01f);
    NEAR(c.dm_target[DM_LIFT_L] - c.dm_boot_pos[DM_LIFT_L], end_deg * 0.1f * ROBOT_DEG2RAD, 0.001f);
    NEAR(c.dm_target[DM_LIFT_R] - c.dm_boot_pos[DM_LIFT_R], -end_deg * ROBOT_DEG2RAD, 0.001f);
    in.rc.sw[ROBOT_RC_LEFT_SWITCH] = RC_DOWN;
    for (unsigned i = 0; i < 75000; i++) {
        in.dm[DM_LIFT_L].position = c.dm_target[DM_LIFT_L];
        in.dm[DM_LIFT_R].position = c.dm_target[DM_LIFT_R];
        step();
    }
    CHECK(c.armed && c.lift_wait_mask == 0u);
    NEAR(c.joint_target[JOINT_LIFT_L], 0.0f, 0.01f);
    NEAR(c.joint_target[JOINT_LIFT_R], 0.0f, 0.01f);
    NEAR(c.dm_target[DM_LIFT_L], -1.25f, 0.001f);
    NEAR(c.dm_target[DM_LIFT_R], 1.75f, 0.001f);
    puts("PASS paired lift enable, catch-up, hysteresis, asymmetric gearing and travel");
}
static void test_safety(void) {
    arm(ROBOT_PROFILE_FULL);
    in.rc.channel[3] = 660;
    run(100);
    in.emergency_stop = true;
    step();
    CHECK(!c.armed && out.abort_pending && (c.stop_reason & ROBOT_STOP_ESTOP));
    safe_frames();
    in.emergency_stop = false;
    run(600);
    CHECK(!c.armed);
    in.rc.channel[3] = 0;
    run(501);
    CHECK(c.armed);
    /* RC timeout with motor feedback remaining live; no large artificial task jump. */
    uint32_t old = in.rc.stamp_ms;
    for (unsigned i = 0; i < ROBOT_RC_TIMEOUT_MS + 1; i++) {
        fresh();
        in.rc.stamp_ms = old;
        RobotCore_Step(&c, &in, now++, &out);
    }
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_RC));
    safe_frames();
    arm(ROBOT_PROFILE_FULL);
    ++in.rc.bad_frames;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_RC));
    arm(ROBOT_PROFILE_FULL);
    in.dji[4].position_lost = true;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_POSITION_LOST));
    arm(ROBOT_PROFILE_ARM);
    in.bus_fault_mask = 1;
    step();
    CHECK(c.armed);
    arm(ROBOT_PROFILE_FULL);
    in.hardware_limit_mask = 1;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_LIMIT));
    arm(ROBOT_PROFILE_FULL);
    in.dm[2].state = 0;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_DM_ERROR));
    arm(ROBOT_PROFILE_FULL);
    in.dji[0].temperature = 80;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_TEMPERATURE));
    arm(ROBOT_PROFILE_FULL);
    in.dm[2].state = 8;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_DM_ERROR));
    arm(ROBOT_PROFILE_FULL);
    in.bus_fault_mask = 2;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_BUS));
    arm(ROBOT_PROFILE_FULL);
    in.dm[DM_LIFT_L].position = 6.0f * ROBOT_DEG2RAD;
    in.dm[DM_LIFT_R].position = -60.0f * ROBOT_DEG2RAD;
    step();
    CHECK(c.armed && c.stop_reason == 0u);
    NEAR(c.lift_skew_deg, 0.0f, 0.001f);
    in.dm[DM_LIFT_L].position = 6.6f * ROBOT_DEG2RAD;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_LIFT_SKEW));
    arm(ROBOT_PROFILE_CHASSIS);
    in.dm[DM_LIFT_L].position = 0.6f * ROBOT_DEG2RAD;
    step();
    CHECK(c.armed && c.stop_reason == 0u);
    arm(ROBOT_PROFILE_FULL);
    in.dji[4].total_counts = 4300;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_LIMIT));
    arm(ROBOT_PROFILE_FULL);
    in.dm[2].position = NAN;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_LIMIT));
    arm(ROBOT_PROFILE_FULL);
    in.deadline_fault = true;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_TIMING));
    arm(ROBOT_PROFILE_FULL);
    now += 10;
    step();
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_TIMING));
    fixture(ROBOT_PROFILE_FULL);
    in.config_confirmed = false;
    run(600);
    CHECK(!c.armed && (c.stop_reason & ROBOT_STOP_CONFIG));
    arm(ROBOT_PROFILE_FULL);
    in.profile = ROBOT_PROFILE_RC_ONLY;
    step();
    CHECK(!c.armed && out.abort_pending);
    safe_frames();
    /* Unsigned timeout arithmetic works across the 32-bit ms rollover. */
    fixture(ROBOT_PROFILE_RC_ONLY);
    now = UINT32_MAX - 250u;
    run(501);
    CHECK(c.armed);
    puts("PASS radio loss, invalid frame latch, rearm, emergency, thermal, bus, limits, lost angle, skew, "
         "deadline, rollover");
}
static void test_grip_and_dm_enable(void) {
    arm(ROBOT_PROFILE_ARM);
    in.rc.sw[1] = RC_DOWN;
    in.rc.channel[0] = 660;
    in.dji[DJI_GRIP].current = 1500;
    run(200);
    CHECK(c.grip_hold);
    CHECK(c.dji_command[DJI_GRIP] == 700);
    in.rc.sw[1] = RC_MID;
    in.rc.channel[0] = 0;
    run(100);
    CHECK(c.grip_hold && c.dji_command[DJI_GRIP] == 700);
    in.rc.sw[1] = RC_DOWN;
    in.rc.channel[0] = -660;
    step();
    CHECK(!c.grip_hold);
    fixture(ROBOT_PROFILE_ARM);
    in.dm[2].state = 0;
    run(501);
    CHECK(c.armed);
    bool enabled = false, preloaded = false;
    for (unsigned i = 0; i < 10; i++) {
        step();
        for (unsigned j = 0; j < out.count; j++)
            if (out.frames[j].id == 0x103) {
                if (out.frames[j].data[7] == 0xfc) {
                    CHECK(preloaded);
                    enabled = true;
                } else
                    preloaded = true;
            }
    }
    CHECK(enabled);
    run(220);
    CHECK(!c.armed); /* repeated enable has a finite timeout */
    puts("PASS gripper contact / hold / release, DM enable handshake and timeout");
}

static PowerConfig power_cfg(void) {
    PowerConfig cfg = g_power_config;
    cfg.require_referee = true;
    return cfg;
}
static void test_power_protocol_and_math(void) {
    PowerTelemetry t = {0};
    const uint8_t be[4] = {0x02, 0x8a, 0x02, 0x58}, le[4] = {0x8a, 0x02, 0x58, 0x02};
    CHECK(PowerTelemetry_Decode(&t, be, 4, POWER_ORDER_BIG_ENDIAN, 100));
    NEAR(t.chassis_power_w, 65.0f, 0.001f);
    NEAR(t.buffer_j, 60.0f, 0.001f);
    CHECK(PowerTelemetry_Decode(&t, le, 4, POWER_ORDER_LITTLE_ENDIAN, 101));
    const uint8_t negative[4] = {0xff, 0x9c, 0xff, 0xce};
    CHECK(PowerTelemetry_Decode(&t, negative, 4, POWER_ORDER_BIG_ENDIAN, 102));
    NEAR(t.chassis_power_w, -10.0f, 0.001f);
    NEAR(t.buffer_j, -5.0f, 0.001f);
    CHECK(!PowerTelemetry_Decode(&t, be, 3, POWER_ORDER_BIG_ENDIAN, 103));
    CHECK(!t.valid && t.stamp_ms == 102);
    CHECK(!PowerTelemetry_Decode(&t, be, 4, POWER_ORDER_UNCONFIRMED, 104));
    const uint8_t bad_energy[4] = {0, 0, 0x02, 0x59};
    CHECK(!PowerTelemetry_Decode(&t, bad_energy, 4, POWER_ORDER_BIG_ENDIAN, 105));
    PowerConfig cfg = power_cfg();
    float amp[4] = {8, 6, 4, 2}, rpm[4] = {3000, 2500, 2000, 1500};
    PowerSolution solution = Power_Solve(&cfg, amp, rpm, 45.0f);
    CHECK(solution.valid && solution.feasible);
    NEAR(solution.a_w, 12.0f, 0.0001f);
    NEAR(solution.b_w, 60.0f, 0.0001f);
    NEAR(solution.predicted_before_w, 77.0f, 0.0001f);
    NEAR(solution.k, 0.5956959f, 0.00001f);
    NEAR(solution.predicted_after_w, 45.0f, 0.0001f);
    solution = Power_Solve(&cfg, amp, rpm, 80);
    NEAR(solution.k, 1.0f, 0.00001f);
    float zero[4] = {0};
    solution = Power_Solve(&cfg, zero, rpm, 60);
    NEAR(solution.k, 1.0f, 0.00001f);
    cfg.resistance_ohm = 0;
    solution = Power_Solve(&cfg, amp, rpm, 45);
    NEAR(solution.k, 2.0f / 3.0f, 0.00001f);
    cfg = power_cfg();
    solution = Power_Solve(&cfg, amp, rpm, 4);
    CHECK(solution.valid && !solution.feasible && solution.k == 0);
    cfg.resistance_ohm = -0.1f;
    solution = Power_Solve(&cfg, amp, rpm, 60);
    CHECK(!solution.valid);
    cfg = power_cfg();
    amp[0] = NAN;
    solution = Power_Solve(&cfg, amp, rpm, 60);
    CHECK(!solution.valid);
    amp[0] = 8;
    /* Large positive B with tiny A exercises the stable positive-root expression. */
    cfg.resistance_ohm = 1e-10f;
    solution = Power_Solve(&cfg, amp, rpm, 45);
    NEAR(solution.k, 2.0f / 3.0f, 0.00001f);
    cfg = power_cfg();
    for (unsigned i = 0; i < 4; i++)
        rpm[i] = -100.0f;
    solution = Power_Solve(&cfg, amp, rpm, 6);
    CHECK(solution.valid && solution.feasible && solution.k >= 0 && solution.k <= 1);
    NEAR(solution.predicted_after_w, 6, 0.0001f);
    NEAR(Power_BufferStep(60, 100, 65, 0.001f), 59.965f, 0.0001f);
    NEAR(Power_BufferStep(59, 0, 65, 1), 60, 0.0001f);
    NEAR(Power_BufferStep(-9, 500, 65, 1), -10, 0.0001f);
    puts("PASS supplied 77W->45W example, big/little-endian signed telemetry, degenerate/stable roots, rule "
         "buffer arithmetic");
}
static void telemetry(PowerTelemetry *t, uint32_t stamp, float watts, float energy) {
    t->seen = true;
    t->valid = true;
    t->chassis_power_w = watts;
    t->buffer_j = energy;
    t->stamp_ms = stamp;
    ++t->sequence;
}
static void test_power_limiter(void) {
    PowerConfig cfg = power_cfg();
    PowerLimiter limiter;
    PowerLimiter_Init(&limiter);
    PowerTelemetry t = {0};
    const int16_t request[4] = {16000, -16000, -16000, 16000}, rpm[4] = {4000, -4000, -4000, 4000},
                  measured[4] = {0};
    int16_t output[4];
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 0, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_REFEREE_MISSING);
    for (unsigned i = 0; i < 4; i++)
        CHECK(output[i] == 0);
    telemetry(&t, 1, 5, 60);
    limiter.k_applied = 1;
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 1, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_SCALED);
    CHECK(limiter.k_applied > 0 && limiter.k_applied < 1);
    CHECK(limiter.predicted_after_w <= 60.0f);
    CHECK(output[0] == -output[1] && output[1] == output[2] && output[0] == output[3]);
    telemetry(&t, 2, 5, 10);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 2, 0.001f, output);
    CHECK((limiter.status & POWER_STATUS_LOW_BUFFER) && limiter.budget_w == 50.0f &&
          limiter.predicted_after_w <= 50.0f);
    /* No duplicate update of the same referee measurement, despite 1kHz control. */
    telemetry(&t, 3, 100, 60);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 3, 0.001f, output);
    float reserve = limiter.residual_reserve_w;
    CHECK(reserve > 0);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 4, 0.001f, output);
    NEAR(limiter.residual_reserve_w, reserve, 0.00001f);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 203, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_REFEREE_MISSING);
    CHECK(limiter.k_applied == 0);
    telemetry(&t, 204, 5, 60);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 204, 0.001f, output);
    CHECK(limiter.k_applied <= 0.002001f);
    telemetry(&t, 205, 5, 0);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 205, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_CUTOFF_HOLD);
    for (unsigned i = 0; i < 4; i++)
        CHECK(output[i] == 0);
    telemetry(&t, 5204, 5, 60);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 5204, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_CUTOFF_HOLD);
    telemetry(&t, 5205, 5, 60);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 5205, 0.001f, output);
    CHECK(!(limiter.status & POWER_STATUS_CUTOFF_HOLD));
    ++t.bad_frames;
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 5206, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_REFEREE_INVALID);
    cfg.require_referee = false;
    t.valid = false;
    t.seen = false;
    PowerLimiter_Init(&limiter);
    limiter.k_applied = 1;
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, 0, 0.001f, output);
    CHECK(limiter.status & POWER_STATUS_BENCH_MODEL_ONLY);
    CHECK(limiter.budget_w == 45 && limiter.predicted_after_w <= 45);
    /* Cutoff hold arithmetic survives the 32-bit tick wrap as well. */
    cfg = power_cfg();
    PowerLimiter_Init(&limiter);
    uint32_t start = UINT32_MAX - 100u;
    telemetry(&t, start, 5, 0);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, start, 0.001f, output);
    telemetry(&t, start + 5000u, 5, 60);
    PowerLimiter_Apply(&limiter, &cfg, &t, request, measured, rpm, start + 5000u, 0.001f, output);
    CHECK(!limiter.cutoff_active);
    puts("PASS model limit, wheel ratios, low-buffer headroom, measured residual, stale/invalid referee, 5s "
         "hold, recovery slope, bench mode");
}
static void test_power_randomized(void) {
    PowerConfig cfg = power_cfg();
    unsigned rng = 0x13572468u;
    unsigned quantization_cases = 0;
    for (unsigned trial = 0; trial < 10000; trial++) {
        float amp[4], rpm_f[4];
        int16_t request[4], rpm[4], measured[4] = {0}, output[4];
        for (unsigned i = 0; i < 4; i++) {
            rng = rng * 1664525u + 1013904223u;
            request[i] = (int16_t)((int)(rng % 32769u) - 16384);
            rng = rng * 1664525u + 1013904223u;
            rpm[i] = (int16_t)((int)(rng % 60001u) - 30000);
            amp[i] = request[i] * POWER_C620_A_PER_COUNT;
            rpm_f[i] = (float)rpm[i];
        }
        float budget = 5.0f + (float)(trial % 600u) * 0.1f;
        PowerSolution sol = Power_Solve(&cfg, amp, rpm_f, budget);
        CHECK(sol.valid && sol.feasible && isfinite(sol.k) && sol.k >= 0 && sol.k <= 1);
        double a = 0, b = 0;
        for (unsigned i = 0; i < 4; i++) {
            a += cfg.resistance_ohm * (double)amp[i] * amp[i];
            b += cfg.mechanical_w_per_a_rpm * (double)amp[i] * rpm_f[i];
        }
        double prediction = a * sol.k * sol.k + b * sol.k + cfg.baseline_w;
        CHECK(prediction <= budget + 0.002);
        if (sol.k < 0.999f) {
            double higher = sol.k + 0.0001;
            CHECK(a * higher * higher + b * higher + cfg.baseline_w > budget - 0.0001);
        }
        PowerLimiter lim;
        PowerLimiter_Init(&lim);
        lim.k_applied = 1;
        PowerTelemetry t = {0};
        telemetry(&t, trial, 5, 60);
        PowerLimiter_Apply(&lim, &cfg, &t, request, measured, rpm, trial, 0.001f, output);
        CHECK(Power_PredictCounts(&cfg, output, rpm) <= lim.budget_w + 0.00001f);
        if (lim.status & POWER_STATUS_QUANTIZED)
            ++quantization_cases;
        for (unsigned i = 0; i < 4; i++) {
            CHECK(abs(output[i]) <= abs(request[i]));
            CHECK((int32_t)output[i] * request[i] >= 0);
            CHECK(fabsf(output[i] - lim.k_applied * request[i]) < 1.001f);
        }
    }
    CHECK(quantization_cases > 0);
    printf("PASS 10000 randomized power cases with double-precision oracle and integer CAN back-substitution "
           "(%u rounding repairs)\n",
           quantization_cases);
}
static void test_power_core_isolation(void) {
    arm(ROBOT_PROFILE_FULL);
    in.rc.channel[3] = 660;
    in.rc.sw[0] = RC_UP;
    in.rc.sw[1] = RC_UP;
    in.rc.channel[0] = 660;
    in.rc.channel[1] = 660;
    run(50);
    RobotCore normal = c, limited = c;
    RobotInput reference = in;
    for (unsigned tick = 0; tick < 20; tick++) {
        ++now;
        in = reference;
        fresh();
        RobotOutput normal_out, limited_out;
        RobotCore_Step(&normal, &in, now, &normal_out);
        in.power.buffer_j = 0;
        RobotCore_Step(&limited, &in, now, &limited_out);
        CHECK(normal.armed && limited.armed);
        if (tick == 0)
            CHECK(limited_out.abort_wheel_pending && !limited_out.abort_pending);
        for (unsigned i = 0; i < 4; i++)
            CHECK(limited.dji_command[i] == 0);
        for (unsigned i = DJI_YAW; i < DJI_COUNT; i++)
            CHECK(limited.dji_command[i] == normal.dji_command[i]);
        for (unsigned i = 0; i < DM_COUNT; i++)
            NEAR(limited.dm_target[i], normal.dm_target[i], 0.00001f);
        for (unsigned i = 0; i < 4; i++)
            NEAR(limited.speed_pid[i].integral, 0, 0.00001f);
    }
    /* The configured referee policy affects only wheel output. */
    c = normal;
    in = reference;
    in.power = (PowerTelemetry){0};
    step();
    CHECK(c.armed);
    if (g_power_config.require_referee) {
        CHECK(c.power.status & POWER_STATUS_REFEREE_MISSING);
        for (unsigned i = 0; i < 4; i++)
            CHECK(c.dji_command[i] == 0);
    } else {
        CHECK(c.power.status & POWER_STATUS_BENCH_MODEL_ONLY);
        CHECK(c.power.budget_w <= g_power_config.bench_limit_w);
        CHECK(c.power.predicted_after_w <= c.power.budget_w);
        for (unsigned i = 0; i < 4; i++)
            CHECK(c.dji_command[i] * g_dji_config[i].direction > 0);
    }
    CHECK(c.joint_target[JOINT_LIFT_L] > 0 && c.joint_target[JOINT_BIG_PITCH] > 0);
    puts("PASS wheel-only integration: lift/arm commands unchanged, constrained wheel PI does not wind up, "
         "referee failure does not globally disarm");
}

int main(void) {
    test_protocols();
    test_profiles_and_rates();
    test_motion();
    test_single_lift_travel();
    test_paired_lift_catch_up();
    test_safety();
    test_grip_and_dm_enable();
    test_power_protocol_and_math();
    test_power_limiter();
    test_power_randomized();
    test_power_core_isolation();
    printf("ALL TESTS PASSED (%u checks)\n", checks);
    return 0;
}
