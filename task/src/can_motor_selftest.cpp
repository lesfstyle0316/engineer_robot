/* Host-only tests. Synthetic frames below exercise code, not on-board CAN. */
#include "DJI_motor_driver.h"
#include "DM_motor_driver.h"
#include "motor_system.h"
#include "robot_core.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <memory>
#include <vector>
static unsigned checks;
#define CHECK(x)                                                                                             \
    do {                                                                                                     \
        ++checks;                                                                                            \
        if (!(x)) {                                                                                          \
            std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x);                                        \
            std::exit(1);                                                                                    \
        }                                                                                                    \
    } while (0)
#define NEAR(a, b, e) CHECK(std::fabs((a) - (b)) < (e))
static MotorFrame frame(uint8_t bus, uint16_t id, uint8_t len = 8) {
    MotorFrame f{};
    f.bus = bus;
    f.id = id;
    f.dlc = len;
    return f;
}
struct received {
    unsigned calls{}, bad{};
    uint32_t stamp{};
    uint8_t value{};
};
static void receive(can_device_receive *rx, const MotorFrame &f, uint32_t now) {
    auto *r = static_cast<received *>(rx->context());
    ++r->calls;
    r->stamp = now;
    r->value = f.data[0];
}
static void bad_length(can_device_receive *rx, const MotorFrame &, uint32_t) {
    ++static_cast<received *>(rx->context())->bad;
}
static unsigned lock_depth, notifies;
static uint32_t enter() {
    return lock_depth++;
}
static void leave(uint32_t previous) {
    CHECK(lock_depth == previous + 1);
    lock_depth = previous;
}
static void wake(uint8_t bus) {
    CHECK(lock_depth == 0 && bus >= 1 && bus <= 2);
    ++notifies;
}
static void test_registry() {
    can_network net;
    net.bind_platform(enter, leave, wake);
    received a, b, p;
    can_device_receive rx1(net, 1, 0x201, receive, &a), rx2(net, 2, 0x201, receive, &b),
        power(net, 2, 0x31, receive, &p, 4, bad_length);
    CHECK(rx1.valid() && rx2.valid() && power.valid());
    CHECK(net.rx_count(1) == 1 && net.rx_count(2) == 2);
    can_device_receive duplicate(net, 1, 0x201, receive, &a);
    CHECK(!duplicate.valid());
    MotorFrame f = frame(1, 0x201);
    f.data[0] = 42;
    CHECK(net.dispatch(f, 10));
    CHECK(a.calls == 1 && b.calls == 0 && a.value == 42);
    f.bus = 2;
    CHECK(net.dispatch(f, 11));
    CHECK(b.calls == 1 && b.stamp == 11);
    f = frame(2, 0x31, 4);
    f.data[0] = 2;
    CHECK(net.dispatch(f, 12));
    CHECK(p.calls == 1 && power.rx_data[4] == 0);
    f.dlc = 8;
    CHECK(!net.dispatch(f, 13));
    CHECK(p.bad == 1 && p.calls == 1);
    f = frame(0, 0x201);
    CHECK(!net.dispatch(f, 14));
    f = frame(1, 0x777);
    CHECK(!net.dispatch(f, 15));
    {
        can_device_receive temporary(net, 1, 0x202, receive, &a);
        CHECK(temporary.valid() && net.rx_count(1) == 2);
    }
    CHECK(net.rx_count(1) == 1);
    std::vector<std::unique_ptr<can_device_receive>> devices;
    for (unsigned i = 0; i < 15; i++) {
        devices.emplace_back(new can_device_receive(net, 1, static_cast<uint16_t>(0x300 + i), receive, &a));
        CHECK(devices.back()->valid());
    }
    can_device_receive overflow(net, 1, 0x500, receive, &a);
    CHECK(!overflow.valid());
    CHECK(lock_depth == 0);
    std::puts("PASS bus/ID/DLC routing, callback contexts, rejection, registry capacity and lifetime");
}
static void test_queues() {
    can_network net;
    net.bind_platform(enter, leave, wake);
    can_device_transmit tx(net, 1, 0x200);
    uint8_t original[8] = {1, 2, 3, 4};
    CHECK(tx.set_buf_data(original, 8));
    CHECK(tx.can_send_msg(10));
    original[0] = 99;
    tx.member.data[0] = 88;
    can_device_transmit_member msg{};
    CHECK(net.pop(1, 10, msg));
    CHECK(msg.frame.data[0] == 1);
    CHECK(net.is_current(msg, 10));
    CHECK(!tx.set_id(0x800));
    CHECK(!tx.set_buf_data(nullptr, 8));
    tx.member.data[0] = 11;
    CHECK(tx.can_send_msg(20));
    tx.member.data[0] = 12;
    CHECK(tx.can_send_msg(21));
    CHECK(net.pending(1) == 1 && net.statistics(1).coalesced == 1);
    CHECK(net.pop(1, 21, msg));
    CHECK(msg.frame.data[0] == 12);
    net.cancel(1);
    CHECK(!net.is_current(msg, 21));
    CHECK(net.pending(1) == 0);
    auto second = frame(2, 0x101);
    CHECK(net.enqueue(second, 30));
    CHECK(tx.can_send_msg(30));
    net.cancel(1);
    CHECK(net.pending(1) == 0 && net.pending(2) == 1);
    CHECK(net.pop(2, 30, msg));
    CHECK(net.is_current(msg, 30));
    CHECK(tx.can_send_msg(40));
    CHECK(!net.pop(1, 46, msg));
    CHECK(net.statistics(1).expired == 1 && (net.take_faults() & 1u));
    CHECK(tx.can_send_msg(UINT32_MAX - 1));
    CHECK(net.pop(1, 1, msg));
    CHECK(net.is_current(msg, 1));
    net.cancel(1);
    for (unsigned i = 0; i < can_network::tx_capacity; i++) {
        auto f = frame(1, static_cast<uint16_t>(0x100 + i));
        f.data[0] = static_cast<uint8_t>(i);
        CHECK(net.enqueue(f, 100));
    }
    CHECK(!net.enqueue(frame(1, 0x600), 100));
    CHECK(net.take_faults() & 1u);
    for (unsigned i = 0; i < can_network::tx_capacity; i++) {
        CHECK(net.pop(1, 100, msg));
        CHECK(msg.frame.id == 0x100 + i && msg.frame.data[0] == i);
    }
    CHECK(net.pending(1) == 0 && notifies > 0 && lock_depth == 0);
    std::puts("PASS owned TX payloads, coalescing, FIFO/capacity, expiry, generation cancellation and "
              "counter wrap");
}
static void test_dji_objects() {
    can_network net;
    DJI_motor_group group(net, 1, 0x200);
    DJI_motor lf(net, group, g_dji_config[0], 1, DJI_M3508), rf(net, group, g_dji_config[1], 2, DJI_M3508);
    CHECK(lf.valid() && rf.valid() && !lf.motor_ready(0));
    CHECK(lf.motor_set_current(0.1f));
    CHECK(rf.motor_set_current(0.1f));
    CHECK(lf.command() == 1638 && rf.command() == -1638);
    CHECK(group.can_send_msg(0));
    can_device_transmit_member direct{};
    CHECK(net.pop(1, 0, direct) && direct.frame.data[0] == 6 && direct.frame.data[1] == 0x66);
    CHECK(!lf.motor_set_voltage(0.1f));
    CHECK(!lf.motor_set_raw_output(3001));
    MotorFrame command;
    const int16_t v[4] = {100, -200, 0, 0};
    Dji_GroupFrame(&command, 1, 0x200, v);
    CHECK(group.apply_frame(command, 1));
    can_device_transmit_member msg{};
    CHECK(net.pop(1, 1, msg));
    CHECK(!std::memcmp(msg.frame.data, command.data, 8));
    command.data[4] = 1;
    CHECK(!group.apply_frame(command, 2));
    CHECK(net.pending(1) == 0);
    auto feedback = frame(1, 0x201);
    const uint8_t bytes[8] = {0x1f, 0xfa, 0, 100, 0, 10, 30, 0};
    std::memcpy(feedback.data, bytes, 8);
    CHECK(net.dispatch(feedback, 5));
    CHECK(lf.motor_ready(5));
    NEAR(lf.motor_get_speed(), 100, 0.001f);
    feedback.data[0] = 0;
    feedback.data[1] = 4;
    CHECK(net.dispatch(feedback, 6));
    CHECK(lf.feedback().total_counts == 10);
    CHECK(!lf.motor_ready(56));
    DJI_motor_group yaw_group(net, 2, 0x1ff);
    DJI_motor yaw(net, yaw_group, g_dji_config[DJI_YAW], 1, DJI_GM6020);
    CHECK(yaw.valid() && !yaw.motor_set_current(0.1f));
    CHECK(yaw.motor_set_voltage(0.1f) && yaw.command() == 3000);
    DjiConfig cfg = g_dji_config[DJI_YAW];
    cfg.feedback_id = 0x20b;
    DJI_motor_group last_group(net, 2, 0x2ff);
    DJI_motor last(net, last_group, cfg, 7, DJI_GM6020);
    CHECK(last.valid() && last.motor_set_voltage(-0.1f));
    CHECK(last_group.data()[4] == 0xf4 && last_group.data()[5] == 0x48);
    DJI_motor invalid(net, last_group, cfg, 8, DJI_GM6020);
    CHECK(!invalid.valid());
    {
        DJI_motor third(net, group, g_dji_config[2], 3, DJI_M3508);
        CHECK(third.valid() && third.motor_set_current(1));
    }
    {
        DJI_motor replacement(net, group, g_dji_config[2], 3, DJI_M3508);
        CHECK(replacement.valid() && replacement.command() == 0);
    }
    std::puts("PASS DJI objects, reverse sign, shared slots, current/GM6020-voltage distinction, wrapping "
              "and lifetime");
}
static void test_dm_objects() {
    can_network net;
    const DmConfig &lift_cfg = g_dm_config[DM_LIFT_L];
    const DmConfig &pitch_cfg = g_dm_config[DM_BIG_PITCH];
    DM_motor motor(net, lift_cfg), pitch(net, pitch_cfg);
    CHECK(motor.valid() && pitch.valid() && !motor.motor_ready(0));
    MotorFrame command;
    Dm_SpecialFrame(&command, &lift_cfg, true);
    CHECK(motor.apply_frame(command, 1));
    can_device_transmit_member msg{};
    CHECK(net.pop(2, 1, msg));
    CHECK(msg.frame.id == 0x02 && msg.frame.data[7] == 0xfc);
    Dm_PositionFrame(&command, &lift_cfg, 1.0f, 0.2f);
    CHECK(motor.apply_frame(command, 2));
    CHECK(net.pop(2, 2, msg));
    CHECK(!std::memcmp(command.data, msg.frame.data, 8));
    CHECK(!pitch.apply_frame(command, 2));
    command.data[4] ^= 1u;
    CHECK(!motor.apply_frame(command, 2));
    Dm_PositionFrame(&command, &pitch_cfg, 1.0f, 0.2f);
    CHECK(!motor.apply_frame(command, 2) && pitch.apply_frame(command, 2));
    CHECK(net.pop(2, 2, msg) && msg.frame.id == 0x103);
    CHECK(!std::memcmp(command.data, msg.frame.data, 8));
    CHECK(pitch.motor_set_position(0.5f, 0.2f) && pitch.can_send_msg(2));
    CHECK(net.pop(2, 2, msg));
    CHECK(!pitch.motor_set_position(NAN, 0.2f));
    CHECK(!pitch.motor_set_position(13.0f, 0.2f));
    CHECK(!pitch.motor_set_position(1, 1));
    Dm_SpecialFrame(&command, &lift_cfg, false);
    command.data[7] = 0xfe;
    CHECK(!motor.apply_frame(command, 3));
    command.id = 0x201;
    CHECK(!motor.apply_frame(command, 3));
    auto feedback = frame(2, 0x03);
    const uint8_t bytes[8] = {0x12, 0x80, 0, 0x80, 0, 0, 25, 26};
    std::memcpy(feedback.data, bytes, 8);
    CHECK(net.dispatch(feedback, 4));
    CHECK(motor.motor_ready(4));
    NEAR(motor.feedback().position, 0, 0.001f);
    feedback.data[0] = 0x82;
    CHECK(net.dispatch(feedback, 5));
    CHECK(!motor.motor_ready(5));
    std::puts("PASS DM object MIT/PV/enable/disable framing, range checks, mode isolation and fault state");
}
static void inject_feedback(uint32_t now) {
    auto &net = robot_can_network();
    for (unsigned i = 0; i < DJI_COUNT; i++) {
        auto f = frame(g_dji_config[i].bus, g_dji_config[i].feedback_id);
        f.data[0] = 4;
        f.data[1] = 0;
        f.data[6] = 25;
        CHECK(net.dispatch(f, now));
    }
    for (unsigned i = 0; i < DM_COUNT; i++) {
        auto f = frame(2, g_dm_config[i].master_id);
        f.data[0] = static_cast<uint8_t>(0x10 | g_dm_config[i].motor_id);
        f.data[1] = 0x80;
        f.data[3] = 0x80;
        f.data[6] = 25;
        f.data[7] = 25;
        CHECK(net.dispatch(f, now));
    }
    auto p = frame(2, 0x31, 4);
    p.data[0] = 0;
    p.data[1] = 50;
    p.data[2] = 2;
    p.data[3] = 0x58;
    CHECK(net.dispatch(p, now));
}
static void test_system_replay() {
    CHECK(MotorSystem_Init());
    auto &net = robot_can_network();
    CHECK(net.rx_count(1) == 4 && net.rx_count(2) == 7);
    CHECK(g_dji_config[DJI_LR].feedback_id == 0x203 && g_dji_config[DJI_RR].feedback_id == 0x204);
    CHECK(g_dm_config[DM_LIFT_R].motor_id == 0x01 && g_dm_config[DM_LIFT_R].master_id == 0x11);
    RobotCore core;
    RobotCore_Init(&core);
    RobotInput input{};
    input.profile = ROBOT_PROFILE_FULL;
    input.config_confirmed = true;
    input.rc.valid = true;
    input.rc.sw[0] = RC_MID;
    input.rc.sw[1] = RC_MID;
    unsigned frames[6]{};
    for (uint32_t now = 0; now < 1000; now++) {
        inject_feedback(now);
        MotorSystem_CopyFeedback(input.dji, input.dm, &input.power);
        input.rc.stamp_ms = now;
        if (now == 600) {
            input.rc.channel[3] = 330;
            input.rc.sw[0] = RC_UP;
            input.rc.sw[1] = RC_UP;
            input.rc.channel[0] = 330;
            input.rc.channel[1] = 330;
        }
        if (now == 700)
            input.rc.sw[1] = RC_DOWN;
        if (now == 800)
            input.rc.valid = false;
        RobotOutput out;
        RobotCore_Step(&core, &input, now, &out);
        if (out.abort_pending) {
            net.cancel(1);
            net.cancel(2);
        } else if (out.abort_wheel_pending)
            net.cancel(1);
        for (unsigned i = 0; i < out.count; i++)
            CHECK(MotorSystem_SendFrame(&out.frames[i], now));
        unsigned sent = 0;
        for (uint8_t bus = 1; bus <= 2; bus++) {
            can_device_transmit_member msg;
            while (net.pop(bus, now, msg)) {
                CHECK(net.is_current(msg, now));
                bool matched = false;
                for (unsigned i = 0; i < out.count; i++)
                    if (out.frames[i].bus == bus && out.frames[i].id == msg.frame.id) {
                        CHECK(out.frames[i].dlc == msg.frame.dlc &&
                              !std::memcmp(out.frames[i].data, msg.frame.data, 8));
                        matched = true;
                    }
                CHECK(matched);
                ++sent;
                if (bus == 1)
                    ++frames[0];
                else if (msg.frame.id == 0x200)
                    ++frames[1];
                else if (msg.frame.id == 0x1ff)
                    ++frames[2];
                else {
                    unsigned dm = 0;
                    while (dm < DM_COUNT && msg.frame.id != Dm_CommandId(&g_dm_config[dm]))
                        ++dm;
                    CHECK(dm < DM_COUNT);
                    ++frames[3 + dm];
                }
            }
        }
        CHECK(sent == out.count);
    }
    CHECK(!core.armed && frames[0] == 500 && frames[1] == 500 && frames[2] == 500 && frames[3] == 200 &&
          frames[4] == 200 && frames[5] == 200);
    /* Wrong-length power messages must reach its reject handler, not refresh stale data. */
    auto bad = frame(2, 0x31, 8);
    CHECK(!net.dispatch(bad, 1000));
    DjiFeedback d[DJI_COUNT];
    DmFeedback m[DM_COUNT];
    PowerTelemetry p;
    MotorSystem_CopyFeedback(d, m, &p);
    CHECK(!p.valid && p.bad_frames == 1);
    CHECK(!MotorSystem_SendFrame(&bad, 1000));
    std::puts("PASS 1000-cycle real controller -> motor objects -> CAN queues replay: every payload "
              "preserved, 500/200Hz scheduling and radio stop");
}
int main() {
    test_registry();
    test_queues();
    test_dji_objects();
    test_dm_objects();
    test_system_replay();
    std::printf("CAN/MOTOR TESTS PASSED (%u checks)\n", checks);
}
