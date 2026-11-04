#include "DM_motor_driver.h"
#include <cmath>
#include <cstring>
namespace {
float float_le(const uint8_t *b) {
    uint32_t u = 0;
    for (unsigned i = 0; i < 4; i++)
        u |= static_cast<uint32_t>(b[i]) << (8 * i);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}
} // namespace
DM_motor::DM_motor(can_network &net, const DmConfig &cfg)
    : motor_dev(cfg.direction), cfg_(cfg), valid_(false),
      can_rx(net, 2, cfg.master_id, dm_motor_data_update_callback, this),
      can_tx(net, 2, 0x100u + cfg.motor_id) {
    valid_ = can_rx.valid() && cfg.motor_id >= 1 && cfg.motor_id <= 15 && std::fabs(cfg.direction) == 1.0f &&
             std::isfinite(cfg.p_max) && cfg.p_max > 0 && std::isfinite(cfg.v_max) && cfg.v_max > 0 &&
             std::isfinite(cfg.t_max) && cfg.t_max > 0 && std::isfinite(cfg.velocity_limit_rad_s) &&
             cfg.velocity_limit_rad_s > 0;
    motor_stop();
}
bool DM_motor::motor_ready(uint32_t now) const {
    return valid_ && motor_data.seen && (uint32_t)(now - motor_data.stamp_ms) < ROBOT_MOTOR_TIMEOUT_MS &&
           motor_data.state == 1;
}
bool DM_motor::motor_enable(bool enable) {
    if (!valid_)
        return false;
    Dm_SpecialFrame(&can_tx.member, &cfg_, enable);
    return true;
}
void DM_motor::motor_stop() {
    (void)motor_enable(false);
}
bool DM_motor::motor_set_position(float pos, float vel) {
    if (!valid_ || !std::isfinite(pos) || !std::isfinite(vel) || std::fabs(pos) > cfg_.p_max || vel <= 0 ||
        vel > cfg_.velocity_limit_rad_s)
        return false;
    Dm_PositionFrame(&can_tx.member, &cfg_, pos, vel);
    return true;
}
bool DM_motor::apply_frame(const MotorFrame &f, uint32_t now) {
    if (!valid_ || f.bus != 2 || f.id != command_id() || f.dlc != 8)
        return false;
    bool special = true;
    for (unsigned i = 0; i < 7; i++)
        special = special && f.data[i] == 0xff;
    bool staged =
        special ? (f.data[7] == 0xfc ? motor_enable(true) : (f.data[7] == 0xfd ? motor_enable(false) : false))
                : motor_set_position(float_le(f.data), float_le(f.data + 4));
    return staged && can_send_msg(now);
}
void DM_motor::dm_motor_data_update_callback(can_device_receive *rx, const MotorFrame &f, uint32_t now) {
    auto *self = static_cast<DM_motor *>(rx->context());
    if (self && self->valid_)
        (void)Dm_Decode(&self->motor_data, &self->cfg_, f.data, f.dlc, now);
}
