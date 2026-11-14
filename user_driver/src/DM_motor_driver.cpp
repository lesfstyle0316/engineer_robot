// 达妙电机模板作者 Tessia，MIT 量程、使能和 CAN 设备接口按本工程适配。
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

void pack_float_le(uint8_t *data, float value) {
    static_assert(sizeof(float) == 4, "IEEE754 binary32 required");
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(raw));
    for (unsigned i = 0; i < 4; ++i)
        data[i] = static_cast<uint8_t>(raw >> (8 * i));
}

unsigned float_to_uint(float value, float min, float max, unsigned bits) {
    const float normalized = (std::fmin(std::fmax(value, min), max) - min) / (max - min);
    return static_cast<unsigned>(normalized * static_cast<float>((1u << bits) - 1u) + 0.5f);
}
} // namespace
DM_motor::DM_motor(can_network &net, const DmConfig &cfg)
    : motor_dev(cfg.direction), cfg_(cfg), valid_(false),
      can_rx(net, 2, cfg.master_id, dm_motor_data_update_callback, this),
      can_tx(net, 2, Dm_CommandId(&cfg)) {
    valid_ = can_rx.valid() && Dm_ConfigIsValid(&cfg);
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
    if (!valid_ || cfg_.mode != DM_MODE_PV || !std::isfinite(pos) || !std::isfinite(vel) || std::fabs(pos) > cfg_.p_max || vel <= 0 ||
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
    if (special) {
        if (f.data[7] != 0xfc && f.data[7] != 0xfd)
            return false;
        return motor_enable(f.data[7] == 0xfc) && can_send_msg(now);
    }
    if (cfg_.mode == DM_MODE_PV)
        return motor_set_position(float_le(f.data), float_le(f.data + 4)) && can_send_msg(now);
    /* 位置位域覆盖整个配置范围，其他位域必须与本机 MIT 保持参数一致。 */
    MotorFrame hold;
    Dm_PositionFrame(&hold, &cfg_, 0.0f, cfg_.velocity_limit_rad_s);
    if (std::memcmp(f.data + 2, hold.data + 2, 6) != 0)
        return false;
    can_tx.member = f;
    return can_send_msg(now);
}
void DM_motor::dm_motor_data_update_callback(can_device_receive *rx, const MotorFrame &f, uint32_t now) {
    auto *self = static_cast<DM_motor *>(rx->context());
    if (self && self->valid_)
        (void)Dm_Decode(&self->motor_data, &self->cfg_, f.data, f.dlc, now);
}

float Dm_Unpack(unsigned value, float max, unsigned bits) {
    return static_cast<float>(value) * (2.0f * max) / static_cast<float>((1u << bits) - 1u) - max;
}

bool Dm_Decode(DmFeedback *motor_data, const DmConfig *config, const uint8_t *data, uint8_t dlc,
               uint32_t now) {
    if (dlc != 8u || (data[0] & 0x0fu) != (config->motor_id & 0x0fu))
        return false;

    const unsigned pos = (data[1] << 8) | data[2];
    const unsigned vel = (data[3] << 4) | (data[4] >> 4);
    const unsigned torque = ((data[4] & 0x0fu) << 8) | data[5];
    motor_data->state = data[0] >> 4;
    motor_data->position = Dm_Unpack(pos, config->p_max, 16);
    motor_data->velocity = Dm_Unpack(vel, config->v_max, 12);
    motor_data->torque = Dm_Unpack(torque, config->t_max, 12);
    motor_data->mos_temperature = data[6];
    motor_data->rotor_temperature = data[7];
    motor_data->stamp_ms = now;
    motor_data->seen = true;
    ++motor_data->sequence;
    return true;
}

bool Dm_ConfigIsValid(const DmConfig *c) {
    if (c->motor_id < 1 || c->motor_id > 15 || c->master_id > 0x7ffu ||
        std::fabs(c->direction) != 1.0f || !std::isfinite(c->external_reduction) || c->external_reduction <= 0 ||
        !std::isfinite(c->p_max) || c->p_max <= 0 || !std::isfinite(c->v_max) || c->v_max <= 0 ||
        !std::isfinite(c->t_max) || c->t_max <= 0 || !std::isfinite(c->velocity_limit_rad_s) ||
        c->velocity_limit_rad_s <= 0)
        return false;
    if (c->mode == DM_MODE_PV)
        return true;
    return c->mode == DM_MODE_MIT && c->mit_kp >= 500.0f / 4095.0f && c->mit_kp <= 500.0f &&
           c->mit_kd >= 5.0f / 4095.0f && c->mit_kd <= 5.0f;
}
uint16_t Dm_CommandId(const DmConfig *c) {
    return c->motor_id + (c->mode == DM_MODE_PV ? 0x100u : 0u);
}
void Dm_SpecialFrame(MotorFrame *f, const DmConfig *c, bool enable) {
    *f = {};
    f->bus = 2;
    f->id = Dm_CommandId(c);
    f->dlc = 8;
    std::memset(f->data, 0xff, sizeof(f->data));
    f->data[7] = enable ? 0xfc : 0xfd;
}

void Dm_PositionFrame(MotorFrame *frame, const DmConfig *config, float p_des, float v_limit) {
    *frame = {};
    frame->bus = 2;
    frame->id = Dm_CommandId(config);
    frame->dlc = 8;
    uint8_t *data = frame->data;
    if (config->mode == DM_MODE_PV) {
        pack_float_le(data, p_des);
        pack_float_le(data + 4, std::fabs(v_limit));
        return;
    }

    const unsigned pos_tmp = float_to_uint(p_des, -config->p_max, config->p_max, 16);
    const unsigned vel_tmp = float_to_uint(0.0f, -config->v_max, config->v_max, 12);
    const unsigned kp_tmp = float_to_uint(config->mit_kp, 0.0f, 500.0f, 12);
    const unsigned kd_tmp = float_to_uint(config->mit_kd, 0.0f, 5.0f, 12);
    const unsigned tor_tmp = float_to_uint(0.0f, -config->t_max, config->t_max, 12);
    data[0] = static_cast<uint8_t>(pos_tmp >> 8);
    data[1] = static_cast<uint8_t>(pos_tmp);
    data[2] = static_cast<uint8_t>(vel_tmp >> 4);
    data[3] = static_cast<uint8_t>(((vel_tmp & 0x0fu) << 4) | (kp_tmp >> 8));
    data[4] = static_cast<uint8_t>(kp_tmp);
    data[5] = static_cast<uint8_t>(kd_tmp >> 4);
    data[6] = static_cast<uint8_t>(((kd_tmp & 0x0fu) << 4) | (tor_tmp >> 8));
    data[7] = static_cast<uint8_t>(tor_tmp);
}
