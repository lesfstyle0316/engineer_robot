// 大疆电机模板作者 Tessia，CAN 设备接口和零点时序按本工程适配。
#include "DJI_motor_driver.h"
#include <cmath>
namespace {
uint16_t feedback_id(uint8_t id, DJI_MOTOR_TYPE type) {
    return (type == DJI_GM6020 ? 0x204u : 0x200u) + id;
}
uint16_t control_id(uint8_t id, DJI_MOTOR_TYPE type) {
    return type == DJI_GM6020 ? (id <= 4 ? 0x1ff : 0x2ff) : (id <= 4 ? 0x200 : 0x1ff);
}
int16_t signed_be(const uint8_t *p) {
    uint16_t u = (static_cast<uint16_t>(p[0]) << 8) | p[1];
    return static_cast<int16_t>(u >= 0x8000u ? static_cast<int32_t>(u) - 65536 : u);
}
void dji_motor_can_tx_write(uint8_t *data, unsigned slot, int16_t current_to_send) {
    const auto value = static_cast<uint16_t>(current_to_send);
    data[slot * 2 + 1] = static_cast<uint8_t>(value & 0xffu);
    data[slot * 2] = static_cast<uint8_t>(value >> 8);
}
} // namespace
DJI_motor_group::DJI_motor_group(can_network &net, uint8_t bus, uint16_t id) : can_tx(net, bus, id) {}
bool DJI_motor_group::claim(unsigned slot, DJI_motor *owner) {
    if (slot > 3 || !owner || owners[slot])
        return false;
    owners[slot] = owner;
    return true;
}
void DJI_motor_group::release(unsigned slot, DJI_motor *owner) {
    if (slot < 4 && owners[slot] == owner) {
        (void)write(slot, 0);
        owners[slot] = nullptr;
    }
}
DJI_motor::~DJI_motor() {
    if (valid_)
        group_.release(slot_, this);
}
bool DJI_motor_group::write(unsigned slot, int16_t value) {
    if (slot > 3 || !owners[slot])
        return false;
    dji_motor_can_tx_write(can_tx.member.data, slot, value);
    return true;
}
bool DJI_motor_group::apply_frame(const MotorFrame &f, uint32_t now) {
    if (f.bus != bus() || f.id != id() || f.dlc != 8)
        return false;
    int16_t values[4];
    for (unsigned i = 0; i < 4; i++) {
        values[i] = signed_be(f.data + 2 * i);
        if (owners[i] ? !owners[i]->accepts_raw(values[i]) : values[i] != 0)
            return false;
    }
    for (unsigned i = 0; i < 4; i++)
        if (owners[i] && !owners[i]->motor_set_raw_output(values[i]))
            return false;
    return can_send_msg(now);
}
DJI_motor::DJI_motor(can_network &net, DJI_motor_group &group, const DjiConfig &cfg, uint8_t id,
                     DJI_MOTOR_TYPE type)
    : motor_dev(cfg.direction), cfg_(cfg), group_(group), type_(type),
      slot_(id ? static_cast<uint8_t>((id - 1) % 4) : 0), valid_(false),
      can_rx(net, cfg.bus, feedback_id(id, type), dji_motor_data_update_callback, this) {
    bool id_ok = id >= 1 && id <= (type == DJI_GM6020 ? 7 : 8);
    valid_ = id_ok && can_rx.valid() && cfg.feedback_id == feedback_id(id, type) && group.bus() == cfg.bus &&
             group.id() == control_id(id, type) && std::fabs(cfg.direction) == 1.0f &&
             std::isfinite(cfg.max_command) && cfg.max_command > 0 && cfg.max_command <= protocol_limit();
    if (valid_)
        valid_ = group_.claim(slot_, this);
}
float DJI_motor::protocol_limit() const {
    return type_ == DJI_M3508 ? ROBOT_C620_MAX_COMMAND : (type_ == DJI_M2006 ? 10000.0f : 30000.0f);
}
bool DJI_motor::accepts_raw(int16_t value) const {
    return valid_ && std::fabs(static_cast<float>(value)) <= cfg_.max_command;
}
bool DJI_motor::motor_set_raw_output(int16_t value) {
    if (!accepts_raw(value))
        return false;
    current_to_send = value;
    return group_.write(slot_, value);
}
bool DJI_motor::normalized_output(float value) {
    if (!valid_ || !std::isfinite(value))
        return false;
    float native = direction_ * std::fmin(1.0f, std::fmax(-1.0f, value)) * protocol_limit();
    native = std::fmin(cfg_.max_command, std::fmax(-cfg_.max_command, native));
    return motor_set_raw_output(static_cast<int16_t>(native));
}
bool DJI_motor::motor_set_current(float value) {
    return type_ != DJI_GM6020 && normalized_output(value);
}
bool DJI_motor::motor_set_voltage(float value) {
    return type_ == DJI_GM6020 && normalized_output(value);
}
void DJI_motor::motor_stop() {
    (void)motor_set_raw_output(0);
}
bool DJI_motor::motor_ready(uint32_t now) const {
    return valid_ && motor_data.zero_ready && (uint32_t)(now - motor_data.stamp_ms) < ROBOT_MOTOR_TIMEOUT_MS &&
           (type_ == DJI_M3508 || !motor_data.position_lost);
}
void DJI_motor::dji_motor_data_update_callback(can_device_receive *rx, const MotorFrame &f, uint32_t now) {
    auto *self = static_cast<DJI_motor *>(rx->context());
    if (self && self->valid_)
        (void)Dji_Decode(&self->motor_data, f.data, f.dlc, now);
}

bool Dji_Decode(DjiFeedback *motor_data, const uint8_t *data, uint8_t dlc, uint32_t now) {
    if (dlc != 8u)
        return false;

    const uint16_t ecd = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (ecd > 8191u)
        return false;

    ++motor_data->sequence;
    if (motor_data->zero_ready) {
        if (static_cast<uint32_t>(now - motor_data->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS)
            motor_data->position_lost = true;
        const uint16_t last_ecd = motor_data->encoder;
        int ecd_raw_rate = ecd - last_ecd;
        if (ecd_raw_rate > 4096)
            ecd_raw_rate -= 8192;
        else if (ecd_raw_rate < -4096)
            ecd_raw_rate += 8192;
        motor_data->total_counts += ecd_raw_rate;
    } else if (motor_data->sequence == ROBOT_DJI_ZERO_FRAMES) {
        motor_data->zero_ready = true;
    }

    motor_data->encoder = ecd;
    motor_data->rpm = signed_be(data + 2);
    motor_data->current = signed_be(data + 4);
    motor_data->temperature = data[6];
    motor_data->stamp_ms = now;
    motor_data->seen = true;
    return true;
}

float Dji_JointDegrees(const DjiFeedback *motor_data, const DjiConfig *config) {
    return static_cast<float>(motor_data->total_counts) * (360.0f / 8192.0f) * config->direction /
           config->reduction;
}

void Dji_GroupFrame(MotorFrame *frame, uint8_t bus, uint16_t id, const int16_t values[4]) {
    *frame = {};
    frame->bus = bus;
    frame->id = id;
    frame->dlc = 8;
    for (unsigned slot = 0; slot < 4; ++slot)
        dji_motor_can_tx_write(frame->data, slot, values[slot]);
}
