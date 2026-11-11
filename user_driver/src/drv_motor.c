#include "drv_motor.h"
#include <string.h>
#include <math.h>
static int16_t signed_be(const uint8_t *p) {
    uint16_t u = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)(u >= 0x8000u ? (int32_t)u - 65536 : (int32_t)u);
}
bool Dji_Decode(DjiFeedback *s, const uint8_t *b, uint8_t dlc, uint32_t now) {
    if (dlc != 8u)
        return false;
    uint16_t enc = ((uint16_t)b[0] << 8) | b[1];
    if (enc > 8191u)
        return false;
    if (s->seen) {
        if ((uint32_t)(now - s->stamp_ms) >= ROBOT_MOTOR_TIMEOUT_MS)
            s->position_lost = true;
        int d = (int)enc - (int)s->encoder;
        if (d > 4096)
            d -= 8192;
        else if (d < -4096)
            d += 8192;
        s->total_counts += d;
    }
    s->encoder = enc;
    s->rpm = signed_be(b + 2);
    s->current = signed_be(b + 4);
    s->temperature = b[6];
    s->stamp_ms = now;
    s->seen = true;
    ++s->sequence;
    return true;
}
float Dji_JointDegrees(const DjiFeedback *s, const DjiConfig *c) {
    return (float)s->total_counts * (360.0f / 8192.0f) * c->direction / c->reduction;
}
float Dm_Unpack(unsigned x, float max, unsigned bits) {
    return (float)x * (2.0f * max) / (float)((1u << bits) - 1u) - max;
}
bool Dm_Decode(DmFeedback *s, const DmConfig *c, const uint8_t *b, uint8_t dlc, uint32_t now) {
    if (dlc != 8u || (b[0] & 0x0fu) != (c->motor_id & 0x0fu))
        return false;
    s->state = b[0] >> 4;
    s->position = Dm_Unpack(((unsigned)b[1] << 8) | b[2], c->p_max, 16);
    s->velocity = Dm_Unpack(((unsigned)b[3] << 4) | (b[4] >> 4), c->v_max, 12);
    s->torque = Dm_Unpack(((unsigned)(b[4] & 15u) << 8) | b[5], c->t_max, 12);
    s->mos_temperature = b[6];
    s->rotor_temperature = b[7];
    s->stamp_ms = now;
    s->seen = true;
    ++s->sequence;
    return true;
}
void Dji_GroupFrame(MotorFrame *f, uint8_t bus, uint16_t id, const int16_t v[4]) {
    memset(f, 0, sizeof(*f));
    f->bus = bus;
    f->id = id;
    f->dlc = 8;
    for (unsigned i = 0; i < 4; i++) {
        uint16_t u = (uint16_t)v[i];
        f->data[2 * i] = (uint8_t)(u >> 8);
        f->data[2 * i + 1] = (uint8_t)u;
    }
}
bool Dm_ConfigIsValid(const DmConfig *c) {
    if (c->motor_id < 1 || c->motor_id > 15 || c->master_id > 0x7ffu ||
        fabsf(c->direction) != 1.0f || !isfinite(c->external_reduction) || c->external_reduction <= 0 ||
        !isfinite(c->p_max) || c->p_max <= 0 || !isfinite(c->v_max) || c->v_max <= 0 ||
        !isfinite(c->t_max) || c->t_max <= 0 || !isfinite(c->velocity_limit_rad_s) ||
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
    memset(f, 0, sizeof(*f));
    f->bus = 2;
    f->id = Dm_CommandId(c);
    f->dlc = 8;
    memset(f->data, 0xff, 8);
    f->data[7] = enable ? 0xfc : 0xfd;
}
/* 显式生成 IEEE754 小端报文；不依赖未对齐访问或指针强转。 */
static void pack_float_le(uint8_t *b, float x) {
    uint32_t u;
    _Static_assert(sizeof(float) == 4, "IEEE754 binary32 required");
    memcpy(&u, &x, 4);
    for (unsigned i = 0; i < 4; i++)
        b[i] = (uint8_t)(u >> (8 * i));
}
static unsigned dm_pack(float value, float min, float max, unsigned bits) {
    float normalized = (fminf(fmaxf(value, min), max) - min) / (max - min);
    return (unsigned)(normalized * (float)((1u << bits) - 1u) + 0.5f);
}
void Dm_PositionFrame(MotorFrame *f, const DmConfig *c, float p, float v) {
    memset(f, 0, sizeof(*f));
    f->bus = 2;
    f->id = Dm_CommandId(c);
    f->dlc = 8;
    if (c->mode == DM_MODE_PV) {
        pack_float_le(f->data, p);
        pack_float_le(f->data + 4, fabsf(v));
        return;
    }
    unsigned position = dm_pack(p, -c->p_max, c->p_max, 16);
    unsigned velocity = dm_pack(0.0f, -c->v_max, c->v_max, 12);
    unsigned kp = dm_pack(c->mit_kp, 0.0f, 500.0f, 12);
    unsigned kd = dm_pack(c->mit_kd, 0.0f, 5.0f, 12);
    unsigned torque = dm_pack(0.0f, -c->t_max, c->t_max, 12);
    f->data[0] = (uint8_t)(position >> 8);
    f->data[1] = (uint8_t)position;
    f->data[2] = (uint8_t)(velocity >> 4);
    f->data[3] = (uint8_t)(((velocity & 0x0fu) << 4) | (kp >> 8));
    f->data[4] = (uint8_t)kp;
    f->data[5] = (uint8_t)(kd >> 4);
    f->data[6] = (uint8_t)(((kd & 0x0fu) << 4) | (torque >> 8));
    f->data[7] = (uint8_t)torque;
}
