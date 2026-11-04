#include "drv_rc.h"
#include "robot_config.h"
#include <stdlib.h>
void Rc_Invalidate(RcState *s) {
    s->valid = false;
    ++s->bad_frames;
}
bool Rc_Decode(RcState *s, const uint8_t *b, uint16_t n, uint32_t now) {
    if (n != 18u) {
        Rc_Invalidate(s);
        return false;
    }
    uint16_t c[4];
    c[0] = ((uint16_t)b[0] | ((uint16_t)b[1] << 8)) & 0x7ffu;
    c[1] = (((uint16_t)b[1] >> 3) | ((uint16_t)b[2] << 5)) & 0x7ffu;
    c[2] = (((uint16_t)b[2] >> 6) | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x7ffu;
    c[3] = (((uint16_t)b[4] >> 1) | ((uint16_t)b[5] << 7)) & 0x7ffu;
    uint8_t left = (b[5] >> 6) & 3u, right = (b[5] >> 4) & 3u;
    for (unsigned i = 0; i < 4; i++)
        if (c[i] < 364u || c[i] > 1684u) {
            Rc_Invalidate(s);
            return false;
        }
    if (left == 0u || right == 0u) {
        Rc_Invalidate(s);
        return false;
    }
    for (unsigned i = 0; i < 4; i++)
        s->channel[i] = (int16_t)((int)c[i] - 1024);
    s->sw[0] = left;
    s->sw[1] = right;
    s->stamp_ms = now;
    ++s->sequence;
    s->valid = true;
    return true;
}
bool Rc_IsNeutral(const RcState *s) {
    if (!s->valid || s->sw[0] != RC_MID || s->sw[1] != RC_MID)
        return false;
    for (unsigned i = 0; i < 4; i++)
        if (abs(s->channel[i]) > ROBOT_RC_DEADBAND)
            return false;
    return true;
}
float Rc_Axis(const RcState *s, unsigned axis) {
    if (axis > 3u || !s->valid)
        return 0.0f;
    int x = s->channel[axis];
    if (abs(x) <= ROBOT_RC_DEADBAND)
        return 0.0f;
    return x > 0 ? (float)(x - ROBOT_RC_DEADBAND) / (660 - ROBOT_RC_DEADBAND)
                 : (float)(x + ROBOT_RC_DEADBAND) / (660 - ROBOT_RC_DEADBAND);
}
