#ifndef DRV_RC_H
#define DRV_RC_H
#include <stdbool.h>
#include <stdint.h>
enum { RC_UP=1, RC_DOWN=2, RC_MID=3 };
typedef struct {
    int16_t channel[4]; /* 0=右水平 1=右竖直 2=左水平 3=左竖直，中点为0 */
    uint8_t sw[2];     /* 0=左S1 1=右S2 */
    uint32_t stamp_ms, sequence, bad_frames;
    bool valid;
} RcState;
bool Rc_Decode(RcState *state, const uint8_t *data, uint16_t len, uint32_t now);
void Rc_Invalidate(RcState *state);
bool Rc_IsNeutral(const RcState *state);
float Rc_Axis(const RcState *state, unsigned axis);
#endif
