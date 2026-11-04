#include "drv_power.h"
#include <math.h>
#include <string.h>
static float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static int16_t signed16(uint16_t u) {
    return (int16_t)(u >= 0x8000u ? (int32_t)u - 65536 : (int32_t)u);
}
static int16_t unpack16(const uint8_t *b, uint8_t order) {
    uint16_t u = order == POWER_ORDER_LITTLE_ENDIAN ? ((uint16_t)b[0] | ((uint16_t)b[1] << 8))
                                                    : (((uint16_t)b[0] << 8) | b[1]);
    return signed16(u);
}
bool PowerTelemetry_Decode(PowerTelemetry *s, const uint8_t *b, uint8_t n, uint8_t order, uint32_t now) {
    if (n == 4u)
        memcpy(s->raw, b, 4);
    if (n != 4u || (order != POWER_ORDER_LITTLE_ENDIAN && order != POWER_ORDER_BIG_ENDIAN)) {
        s->valid = false;
        ++s->bad_frames;
        return false;
    }
    int16_t p = unpack16(b, order), z = unpack16(b + 2, order);
    /* 规则允许负功率（字段为int16），缓冲严格为-10..60J。 */
    if (z < -100 || z > 600) {
        s->valid = false;
        ++s->bad_frames;
        return false;
    }
    s->chassis_power_w = (float)p * 0.1f;
    s->buffer_j = (float)z * 0.1f;
    s->stamp_ms = now;
    s->seen = true;
    s->valid = true;
    ++s->sequence;
    return true;
}
bool PowerConfig_IsValid(const PowerConfig *c) {
    return isfinite(c->resistance_ohm) && c->resistance_ohm >= 0 && isfinite(c->mechanical_w_per_a_rpm) &&
           c->mechanical_w_per_a_rpm >= 0 && isfinite(c->baseline_w) && c->baseline_w >= 0 &&
           isfinite(c->rule_limit_w) && c->rule_limit_w > 0 && c->rule_limit_w <= 65.0f &&
           isfinite(c->safety_margin_w) && c->safety_margin_w >= 0 && c->safety_margin_w < c->rule_limit_w &&
           isfinite(c->bench_limit_w) && c->bench_limit_w >= 0 &&
           c->bench_limit_w <= c->rule_limit_w - c->safety_margin_w && isfinite(c->buffer_target_j) &&
           c->buffer_target_j > 0 && c->buffer_target_j <= 60.0f && isfinite(c->recharge_time_s) &&
           c->recharge_time_s > 0 && isfinite(c->reserve_up_tau_s) && c->reserve_up_tau_s > 0 &&
           isfinite(c->reserve_down_tau_s) && c->reserve_down_tau_s > 0 && isfinite(c->k_recovery_per_s) &&
           c->k_recovery_per_s > 0 && c->referee_timeout_ms > 0u && c->referee_timeout_ms < 0x80000000u &&
           c->cutoff_hold_ms >= 5000u && c->cutoff_hold_ms < 0x80000000u;
}
static float model(float a, float b, float l, float k) {
    return (a * k + b) * k + l;
}
PowerSolution Power_Solve(const PowerConfig *cfg, const float amp[4], const float rpm[4], float budget) {
    PowerSolution s = {0};
    s.baseline_w = cfg->baseline_w;
    if (!PowerConfig_IsValid(cfg) || !isfinite(budget) || budget < 0)
        return s;
    for (unsigned i = 0; i < POWER_WHEEL_COUNT; i++) {
        if (!isfinite(amp[i]) || !isfinite(rpm[i]))
            return s;
        s.a_w += cfg->resistance_ohm * amp[i] * amp[i];
        s.b_w += cfg->mechanical_w_per_a_rpm * amp[i] * rpm[i];
    }
    if (!isfinite(s.a_w) || !isfinite(s.b_w))
        return s;
    s.predicted_before_w = model(s.a_w, s.b_w, cfg->baseline_w, 1.0f);
    s.predicted_after_w = cfg->baseline_w;
    s.valid = true;
    /* 静态损耗已经超过预算时，零电流也不能满足模型，明确报告不可满足。
     * 不依靠制动回馈来抵消静态超预算，不为此主动施加电流。 */
    float available = budget - cfg->baseline_w;
    if (available < 0)
        return s;
    s.feasible = true;
    if (s.predicted_before_w <= budget) {
        s.k = 1.0f;
        s.predicted_after_w = s.predicted_before_w;
        return s;
    }
    if (s.a_w <= 1.0e-12f) {
        s.k = s.b_w > 0 ? clamp(available / s.b_w, 0.0f, 1.0f) : 0.0f;
    } else {
        /* hypot形式避免B^2溢出；B>=0时用等价有理式避免相减消去。 */
        float disc = hypotf(s.b_w, 2.0f * sqrtf(s.a_w) * sqrtf(available));
        if (!isfinite(disc)) {
            s.valid = false;
            s.feasible = false;
            return s;
        }
        float root = s.b_w >= 0 ? (disc + s.b_w > 0 ? 2.0f * available / (disc + s.b_w) : 0.0f)
                                : (disc - s.b_w) / (2.0f * s.a_w);
        s.k = clamp(root, 0.0f, 1.0f);
    }
    s.predicted_after_w = model(s.a_w, s.b_w, cfg->baseline_w, s.k);
    return s;
}
float Power_PredictCounts(const PowerConfig *c, const int16_t counts[4], const int16_t rpm[4]) {
    float p = c->baseline_w;
    for (unsigned i = 0; i < POWER_WHEEL_COUNT; i++) {
        float a = (float)counts[i] * POWER_C620_A_PER_COUNT;
        p += c->resistance_ohm * a * a + c->mechanical_w_per_a_rpm * a * (float)rpm[i];
    }
    return p;
}
void PowerLimiter_Init(PowerLimiter *s) {
    memset(s, 0, sizeof(*s));
}
void PowerLimiter_ResetOutput(PowerLimiter *s) {
    s->k_applied = 0;
    s->predicted_after_w = 0;
}
static void quantize(const int16_t in[4], float k, int16_t out[4]) {
    for (unsigned i = 0; i < POWER_WHEEL_COUNT; i++)
        out[i] = (int16_t)((float)in[i] * k);
}
float Power_BufferStep(float z, float p, float limit, float dt) {
    if (!isfinite(z) || !isfinite(p) || !isfinite(limit) || !isfinite(dt) || dt < 0)
        return NAN;
    return clamp(z + (limit - p) * dt, -10.0f, 60.0f);
}
void PowerLimiter_Apply(PowerLimiter *s, const PowerConfig *cfg, const PowerTelemetry *t,
                        const int16_t request[4], const int16_t measured[4], const int16_t rpm[4],
                        uint32_t now, float dt, int16_t out[4]) {
    memset(out, 0, 4 * sizeof(*out));
    s->status = 0;
    s->k_max = 0;
    s->predicted_after_w = 0;
    if (!PowerConfig_IsValid(cfg) || !isfinite(dt) || dt <= 0) {
        s->status = POWER_STATUS_INVALID_MODEL;
        s->k_applied = 0;
        return;
    }
    bool fresh = t->seen && t->valid && (uint32_t)(now - t->stamp_ms) < cfg->referee_timeout_ms;
    bool new_bad = t->bad_frames != s->last_bad_frames;
    s->last_bad_frames = t->bad_frames;
    s->budget_w = cfg->rule_limit_w - cfg->safety_margin_w;
    if (fresh) {
        s->measured_power_w = t->chassis_power_w;
        s->buffer_j = t->buffer_j;
        /* 此函数也可由非CAN源调用，因此对浮点输入再校验一次。 */
        if (!isfinite(t->chassis_power_w) || !isfinite(t->buffer_j) || t->buffer_j < -10 ||
            t->buffer_j > 60) {
            s->status = POWER_STATUS_REFEREE_INVALID;
            s->k_applied = 0;
            return;
        }
        if (!s->sample_seen || t->sequence != s->last_sequence) {
            float sample_dt =
                s->sample_seen ? (float)(uint32_t)(t->stamp_ms - s->last_sample_ms) * 0.001f : dt;
            sample_dt = clamp(sample_dt, 0.001f, 1.0f);
            s->model_observed_w = Power_PredictCounts(cfg, measured, rpm);
            float target = clamp(t->chassis_power_w - s->model_observed_w, 0.0f, cfg->rule_limit_w);
            float tau = target > s->residual_reserve_w ? cfg->reserve_up_tau_s : cfg->reserve_down_tau_s;
            s->residual_reserve_w += sample_dt / (tau + sample_dt) * (target - s->residual_reserve_w);
            s->last_sequence = t->sequence;
            s->last_sample_ms = t->stamp_ms;
            s->sample_seen = true;
            if (t->buffer_j <= 0) {
                s->last_cutoff_ms = now;
                s->cutoff_active = true;
            }
        }
        if (t->buffer_j < cfg->buffer_target_j) {
            s->budget_w -= (cfg->buffer_target_j - t->buffer_j) / cfg->recharge_time_s;
            s->status |= POWER_STATUS_LOW_BUFFER;
        }
    } else {
        if (cfg->require_referee) {
            s->status = (!t->valid && (t->seen || t->bad_frames)) ? POWER_STATUS_REFEREE_INVALID
                                                                  : POWER_STATUS_REFEREE_MISSING;
            s->k_applied = 0;
            return;
        }
        s->budget_w = fminf(s->budget_w, cfg->bench_limit_w);
        s->status |= POWER_STATUS_BENCH_MODEL_ONLY;
    }
    if (new_bad) {
        s->status |= POWER_STATUS_REFEREE_INVALID;
        s->k_applied = 0;
        return;
    }
    if (s->cutoff_active) {
        if ((uint32_t)(now - s->last_cutoff_ms) >= cfg->cutoff_hold_ms && fresh && t->buffer_j > 0)
            s->cutoff_active = false;
        else {
            s->status |= POWER_STATUS_CUTOFF_HOLD;
            s->k_applied = 0;
            return;
        }
    }
    s->budget_w = fmaxf(0.0f, s->budget_w - s->residual_reserve_w);
    float amp[4], speed[4];
    for (unsigned i = 0; i < 4; i++) {
        amp[i] = request[i] * POWER_C620_A_PER_COUNT;
        speed[i] = (float)rpm[i];
    }
    PowerSolution solution = Power_Solve(cfg, amp, speed, s->budget_w);
    s->a_w = solution.a_w;
    s->b_w = solution.b_w;
    s->predicted_before_w = solution.predicted_before_w;
    s->predicted_after_w = solution.predicted_after_w;
    if (!solution.valid) {
        s->status |= POWER_STATUS_INVALID_MODEL;
        s->k_applied = 0;
        return;
    }
    if (!solution.feasible) {
        s->status |= POWER_STATUS_NO_STATIC_BUDGET;
        s->k_applied = 0;
        return;
    }
    s->k_max = solution.k;
    /* 下降立即响应；回升限速，减少限功率解除时的电流突变。 */
    float k = fminf(solution.k, s->k_applied + cfg->k_recovery_per_s * dt);
    quantize(request, k, out);
    float predicted = Power_PredictCounts(cfg, out, rpm);
    if (predicted > s->budget_w) {
        /* 带符号制动项下，四次独立整数截断不一定降低总功率。
         * 对实际整型CAN命令回代，并保留已验证可行的下界。 */
        float lo = 0, hi = k;
        int16_t candidate[4];
        for (unsigned i = 0; i < 20; i++) {
            float mid = (lo + hi) * 0.5f;
            quantize(request, mid, candidate);
            if (Power_PredictCounts(cfg, candidate, rpm) <= s->budget_w)
                lo = mid;
            else
                hi = mid;
        }
        k = lo;
        quantize(request, k, out);
        predicted = Power_PredictCounts(cfg, out, rpm);
        s->status |= POWER_STATUS_QUANTIZED;
    }
    if (!isfinite(predicted) || predicted > s->budget_w) {
        memset(out, 0, 4 * sizeof(*out));
        k = 0;
        predicted = cfg->baseline_w;
    }
    s->k_applied = k;
    s->predicted_after_w = predicted;
    if (k < 0.999999f)
        s->status |= POWER_STATUS_SCALED;
}
