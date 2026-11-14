#ifndef DRV_POWER_H
#define DRV_POWER_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
/* 此接口只有四个轮电机，不接收升降/机械臂电机数组。 */
#define POWER_WHEEL_COUNT 4u
#define POWER_C620_A_PER_COUNT (20.0f / 16384.0f)
enum { POWER_ORDER_UNCONFIRMED = 0, POWER_ORDER_LITTLE_ENDIAN = 1, POWER_ORDER_BIG_ENDIAN = 2 };
enum {
    POWER_STATUS_REFEREE_MISSING = 1u,
    POWER_STATUS_REFEREE_INVALID = 2u,
    POWER_STATUS_LOW_BUFFER = 4u,
    POWER_STATUS_CUTOFF_HOLD = 8u,
    POWER_STATUS_INVALID_MODEL = 16u,
    POWER_STATUS_NO_STATIC_BUDGET = 32u,
    POWER_STATUS_SCALED = 64u,
    POWER_STATUS_QUANTIZED = 128u,
    POWER_STATUS_BENCH_MODEL_ONLY = 256u
};
typedef struct {
    float resistance_ohm, mechanical_w_per_a_rpm, baseline_w;
    float rule_limit_w, safety_margin_w, bench_limit_w;
    float buffer_target_j, recharge_time_s;
    float reserve_up_tau_s, reserve_down_tau_s, k_recovery_per_s;
    uint32_t referee_timeout_ms, cutoff_hold_ms;
    bool require_referee;
} PowerConfig;
extern const PowerConfig g_power_config;
typedef struct {
    bool seen, valid;
    uint8_t last_dlc; /* 最近一次收到的裁判帧长度，含坏帧 */
    float chassis_power_w, buffer_j;
    uint32_t stamp_ms, sequence, bad_frames;
    uint8_t raw[4];
} PowerTelemetry;
typedef struct {
    bool valid, feasible;
    float a_w, b_w, baseline_w, predicted_before_w, predicted_after_w, k;
} PowerSolution;
typedef struct {
    float a_w, b_w, budget_w, model_observed_w, residual_reserve_w;
    float predicted_before_w, predicted_after_w, k_max, k_applied;
    float measured_power_w, buffer_j;
    uint32_t status, last_sequence, last_bad_frames, last_sample_ms, last_cutoff_ms;
    bool sample_seen, cutoff_active;
} PowerLimiter;
bool PowerTelemetry_Decode(PowerTelemetry *s, const uint8_t *data, uint8_t dlc, uint8_t byte_order,
                           uint32_t now);
bool PowerConfig_IsValid(const PowerConfig *cfg);
/* I为电流A，n为电机转子rpm；二者均使用同一个电机原生正方向。 */
PowerSolution Power_Solve(const PowerConfig *cfg, const float amp[4], const float rpm[4], float budget_w);
float Power_PredictCounts(const PowerConfig *cfg, const int16_t counts[4], const int16_t rpm[4]);
void PowerLimiter_Init(PowerLimiter *s);
void PowerLimiter_ResetOutput(PowerLimiter *s);
/* requested和measured都只包含四个C620轮电机。输出不会改变符号或放大原请求。 */
void PowerLimiter_Apply(PowerLimiter *s, const PowerConfig *cfg, const PowerTelemetry *telemetry,
                        const int16_t requested[4], const int16_t measured[4], const int16_t rpm[4],
                        uint32_t now, float dt, int16_t output[4]);
/* 按功率差积分缓冲能量，仅用于验算和日志。 */
float Power_BufferStep(float joules, float measured_w, float limit_w, float dt);
#ifdef __cplusplus
}
#endif
#endif
