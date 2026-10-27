#include "ir_wall_signal.h"
#include <string.h>

/* Same float EMA and integer rounding as IR_FilterStep(ema10).
 * The dedicated implementation executes only one filter bank in TIM11. */
static uint16_t RoundedAdc(float value)
{
    if (value <= 0.0f) return 0U;
    if (value >= 4095.0f) return 4095U;
    return (uint16_t)(value + 0.5f);
}

void IR_WallSignal_Init(IR_WallSignal *state)
{
    if (state) memset(state, 0, sizeof(*state));
}

uint8_t IR_WallSignal_Update(IR_WallSignal *state,
                             const uint16_t raw_adc[IR_WALL_CHANNELS],
                             uint32_t scan_count,
                             uint32_t completed_us)
{
    uint8_t i, reset;
    uint32_t delta = 1U;
    if (!state || !raw_adc) return 0U;
    for (i = 0U; i < IR_WALL_CHANNELS; ++i) {
        if (raw_adc[i] > 4095U) {
            state->rejected_frames++;
            return 0U; /* Atomic rejection: do not update any channel. */
        }
    }
    if (!state->initialized && scan_count == 0U) {
        state->rejected_frames++;
        return 0U; /* 0 is the raw driver's not-yet-started value. */
    }
    if (state->initialized) {
        delta = scan_count - state->scan_count; /* uint32 wrap-safe */
        if (delta == 0U || delta > 0x7FFFFFFFUL ||
            (int32_t)(completed_us - state->completed_us) <= 0) {
            state->rejected_frames++;
            return 0U;
        }
        if (delta > 1U) state->skipped_scans += delta - 1U;
    }
    reset = (uint8_t)(!state->initialized || delta > 1U ||
                     (uint32_t)(completed_us - state->completed_us) > IR_WALL_RESET_GAP_US);
    if (reset && state->initialized) state->reset_count++;
    for (i = 0U; i < IR_WALL_CHANNELS; ++i) {
        state->raw[i] = raw_adc[i];
        if (reset) state->ema[i] = (float)raw_adc[i];
        else state->ema[i] += IR_WALL_EMA_ALPHA *
                               ((float)raw_adc[i] - state->ema[i]);
        state->rounded[i] = RoundedAdc(state->ema[i]);
    }
    state->scan_count = scan_count;
    state->completed_us = completed_us;
    state->initialized = 1U;
    if (reset) state->warmup_count = 1U;
    else if (state->warmup_count < IR_WALL_WARMUP_SAMPLES)
        state->warmup_count++;
    return 1U;
}

IR_WallStatus IR_WallSignal_Read(const IR_WallSignal *state,
                                 uint32_t now_us,
                                 IR_WallSnapshot *out)
{
    uint8_t i;
    IR_WallStatus result;
    if (!out) return IR_WALL_NO_SAMPLE;
    memset(out, 0, sizeof(*out));
    if (!state || !state->initialized) {
        out->status = IR_WALL_NO_SAMPLE;
        return out->status;
    }
    for (i = 0U; i < IR_WALL_CHANNELS; ++i) {
        out->raw_adc[i] = state->raw[i];
        out->ema10_adc[i] = state->rounded[i];
    }
    out->scan_count = state->scan_count;
    out->completed_us = state->completed_us;
    out->age_us = (uint32_t)(now_us - state->completed_us);
    out->warmup_count = state->warmup_count;
    out->skipped_scans = state->skipped_scans;
    out->reset_count = state->reset_count;
    out->rejected_frames = state->rejected_frames;
    if (out->age_us > IR_WALL_MAX_AGE_US) result = IR_WALL_STALE;
    else if (out->warmup_count < IR_WALL_WARMUP_SAMPLES) result = IR_WALL_WARMUP;
    else result = IR_WALL_VALID;
    out->status = result;
    return result;
}
