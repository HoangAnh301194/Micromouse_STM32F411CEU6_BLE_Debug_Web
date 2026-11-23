#include "encoder_onebox_core.h"

static int32_t Abs32(int32_t x) { return x < 0 ? -x : x; }
static int32_t Clamp32(int32_t v, int32_t min, int32_t max)
{
    return v < min ? min : (v > max ? max : v);
}

void OneBoxCore_Start(OneBoxCore *s, int32_t left, int32_t right)
{
    if (!s) return;
    s->start_l = s->last_l = left;
    s->start_r = s->last_r = right;
    s->delta_l = s->delta_r = 0;
    s->ticks = 0U;
    s->stale_ticks = 0U;
    s->host_age_ticks = 0U;
    s->reason = ONEBOX_REASON_NONE;
    s->running = 1U;
}
void OneBoxCore_HostPing(OneBoxCore *s)
{
    if (s && s->running) s->host_age_ticks = 0U;
}
void OneBoxCore_Stop(OneBoxCore *s, uint8_t reason)
{
    if (!s || !s->running) return;
    s->running = 0U;
    s->reason = reason;
}
OneBoxOutput OneBoxCore_Step(OneBoxCore *s, int32_t left, int32_t right)
{
    OneBoxOutput out = {0,0,ONEBOX_REASON_NONE};
    int32_t dl,dr,average,remaining,base,trim,imbalance;
    if (!s || !s->running) return out;

    s->ticks++;
    s->host_age_ticks++;
    dl = left - s->start_l;
    dr = right - s->start_r;
    s->delta_l = dl;
    s->delta_r = dr;

    if (left != s->last_l || right != s->last_r) {
        s->stale_ticks = 0U;
        s->last_l = left;
        s->last_r = right;
    } else if (s->stale_ticks < 65535U) {
        s->stale_ticks++;
    }

    if (s->host_age_ticks >= ONEBOX_LEASE_TICKS)
        out.reason = ONEBOX_REASON_HOST_LOST;
    else if (s->ticks >= ONEBOX_MAX_TICKS)
        out.reason = ONEBOX_REASON_TIMEOUT;
    else if (dl < -30L || dr < -30L)
        out.reason = ONEBOX_REASON_ENCODER;
    else if (dl > ONEBOX_TARGET_L + 170L || dr > ONEBOX_TARGET_R + 170L)
        out.reason = ONEBOX_REASON_ENCODER;
    else if (s->stale_ticks >= ONEBOX_STALL_TICKS)
        out.reason = ONEBOX_REASON_STALL;

    average = (dl + dr) / 2L;
    imbalance = ((dl * ONEBOX_TARGET_R) - (dr * ONEBOX_TARGET_L)) /
                ((ONEBOX_TARGET_L + ONEBOX_TARGET_R) / 2L);
    if (!out.reason && average > 300L &&
        Abs32(imbalance) > ONEBOX_DIVERGENCE_COUNT)
        out.reason = ONEBOX_REASON_ENCODER;

    /* Stop both wheels near one-cell distance when neither side is more
     * than 75 counts behind. A few counts of on-floor coast or wheel slip
     * remain uncharacterized: do not claim physical mm-level accuracy.
     */
    if (!out.reason && average >= (ONEBOX_TARGET_L + ONEBOX_TARGET_R) / 2L &&
        dl >= ONEBOX_TARGET_L - 75L &&
        dr >= ONEBOX_TARGET_R - 75L)
        out.reason = ONEBOX_REASON_DONE;

    if (out.reason) {
        OneBoxCore_Stop(s, out.reason);
        return out;
    }

    remaining = ((ONEBOX_TARGET_L + ONEBOX_TARGET_R) / 2L) - average;
    /* Taper from 26% toward 13% in the last ~800 counts (real vehicle
     * must validate minimum moving PWM; a stalled wheel times out).
     */
    base = Clamp32(remaining / 6L, ONEBOX_MIN_PWM, ONEBOX_MAX_PWM);
    trim = Clamp32(imbalance / 8L, -45L, 45L);
    out.pwm_l = (int16_t)Clamp32(base - trim, 0L, ONEBOX_MAX_PWM);
    out.pwm_r = (int16_t)Clamp32(base + trim, 0L, ONEBOX_MAX_PWM);
    if (dl >= ONEBOX_TARGET_L) out.pwm_l = 0;
    if (dr >= ONEBOX_TARGET_R) out.pwm_r = 0;
    return out;
}
