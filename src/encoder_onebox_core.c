#include "encoder_onebox_core.h"

static int32_t Abs32(int32_t x) { return x < 0 ? -x : x; }
static float ClampF(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
uint8_t OneBoxGains_Valid(const OneBoxGains *g)
{
    return g && g->p <= ONEBOX_PID_P_MAX && g->i <= ONEBOX_PID_I_MAX &&
           g->d <= ONEBOX_PID_D_MAX;
}
void OneBoxSpeedPid_Reset(OneBoxSpeedPid *s)
{
    *s = (OneBoxSpeedPid){0};
}
int16_t OneBoxSpeedPid_Step(OneBoxSpeedPid *s, const OneBoxGains *g,
                            float reference, float measured)
{
    float error, p, d, candidate, output, ki, ff;
    if (!OneBoxGains_Valid(g) || !(reference > 0.0f) ||
        !(measured >= -100000.0f && measured <= 100000.0f)) {
        OneBoxSpeedPid_Reset(s);
        return 0;
    }
    error = reference - measured;
    p = (float)g->p / ONEBOX_GAIN_SCALE * error;
    ki = (float)g->i / ONEBOX_GAIN_SCALE;
    ff = ONEBOX_KS_PM + ONEBOX_KV_PM_PER_PPS * reference;
    d = s->initialized ? -(float)g->d / ONEBOX_GAIN_SCALE *
             (measured - s->previous_speed) / ONEBOX_PID_DT_S : 0.0f;
    s->previous_speed = measured;
    s->initialized = 1U;
    candidate = ki > 0.0f ? ClampF(s->integral + error * ONEBOX_PID_DT_S,
                                 -ONEBOX_INTEGRAL_LIMIT, ONEBOX_INTEGRAL_LIMIT) : 0.0f;
    output = ff + p + ki * candidate + d;
    /* Conditional integration against the FINAL actuator saturation. */
    if (!((output > ONEBOX_MAX_PWM && error > 0.0f) ||
          (output < 0.0f && error < 0.0f))) s->integral = candidate;
    output = ff + p + ki * s->integral + d;
    return (int16_t)(ClampF(output, 0.0f, ONEBOX_MAX_PWM) + 0.5f);
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
    s->reached_l = s->reached_r = 0U;
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
    int32_t dl,dr,average,imbalance;
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

    /* Stop commanding each wheel independently at its target. No early
     * completion from an average or a 75-count shortfall. Latch arrival
     * to avoid restarting a wheel because of encoder bounce. */
    if (dl >= ONEBOX_TARGET_L) s->reached_l = 1U;
    if (dr >= ONEBOX_TARGET_R) s->reached_r = 1U;
    if (!out.reason && s->reached_l && s->reached_r)
        out.reason = ONEBOX_REASON_DONE;

    if (out.reason) {
        OneBoxCore_Stop(s, out.reason);
        return out;
    }

    /* Distance core owns only stop/fault decisions. Fixed nominal output
     * is a compatibility indicator; actual PWM is produced by wheel PID. */
    out.pwm_l = s->reached_l ? 0 : (int16_t)(ONEBOX_KS_PM + ONEBOX_KV_PM_PER_PPS * ONEBOX_SPEED_PPS + 0.5f);
    out.pwm_r = s->reached_r ? 0 : (int16_t)(ONEBOX_KS_PM + ONEBOX_KV_PM_PER_PPS * ONEBOX_SPEED_PPS + 0.5f);
    return out;
}
