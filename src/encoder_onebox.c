/* Unified menu: one-cell motion ONLY when Encoder / 1 Box Forward is
 * active AND the GUI sends Start. The control loop uses encoder feedback
 * only; it never reads MPU or IR. PA0 abort, host lease, direction,
 * divergence, stall, deadline and motor watchdog remain independent guards.
 */
#if defined(MODE_MENU_ONLY)
#include "encoder_onebox.h"
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "system_timer.h"
#include "stm32f4xx.h"

static OneBoxCore core;
static OneBoxGains gains_l = {ONEBOX_DEFAULT_P, ONEBOX_DEFAULT_I, ONEBOX_DEFAULT_D},
                   gains_r = {ONEBOX_DEFAULT_P, ONEBOX_DEFAULT_I, ONEBOX_DEFAULT_D};
static OneBoxSpeedPid pid_l, pid_r;
static volatile uint8_t phase; /* 0 idle, 1 driving, 2 braking */
static volatile uint8_t result_pending;
static uint8_t fault_latched; /* severe fault requires MCU reset to re-arm */
static uint8_t physical_stop_latched; /* suppress PA0 key event after abort */
static uint16_t brake_ticks;
static uint32_t started_at_ms;
static EncoderOneBoxResult result;

static uint32_t Lock(void)
{
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    return x;
}
static void Unlock(uint32_t x) { if (!x) __enable_irq(); }
uint8_t EncoderOneBox_SetGains(const OneBoxGains *left, const OneBoxGains *right)
{
    uint32_t irq;
    if (!OneBoxGains_Valid(left) || !OneBoxGains_Valid(right)) return 0U;
    irq = Lock();
    if (phase != 0U || !Motor_OutputsLocked()) { Unlock(irq); return 0U; }
    gains_l = *left; gains_r = *right;
    OneBoxSpeedPid_Reset(&pid_l); OneBoxSpeedPid_Reset(&pid_r);
    Unlock(irq);
    return 1U;
}
void EncoderOneBox_GetGains(OneBoxGains *left, OneBoxGains *right)
{
    uint32_t irq = Lock();
    *left = gains_l; *right = gains_r;
    Unlock(irq);
}
static uint8_t IsSevere(uint8_t reason)
{
    return reason != ONEBOX_REASON_DONE &&
           reason != ONEBOX_REASON_USER_STOP;
}
static void Finish(uint8_t reason)
{
    if (phase != 1U) return;
    OneBoxCore_Stop(&core, reason);
    OneBoxSpeedPid_Reset(&pid_l); OneBoxSpeedPid_Reset(&pid_r);
    result.reason = reason;
    result.count_l = core.delta_l;
    result.count_r = core.delta_r;
    result.elapsed_ms = core.ticks;
    if (IsSevere(reason)) {
        fault_latched = 1U;
        Motor_LockOutputs(); /* fault latch: reboot needed to re-arm */
        phase = 0U;
        result.motor_locked = 1U;
        result_pending = 1U; /* severe fault: report immediately */
    } else {
        Motor_Brake(); /* 100 ms bounded braking interval */
        brake_ticks = 100U;
        phase = 2U;
        result_pending = 0U; /* report only AFTER the driver re-locks */
    }
}
void EncoderOneBox_Init(void)
{
    uint32_t irq = Lock();
    phase = 0U;
    result_pending = 0U;
    fault_latched = 0U;
    physical_stop_latched = 0U;
    brake_ticks = 0U;
    started_at_ms = 0U;
    core.running = 0U;
    Unlock(irq);
}
uint8_t EncoderOneBox_Start(void)
{
    uint32_t irq;
    if (fault_latched || Board_GetClockFault() || !Control_IsRunning() ||
        Board_ButtonPressed()) return 0U;
    irq = Lock();
    if (phase != 0U || result_pending || !Motor_OneBoxUnlockOutputs()) {
        Unlock(irq);
        return 0U;
    }
    Motor_Stop();
    started_at_ms = millis();
    OneBoxCore_Start(&core, Encoder_GetLeftCount(), Encoder_GetRightCount());
    OneBoxSpeedPid_Reset(&pid_l); OneBoxSpeedPid_Reset(&pid_r);
    result_pending = 0U;
    phase = 1U;
    Unlock(irq);
    return 1U;
}
void EncoderOneBox_HostPing(void)
{
    uint32_t irq = Lock();
    if (phase == 1U) OneBoxCore_HostPing(&core);
    Unlock(irq);
}
void EncoderOneBox_Abort(void)
{
    uint32_t irq = Lock();
    if (phase == 1U) Finish(ONEBOX_REASON_USER_STOP);
    Unlock(irq);
}
void EncoderOneBox_ControlTick(void)
{
    OneBoxOutput cmd;
    if (phase == 2U) {
        if (brake_ticks > 0U) --brake_ticks;
        if (Board_ButtonPressed()) {
            physical_stop_latched = 1U;
            brake_ticks = 0U;
        }
        if (brake_ticks == 0U) {
            /* Report the FINAL lock state after braking, not the transient
             * unlocked braking phase. Physical PA0 can shorten braking. */
            Motor_LockOutputs();
            phase = 0U;
            result.motor_locked = 1U;
            result_pending = 1U;
        }
        return;
    }
    if (phase != 1U) return;
    if (Board_ButtonPressed()) {
        physical_stop_latched = 1U;
        Finish(ONEBOX_REASON_USER_STOP);
        return;
    }
    if (Motor_OutputsLocked()) {
        Finish(ONEBOX_REASON_MOTOR);
        return;
    }
    /* Encoder-only test: no IMU/IR samples consumed here. */
    cmd = OneBoxCore_Step(&core, Encoder_GetLeftCount(), Encoder_GetRightCount());
    if (cmd.reason != ONEBOX_REASON_NONE) {
        Finish(cmd.reason);
        return;
    }
    { /* TIM11: one speed update per 1 ms tick. */
        int16_t left = OneBoxSpeedPid_Step(&pid_l, &gains_l, core.reached_l ? 0.0f : ONEBOX_SPEED_PPS,
                                           Encoder_GetLeftSpeedPPS());
        int16_t right = OneBoxSpeedPid_Step(&pid_r, &gains_r, core.reached_r ? 0.0f : ONEBOX_SPEED_PPS,
                                            Encoder_GetRightSpeedPPS());
        Motor_SetPairPermille(left, right);
    }
}
uint8_t EncoderOneBox_IsRunning(void)
{
    return phase == 1U;
}
uint8_t EncoderOneBox_GetTelemetry(EncoderOneBoxTelemetry *out)
{
    uint32_t irq;
    if (!out) return 0U;
    irq = Lock();
    if (phase == 0U && !result_pending) {
        Unlock(irq);
        return 0U;
    }
    out->elapsed_ms = (uint32_t)(millis() - started_at_ms);
    /* These are cumulative counts, exactly like Encoder Log Receive.
     * Query the motor driver for APPLIED PWM (not a PID target). */
    out->left = Encoder_GetLeftCount();
    out->right = Encoder_GetRightCount();
    Motor_GetAppliedPairPermille(&out->pwm_l, &out->pwm_r);
    out->ref_l = phase == 1U && !core.reached_l ? ONEBOX_SPEED_PPS : 0;
    out->ref_r = phase == 1U && !core.reached_r ? ONEBOX_SPEED_PPS : 0;
    out->speed_l = (int32_t)Encoder_GetLeftSpeedPPS();
    out->speed_r = (int32_t)Encoder_GetRightSpeedPPS();
    Unlock(irq);
    return 1U;
}
uint8_t EncoderOneBox_SuppressMenuButton(void)
{
    if (!physical_stop_latched) return 0U;
    if (!Board_ButtonPressed()) physical_stop_latched = 0U;
    return 1U; /* suppress press plus final release event */
}
uint8_t EncoderOneBox_PopResult(EncoderOneBoxResult *out)
{
    uint32_t irq;
    if (!out) return 0U;
    irq = Lock();
    if (!result_pending) { Unlock(irq); return 0U; }
    *out = result;
    result_pending = 0U;
    Unlock(irq);
    return 1U;
}
const char *EncoderOneBox_Reason(uint8_t r)
{
    switch(r) {
    case ONEBOX_REASON_DONE: return "DONE";
    case ONEBOX_REASON_USER_STOP: return "STOP";
    case ONEBOX_REASON_TIMEOUT: return "TIMEOUT";
    case ONEBOX_REASON_STALL: return "STALL";
    case ONEBOX_REASON_ENCODER: return "ENCODER";
    case ONEBOX_REASON_IMU: return "IMU";
    case ONEBOX_REASON_MOTOR: return "MOTOR";
    case ONEBOX_REASON_HOST_LOST: return "HOST_LOST";
    default: return "UNKNOWN";
    }
}
#endif
