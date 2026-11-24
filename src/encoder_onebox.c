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
static volatile uint8_t phase; /* 0 idle, 1 driving, 2 braking */
static volatile uint8_t result_pending;
static uint8_t fault_latched; /* severe fault requires MCU reset to re-arm */
static uint8_t physical_stop_latched; /* suppress PA0 key event after abort */
static uint16_t brake_ticks;
static EncoderOneBoxResult result;

static uint32_t Lock(void)
{
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    return x;
}
static void Unlock(uint32_t x) { if (!x) __enable_irq(); }
static uint8_t IsSevere(uint8_t reason)
{
    return reason != ONEBOX_REASON_DONE &&
           reason != ONEBOX_REASON_USER_STOP;
}
static void Finish(uint8_t reason)
{
    if (phase != 1U) return;
    OneBoxCore_Stop(&core, reason);
    result.reason = reason;
    result.count_l = core.delta_l;
    result.count_r = core.delta_r;
    result.elapsed_ms = core.ticks;
    if (IsSevere(reason)) {
        fault_latched = 1U;
        Motor_LockOutputs(); /* fault latch: reboot needed to re-arm */
        phase = 0U;
    } else {
        Motor_Brake(); /* short brake 100ms, then coast */
        brake_ticks = 100U;
        phase = 2U;
    }
    result.motor_locked = Motor_OutputsLocked();
    result_pending = 1U; /* main emits exactly once, no USART in TIM11 */
}
void EncoderOneBox_Init(void)
{
    uint32_t irq = Lock();
    phase = 0U;
    result_pending = 0U;
    fault_latched = 0U;
    physical_stop_latched = 0U;
    brake_ticks = 0U;
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
    OneBoxCore_Start(&core, Encoder_GetLeftCount(), Encoder_GetRightCount());
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
        if (brake_ticks == 0U) {
            Motor_Stop();
            phase = 0U;
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
    if ((core.ticks % 10U) == 0U)
        Motor_SetPairPermille(cmd.pwm_l, cmd.pwm_r);
}
uint8_t EncoderOneBox_IsRunning(void)
{
    return phase == 1U;
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
