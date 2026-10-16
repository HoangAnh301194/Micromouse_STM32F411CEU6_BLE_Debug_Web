/* Deliberately opt-in 96MHz combined-system motor and shadow-off sweep.
 * Default production firmware has no command path into these functions.
 * PA10 'MODE OFF/ON' then explicit 'START'; no physical KEY input.
 * All profiles run the SAME 3x11 stage schedule and UART telemetry;
 * motor-off compiles output writes away and locks Motor Driver at boot.
 * This is a motor-on diagnostic only: NOT electrical safety qualification.
 */
#include "integration_sweep.h"
#if defined(INTEGRATION_DIAG)
#include "control.h"
#include "motor.h"
#include "board.h"
#include "system_timer.h"
#include "stm32f4xx.h"

#if defined(INTEGRATION_DIAG_MOTOR_OFF) && defined(INTEGRATION_DIAG_MOTOR_ON)
#error "Motor ON/OFF diagnostics cannot both be enabled"
#endif
#if !defined(INTEGRATION_DIAG_MOTOR_OFF) && !defined(INTEGRATION_DIAG_MOTOR_ON)
#error "INTEGRATION_DIAG needs an explicit MOTOR_OFF or MOTOR_ON mode"
#endif
#define SWEEP_STEP_DRIVE_MS 250U
#define SWEEP_STEP_COAST_MS 150U
#define SWEEP_STEP_TOTAL 33U
#define SWEEP_COMMAND_RENEW_MS 40U
#define SWEEP_HARD_LIMIT_MS 18000U
#define SWEEP_LINK_TIMEOUT_MS 750U
#define SWEEP_GYRO_MAX_AGE_US 3000U
#define SWEEP_TIM11_MAX_CYCLES 96000U

static volatile SweepSnapshot state;
static uint32_t since_ms, last_cmd_ms, session_ms, last_heartbeat_ms;
static uint32_t baseline_gap, baseline_timeout;

static void LockDone(uint8_t newstate, uint8_t reason)
{
    state.state = newstate;
    state.fault = reason;
    Motor_LockOutputs(); /* Sticky lock; cannot rearm until reset. */
}
void Sweep_Init(void)
{
    state = (SweepSnapshot){0};
    since_ms = last_cmd_ms = session_ms = last_heartbeat_ms = 0U;
    baseline_gap = baseline_timeout = 0U;
    /* Default is always OFF shadow. Motor output is not enabled until
     * exact UART command MODE ON, then START issued by the host. */
    state.motor_enabled = 0U;
}
uint8_t Sweep_SelectMode(uint8_t motor_on)
{
    if (state.state != SWEEP_DISARMED || state.armed_once || motor_on > 1U)
        return 0U;
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    if (motor_on) return 0U; /* compile-time motor-locked env */
#endif
    if (motor_on && Motor_OutputsLocked()) return 0U;
    state.motor_enabled = motor_on;
    return 1U;
}
uint8_t Sweep_IsActive(void)
{
    return state.state == SWEEP_DRIVE || state.state == SWEEP_COAST;
}
static uint8_t SensorFresh(const MPU6050_Handle_t *mpu)
{
    MPU6050_Snapshot_t sample = {0};
    if (!mpu || !mpu->calibrated || mpu->cal_state != 3U ||
        !MPU6050_GetSnapshot(mpu, &sample) || !sample.valid) return 0U;
    return ((uint32_t)(micros() - sample.completed_us) <= SWEEP_GYRO_MAX_AGE_US);
}
static void Command(uint8_t idx)
{
#if defined(INTEGRATION_DIAG_MOTOR_ON)
    if (state.motor_enabled) {
        /* 0,10,...100% on LEFT then RIGHT then BOTH.
         * No reverse, brake, PID or closed-loop compensation. */
        uint16_t duty = (uint16_t)((idx % 11U) * 100U);
        switch (idx / 11U) {
        case 0U: Motor_SetPairPermille((int16_t)duty, 0); break;
        case 1U: Motor_SetPairPermille(0, (int16_t)duty); break;
        default: Motor_SetPairPermille((int16_t)duty, (int16_t)duty); break;
        }
    } else {
        Motor_Stop(); /* No PWM demand in shadow; peripherals stay busy. */
    }
#else
    (void)idx; /* Motor outputs are locked in the safe shadow firmware. */
#endif
}
uint8_t Sweep_Arm(uint32_t now_ms, const MPU6050_Handle_t *mpu,
                  uint8_t host_authorized)
{
    Control_DebugStats_t stats;
    if (state.armed_once || state.state != SWEEP_DISARMED ||
        !SensorFresh(mpu) || Board_GetClockFault() != 0U)
        return 0U;
    /* START is an explicit, host-originated authorization, not a KEY press. */
    if (state.motor_enabled && !host_authorized) return 0U;
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    if (!Motor_OutputsLocked() || state.motor_enabled) return 0U;
#else
    if (Motor_OutputsLocked()) return 0U;
#endif
    Control_GetDebugStats(&stats);
    state.armed_once = 1U; /* One shot, even if terminated early. */
    state.fault = SWEEP_OK;
    state.stage_id = 0U;
    state.group = 0U;
    state.step = 0U;
    state.duty_permille = 0U;
    state.state = SWEEP_DRIVE;
    since_ms = last_cmd_ms = session_ms = last_heartbeat_ms = now_ms;
    baseline_gap = stats.gaps_gt1500us;
    baseline_timeout = mpu->timeout_reads;
    Command(0U);
    return 1U;
}
void Sweep_Heartbeat(uint32_t now_ms)
{
    if (Sweep_IsActive()) last_heartbeat_ms = now_ms;
}
void Sweep_Abort(uint8_t reason)
{
    if (Sweep_IsActive()) LockDone(SWEEP_FAULT, reason);
}
void Sweep_ImuFaultFromISR(void)
{
    if (Sweep_IsActive()) LockDone(SWEEP_FAULT, SWEEP_IMU_STALE);
}
void Sweep_Update(uint32_t now_ms, const MPU6050_Handle_t *mpu)
{
    Control_DebugStats_t stats;
    Motor_Diagnostics_t motor;
    uint32_t age;
    if (!Sweep_IsActive()) return;
    age = (uint32_t)(now_ms - session_ms);
    if (state.motor_enabled &&
        (uint32_t)(now_ms - last_heartbeat_ms) > SWEEP_LINK_TIMEOUT_MS) {
        Sweep_Abort(SWEEP_UART_LINK_LOST); return;
    }
    if (age > SWEEP_HARD_LIMIT_MS) {
        Sweep_Abort(SWEEP_SESSION_TIMEOUT); return;
    }
    if (Board_GetClockFault() || !SensorFresh(mpu)) {
        Sweep_Abort(SWEEP_IMU_STALE); return;
    }
    Control_GetDebugStats(&stats);
    Motor_GetDiagnostics(&motor);
    if (stats.gaps_gt1500us != baseline_gap ||
        stats.max_irq_cycles > SWEEP_TIM11_MAX_CYCLES) {
        Sweep_Abort(SWEEP_TIM11_DEADLINE); return;
    }
    if (mpu->timeout_reads != baseline_timeout) {
        Sweep_Abort(SWEEP_I2C_TIMEOUT); return;
    }
    if (motor.command_timeout_latched || (state.motor_enabled && motor.outputs_locked)) {
        Sweep_Abort(SWEEP_MOTOR_WATCHDOG); return;
    }
    state.elapsed_ms = age;
    if (state.state == SWEEP_DRIVE) {
        if ((uint32_t)(now_ms - since_ms) >= SWEEP_STEP_DRIVE_MS) {
            Motor_Stop();
            state.state = SWEEP_COAST;
            since_ms = now_ms;
        } else if ((uint32_t)(now_ms - last_cmd_ms) >= SWEEP_COMMAND_RENEW_MS) {
            last_cmd_ms = now_ms;
            Command((uint8_t)state.stage_id); /* real command renews lease */
        }
    } else if ((uint32_t)(now_ms - since_ms) >= SWEEP_STEP_COAST_MS) {
        if (++state.stage_id >= SWEEP_STEP_TOTAL) {
            LockDone(SWEEP_DONE, SWEEP_OK);
            return;
        }
        state.group = (uint8_t)(state.stage_id / 11U);
        state.step = (uint8_t)(state.stage_id % 11U);
        state.duty_permille = (uint16_t)state.step * 100U;
        since_ms = last_cmd_ms = now_ms;
        state.state = SWEEP_DRIVE;
        Command((uint8_t)state.stage_id);
    }
}
void Sweep_GetSnapshot(SweepSnapshot *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *out = state;
    if (!primask) __enable_irq();
}
#endif
