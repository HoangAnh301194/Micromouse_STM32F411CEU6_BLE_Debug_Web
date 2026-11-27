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
static MPU6050_Handle_t *pivot_mpu;
static int16_t pivot_target;
static int32_t pivot_ref_pps;
#define CAL_PIVOT_SPEED_PPS 800
static float pivot_origin;
static uint16_t stall_l, stall_r;
/* Relative 90-degree turns must remain valid at +/-180 yaw wrap. */
static float RelativeYaw(float now, float origin)
{
    float d = now-origin;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

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
    result.pivot_target = pivot_target;
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
static uint8_t StartMotion(MPU6050_Handle_t *mpu, int16_t target, int32_t speed_pps)
{
    MPU6050_Snapshot_t snap = {0};
    if (target && (!mpu || !MPU6050_GetSnapshot(mpu,&snap) || !snap.valid ||
        (uint32_t)(micros()-snap.completed_us)>3000U || snap.gyro_z_dps > 2.0f || snap.gyro_z_dps < -2.0f)) return 0U;
    uint32_t irq;
    if (fault_latched || Board_GetClockFault() || !Control_IsRunning() ||
        Board_ButtonPressed()) return 0U;
    irq = Lock();
    if (phase != 0U || result_pending || !Motor_OneBoxUnlockOutputs()) {
        Unlock(irq);
        return 0U;
    }
    pivot_mpu = mpu; pivot_target = target;
    pivot_ref_pps = speed_pps;
    pivot_origin = target ? snap.yaw_deg : 0.0f;
    stall_l = stall_r = 0U;
    Motor_Stop();
    started_at_ms = millis();
    OneBoxCore_Start(&core, Encoder_GetLeftCount(), Encoder_GetRightCount());
    OneBoxSpeedPid_Reset(&pid_l); OneBoxSpeedPid_Reset(&pid_r);
    result_pending = 0U;
    phase = 1U;
    Unlock(irq);
    return 1U;
}
uint8_t EncoderOneBox_Start(void) { return StartMotion(0,0,ONEBOX_SPEED_PPS); }
uint8_t EncoderOneBox_StartPivot(MPU6050_Handle_t *mpu, int16_t target) {
    if (target!=90 && target!=-90 && target!=180 && target!=-180) return 0U;
    return StartMotion(mpu,target,ONEBOX_SPEED_PPS);
}
uint8_t EncoderOneBox_StartCalibrationPivot(MPU6050_Handle_t *mpu, int16_t target) {
    if (target!=90 && target!=-90) return 0U;
    return StartMotion(mpu,target,CAL_PIVOT_SPEED_PPS);
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
    if (pivot_target) {
        MPU6050_Snapshot_t snap;
        int32_t l=Encoder_GetLeftCount(), r=Encoder_GetRightCount();
        int sl=pivot_target>0?-1:1, sr=-sl;
        int32_t dl=(l-core.start_l)*sl, dr=(r-core.start_r)*sr;
        float sign=pivot_target>0?1.0f:-1.0f;
        core.ticks++; core.host_age_ticks++;
        core.delta_l=l-core.start_l; core.delta_r=r-core.start_r;
        stall_l = l==core.last_l ? stall_l+1U : 0U;
        stall_r = r==core.last_r ? stall_r+1U : 0U;
        core.last_l=l; core.last_r=r;
        if (core.host_age_ticks>=ONEBOX_LEASE_TICKS) { Finish(ONEBOX_REASON_HOST_LOST); return; }
        if (core.ticks>=ONEBOX_MAX_TICKS) { Finish(ONEBOX_REASON_TIMEOUT); return; }
        if (!MPU6050_GetSnapshot(pivot_mpu,&snap) || !snap.valid ||
            (uint32_t)(micros()-snap.completed_us)>3000U) { Finish(ONEBOX_REASON_IMU); return; }
        if (dl < -30 || dr < -30 || dl > 3000 || dr > 3000 ||
            (dl+dr>600 && (dl-dr>250 || dr-dl>250))) { Finish(ONEBOX_REASON_ENCODER); return; }
        if (stall_l>=ONEBOX_STALL_TICKS || stall_r>=ONEBOX_STALL_TICKS) { Finish(ONEBOX_REASON_STALL); return; }
        float angle=RelativeYaw(snap.yaw_deg,pivot_origin)*sign;
        if (angle < -5.0f) { Finish(ONEBOX_REASON_IMU); return; }
        if (angle >= pivot_target*sign) { Finish(ONEBOX_REASON_DONE); return; }
        int16_t pl=OneBoxSpeedPid_Step(&pid_l,&gains_l,pivot_ref_pps,sl*Encoder_GetLeftSpeedPPS());
        int16_t pr=OneBoxSpeedPid_Step(&pid_r,&gains_r,pivot_ref_pps,sr*Encoder_GetRightSpeedPPS());
        Motor_SetPairPermille(sl*pl,sr*pr);
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
    out->pivot_target = pivot_target;
    out->imu_valid = 0U; out->age_us=0U; out->yaw=out->angle=out->gz=0.0f;
    if (pivot_target) {
        MPU6050_Snapshot_t snap;
        int sl=pivot_target>0?-1:1;
        out->ref_l=phase==1U?sl*pivot_ref_pps:0;
        out->ref_r=-out->ref_l;
        if (MPU6050_GetSnapshot(pivot_mpu,&snap) && snap.valid) {
            out->age_us=micros()-snap.completed_us;
            if (out->age_us<=3000U) { out->imu_valid=1U; out->yaw=snap.yaw_deg; out->angle=RelativeYaw(snap.yaw_deg,pivot_origin); out->gz=snap.gyro_z_dps; }
        }
    }
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
