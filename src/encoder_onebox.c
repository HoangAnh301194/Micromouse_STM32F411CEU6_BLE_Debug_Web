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
static float pivot_previous_yaw, pivot_angle;
static uint16_t stall_l, stall_r;
static float motion_base, motion_ref_l, motion_ref_r;
static uint8_t motion_stage;
static float AbsF(float x) { return x<0.0f?-x:x; }
static void ApplyMotion(float desired, float limit, int32_t sync, int sl, int sr)
{
    float l,r;
    motion_base=Motion_Base(motion_base,desired,limit);
    motion_stage=desired<limit-1.0f && motion_base>=desired-1.0f ? MOTION_DEC :
                 motion_base<limit-1.0f ? MOTION_ACC : MOTION_CRUISE;
    Motion_Sync(motion_base,sync,&l,&r);
    motion_ref_l=sl*l; motion_ref_r=sr*r;
    Motor_SetPairPermille(sl*OneBoxSpeedPid_Step(&pid_l,&gains_l,l,sl*Encoder_GetLeftSpeedPPS()),
                          sr*OneBoxSpeedPid_Step(&pid_r,&gains_r,r,sr*Encoder_GetRightSpeedPPS()));
}
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
uint8_t EncoderOneBox_SetMotionConfig(const MotionConfig *config)
{
    if (!MotionConfig_Valid(config)) return 0U;
    uint32_t irq=Lock();
    if (phase!=0U || result_pending || !Motor_OutputsLocked()) { Unlock(irq); return 0U; }
    motion_config=*config;
    Unlock(irq);
    return 1U;
}
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
    motion_ref_l=motion_ref_r=0.0f;
    motion_stage=MOTION_SETTLE;
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
        motion_stage=MOTION_END;
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
    pivot_previous_yaw = target ? snap.yaw_deg : 0.0f;
    pivot_angle = 0.0f;
    motion_base=motion_ref_l=motion_ref_r=0.0f;
    motion_stage=MOTION_ACC;
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
    else if (phase==2U) { result.reason=ONEBOX_REASON_USER_STOP; brake_ticks=0U; }
    Unlock(irq);
}
void EncoderOneBox_ControlTick(void)
{
    OneBoxOutput cmd;
    if (phase == 2U) {
        if (brake_ticks > 0U) --brake_ticks;
        if (Board_ButtonPressed()) {
            physical_stop_latched = 1U;
            result.reason=ONEBOX_REASON_USER_STOP;
            brake_ticks = 0U;
        }
        if (brake_ticks == 0U) {
            /* Report the FINAL lock state after braking, not the transient
             * unlocked braking phase. Physical PA0 can shorten braking. */
            Motor_LockOutputs();
            result.count_l=Encoder_GetLeftCount()-core.start_l;
            result.count_r=Encoder_GetRightCount()-core.start_r;
            if (result.reason==ONEBOX_REASON_DONE) {
                uint8_t ok=AbsF(Encoder_GetLeftSpeedPPS())<=80.0f &&
                           AbsF(Encoder_GetRightSpeedPPS())<=80.0f;
                if (pivot_target) {
                    MPU6050_Snapshot_t snap;
                    ok=ok && MPU6050_GetSnapshot(pivot_mpu,&snap) && snap.valid &&
                        (uint32_t)(micros()-snap.completed_us)<=3000U &&
                        AbsF(snap.gyro_z_dps)<=MOTION_GYRO_TOL &&
                        AbsF(pivot_angle+RelativeYaw(snap.yaw_deg,pivot_previous_yaw)-pivot_target)<=MOTION_ANGLE_TOL;
                } else {
                    ok=ok && AbsF((float)(result.count_l-ONEBOX_TARGET_L))<=MOTION_COUNT_TOL &&
                        AbsF((float)(result.count_r-ONEBOX_TARGET_R))<=MOTION_COUNT_TOL;
                }
                if (!ok) result.reason=ONEBOX_REASON_TOLERANCE;
            }
            motion_stage=MOTION_END;
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
        pivot_angle += RelativeYaw(snap.yaw_deg, pivot_previous_yaw);
        pivot_previous_yaw = snap.yaw_deg;
        float angle = pivot_angle * sign;
        if (angle < -5.0f) { Finish(ONEBOX_REASON_IMU); return; }
        float remaining=pivot_target*sign-angle;
        /* Stop together before the endpoint; no reverse correction. */
        if (remaining<=0.35f) { Finish(ONEBOX_REASON_DONE); return; }
        float limit=pivot_ref_pps<((float)motion_config.cruise)?pivot_ref_pps:((float)motion_config.cruise);
        ApplyMotion(Motion_AngleDesired(remaining,sign*snap.gyro_z_dps),limit,dl-dr,sl,sr);
        return;
    }
    /* Encoder-only test: no IMU/IR samples consumed here. */
    cmd = OneBoxCore_Step(&core, Encoder_GetLeftCount(), Encoder_GetRightCount());
    if (cmd.reason != ONEBOX_REASON_NONE) {
        Finish(cmd.reason);
        return;
    }
    {
        float remaining=(float)ONEBOX_TARGET_L-0.5f*(core.delta_l+core.delta_r);
        ApplyMotion(Motion_DistanceDesired(remaining),((float)motion_config.cruise),
                    core.delta_l-core.delta_r,1,1);
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
    out->ref_l = (int32_t)motion_ref_l;
    out->ref_r = (int32_t)motion_ref_r;
    out->delta_l=out->left-core.start_l;
    out->delta_r=out->right-core.start_r;
    out->motion_stage=motion_stage;
    out->pivot_target = pivot_target;
    out->imu_valid = 0U; out->age_us=0U; out->yaw=out->angle=out->gz=0.0f;
    if (pivot_target) {
        MPU6050_Snapshot_t snap;
        /* References are the actual signed profile outputs above. */
        if (MPU6050_GetSnapshot(pivot_mpu,&snap) && snap.valid) {
            out->age_us=micros()-snap.completed_us;
            if (out->age_us<=3000U) { out->imu_valid=1U; out->yaw=snap.yaw_deg; out->angle=pivot_angle + RelativeYaw(snap.yaw_deg,pivot_previous_yaw); out->gz=snap.gyro_z_dps; }
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
    case ONEBOX_REASON_TOLERANCE: return "TOLERANCE";
    default: return "UNKNOWN";
    }
}
#endif
