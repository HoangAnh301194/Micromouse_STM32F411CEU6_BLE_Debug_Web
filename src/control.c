#include "control.h"
#if defined(CONTROL_LOAD_DIAG)
#include "pid.h"
#endif

#include "board.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor_test.h"
#if defined(MOTOR_STARTUP_BENCH)
#include "motor_startup.h"
#endif
#include "motor_trace.h"
#if defined(MOTOR_AUTO_BENCH)
#include "motor_functional_trace.h"
#endif
#include "motor.h"
#include "stm32f4xx.h"
#include "system_timer.h"

#if defined(CONTROL_LOAD_DIAG)
static volatile uint8_t load_stage = 0U;
static volatile uint32_t load_ticks = 0U;
static volatile uint32_t load_max_cycles = 0U;
static volatile uint32_t load_over_800us = 0U;
static volatile uint32_t load_over_1000us = 0U;
static volatile int32_t load_pid_milli = 0;
static PID_Handle_t load_pid_left, load_pid_right;
void Control_SetLoadStage(uint8_t stage)
{
    uint32_t primask = __get_PRIMASK();
    if (stage > 2U) return;
    __disable_irq();
    load_stage = stage;
    load_ticks = load_max_cycles = load_over_800us = load_over_1000us = 0U;
    load_pid_milli = 0;
    PID_Reset(&load_pid_left);
    PID_Reset(&load_pid_right);
    if (!primask) __enable_irq();
}
void Control_GetLoadStats(Control_LoadStats_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->stage = load_stage;
    out->ticks = load_ticks;
    out->max_cycles = load_max_cycles;
    out->over_800us = load_over_800us;
    out->over_1000us = load_over_1000us;
    out->pid_output_milli = (uint32_t)load_pid_milli;
    if (!primask) __enable_irq();
}
#endif
#if defined(MPU_INTEGRATION_BENCH)
static volatile uint32_t integ_ticks = 0U;
static volatile uint32_t integ_reused = 0U;
static volatile uint32_t integ_age_over1500 = 0U;
static volatile uint32_t integ_age_max = 0U;
static volatile uint32_t integ_missing = 0U;
static volatile uint32_t integ_last_sequence = 0U;

void Control_GetIntegrationStats(Control_IntegrationStats_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->ticks = integ_ticks;
    out->reused = integ_reused;
    out->age_over1500 = integ_age_over1500;
    out->age_max_us = integ_age_max;
    out->missing_or_expired = integ_missing;
    if (!primask) __enable_irq();
}
#endif
#if defined(MPU_PHASE_AB) && defined(MPU_INTEGRATION_BENCH)
static volatile uint32_t phase_ticks = 0U;
static volatile uint32_t phase_irq_reused = 0U;
static volatile uint32_t phase_published_reused = 0U;
static volatile uint32_t phase_irq_ahead_ticks = 0U;
static volatile uint32_t phase_contention = 0U;
static volatile uint32_t phase_irq_age_max = 0U;
static volatile uint32_t phase_main_age_max = 0U;
static volatile uint32_t phase_last_irq_seq = 0U;
static volatile uint32_t phase_last_published_seq = 0U;
static uint8_t phase_armed = 0U;

void Control_GetPhasePair(Control_PhasePair_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->paired_ticks = phase_ticks;
    out->irq_complete_reused = phase_irq_reused;
    out->snapshot_published_reused = phase_published_reused;
    out->irq_ahead_of_snapshot_ticks = phase_irq_ahead_ticks;
    out->trace_contention_ticks = phase_contention;
    out->irq_completion_age_max_us = phase_irq_age_max;
    out->main_observed_age_max_us = phase_main_age_max;
    out->last_irq_sequence = phase_last_irq_seq;
    out->last_published_sequence = phase_last_published_seq;
    if (!primask) __enable_irq();
}
#endif
static MPU6050_Handle_t *control_mpu = 0;
static volatile uint8_t control_running = 0;
static volatile uint8_t control_diagnostic_only = 0;
static volatile uint32_t control_tick_count = 0;
static volatile IR_Data_t ir_latest;

/* TIM11 timing: TIM5 runs while CPU is in WFI; CYCCNT measures CPU cycles.
 * Values are observed maxima/minima, NOT a proven formal WCET bound. */
static volatile uint32_t timing_last_entry_us = 0U;
static volatile uint32_t timing_periods = 0U;
static volatile uint32_t timing_min_us = 0xFFFFFFFFUL;
static volatile uint32_t timing_max_us = 0U;
static volatile uint32_t timing_late_50us = 0U;
static volatile uint32_t timing_gap_1500us = 0U;
static volatile uint32_t timing_max_cycles = 0U;
#if defined(MPU_FRESHNESS_DIAG)
/* Statistics only. Main produces a sample timestamp; ISR observes it.
 * Do not acquire I2C or run a motor callback here. */
static volatile uint32_t sync_ticks = 0U;
static volatile uint32_t sync_reused_ticks = 0U;
static volatile uint32_t sync_age_over1000 = 0U;
static volatile uint32_t sync_age_over1500 = 0U;
static volatile uint32_t sync_age_max_us = 0U;
static uint32_t sync_last_sequence = 0U;
static volatile uint32_t sync_age_hist[8] = {0U};
static uint32_t Control_AgeBinUpperBound(uint32_t bin)
{
    /* [0..249] ... [1500..1749], final bin is >=1750. */
    return (bin < 7U) ? ((bin + 1U) * 250U) : 1750U;
}
static uint32_t Control_AgeQuantileBound(uint32_t total, uint32_t pct)
{
    uint32_t need = (total * pct + 99U) / 100U;
    uint32_t acc = 0U;
    uint32_t idx;
    if (total == 0U) return 0U;
    for (idx = 0U; idx < 8U; ++idx) {
        acc += sync_age_hist[idx];
        if (acc >= need) return Control_AgeBinUpperBound(idx);
    }
    return 1750U;
}
#endif

static void Control_TimingFinish(uint32_t start_cycles)
{
    uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > timing_max_cycles) timing_max_cycles = cycles;
}

#if defined(MPU_FRESHNESS_DIAG)
void Control_GetImuSync(Control_ImuSync_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->control_ticks = sync_ticks;
    out->reused_sample_ticks = sync_reused_ticks;
    out->age_over1000_ticks = sync_age_over1000;
    out->age_over1500_ticks = sync_age_over1500;
    out->age_max_us = sync_age_max_us;
    out->age_p95_bin_us = Control_AgeQuantileBound(sync_ticks, 95U);
    out->age_p99_bin_us = Control_AgeQuantileBound(sync_ticks, 99U);
    if (primask == 0U) __enable_irq();
}
#endif

void Control_GetTiming(Control_Timing_t *out)
{
    uint32_t primask;
    if (!out) return;
    primask = __get_PRIMASK();
    __disable_irq();
    out->periods = timing_periods;
    out->period_min_us = timing_periods ? timing_min_us : 0U;
    out->period_max_us = timing_max_us;
    out->late_over_50us = timing_late_50us;
    out->gap_over_1500us = timing_gap_1500us;
    out->exec_max_cycles = timing_max_cycles;
    out->core_hz = SystemCoreClock;
    if (!primask) __enable_irq();
}


void Control_Init(MPU6050_Handle_t *mpu_handle)
{
    uint32_t timer_clock;
    uint32_t prescaler;
    uint32_t ticks_per_period;

#if defined(CONTROL_LOAD_DIAG)
    PID_Init(&load_pid_left, 0.1f, 0.01f, 0.0f, 1000.0f, 100.0f);
    PID_Init(&load_pid_right, 0.1f, 0.01f, 0.0f, 1000.0f, 100.0f);
    Control_SetLoadStage(0U);
#endif
    control_mpu = mpu_handle;
#if defined(MPU_INTEGRATION_BENCH)
    integ_ticks = integ_reused = integ_age_over1500 = 0U;
    integ_age_max = integ_missing = integ_last_sequence = 0U;
#endif
#if defined(MPU_PHASE_AB) && defined(MPU_INTEGRATION_BENCH)
    phase_ticks = phase_irq_reused = phase_published_reused = 0U;
    phase_irq_ahead_ticks = phase_contention = 0U;
    phase_irq_age_max = phase_main_age_max = 0U;
    phase_last_irq_seq = phase_last_published_seq = 0U;
    phase_armed = 0U;
#endif
    control_running = 0;
    control_tick_count = 0;
    timing_last_entry_us = 0U;
    timing_periods = 0U;
    timing_min_us = 0xFFFFFFFFUL;
    timing_max_us = 0U;
    timing_late_50us = 0U;
    timing_gap_1500us = 0U;
    timing_max_cycles = 0U;
#if defined(MPU_FRESHNESS_DIAG)
    sync_ticks = 0U;
    sync_reused_ticks = 0U;
    sync_age_over1000 = 0U;
    sync_age_over1500 = 0U;
    sync_age_max_us = 0U;
    sync_last_sequence = 0U;
    for (uint32_t idx = 0U; idx < 8U; ++idx)
        sync_age_hist[idx] = 0U;
#endif

    /* DWT cycle counter for short ISR duration measurements. */
    SystemCoreClockUpdate();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    timer_clock = Board_GetAPB2TimerClockHz();

    /* Generate a 1 MHz timer base, then divide to the requested control rate. */
    prescaler = timer_clock / 1000000UL;
    if (prescaler == 0U) {
        prescaler = 1U;
    }

    ticks_per_period = 1000000UL / CONTROL_FREQUENCY_HZ;
    if (ticks_per_period == 0U) {
        ticks_per_period = 1U;
    }

    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    TIM11->CR1 = 0;
    TIM11->PSC = prescaler - 1U;
    TIM11->ARR = ticks_per_period - 1U;
    TIM11->CNT = 0;
    TIM11->EGR = TIM_EGR_UG;
    TIM11->SR = 0;
    TIM11->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void Control_SetDiagnosticOnly(uint8_t enable)
{
    control_diagnostic_only = enable ? 1U : 0U;
}

void Control_Start(void)
{
    /* Motor diagnostics and encoder timing remain available without IMU. */
    TIM11->CNT = 0;
    TIM11->SR &= ~TIM_SR_UIF;
    control_running = 1U;
    TIM11->CR1 |= TIM_CR1_CEN;
}

void Control_Stop(void)
{
    /*
     * Stopping the scheduler while a motion command is active must also
     * force the motor command to zero; otherwise the last PWM value could
     * remain latched after TIM11 stops.
     */
    MotorTest_Stop();
    Motion_Stop();
    TIM11->CR1 &= ~TIM_CR1_CEN;
    control_running = 0U;
}

uint8_t Control_IsRunning(void)
{
    return control_running;
}

uint32_t Control_GetTickCount(void)
{
    /* Atomic 32-bit read on Cortex-M4. Monotonic counter, wraps naturally. */
    return control_tick_count;
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    IR_Data_t sample;
    uint32_t start_cycles;
    uint32_t entry_us;
    uint32_t interval_us = 0U;

    if ((TIM11->SR & TIM_SR_UIF) == 0U) return;

    start_cycles = DWT->CYCCNT;
    entry_us = micros();
    TIM11->SR &= ~TIM_SR_UIF;

    if (!control_running) return;

    if (timing_last_entry_us != 0U) {
        interval_us = (uint32_t)(entry_us - timing_last_entry_us);
        timing_periods++;
        if (interval_us < timing_min_us) timing_min_us = interval_us;
        if (interval_us > timing_max_us) timing_max_us = interval_us;
        if (interval_us > 1050U || interval_us < 950U) timing_late_50us++;
        if (interval_us > 1500U) timing_gap_1500us++;
    }
    timing_last_entry_us = entry_us;
    control_tick_count++;
#if defined(MPU_TIM11_TICK_SCHED)
    /* Schedule a sample opportunity; I2C still runs solely in main. */
    MPU6050_NotifyControlTick(control_mpu, entry_us);
#endif
#if defined(MPU_FRESHNESS_DIAG)
    if (control_mpu && control_mpu->diag_control_ready &&
        control_mpu->diag_fresh_sequence != 0U) {
        uint32_t sequence = control_mpu->diag_fresh_sequence;
        uint32_t age_us = entry_us - control_mpu->diag_last_fresh_us;
        ++sync_ticks;
        if (sequence == sync_last_sequence) ++sync_reused_ticks;
        sync_last_sequence = sequence;
        if (age_us > 1000U) ++sync_age_over1000;
        if (age_us > 1500U) ++sync_age_over1500;
        if (age_us > sync_age_max_us) sync_age_max_us = age_us;
        ++sync_age_hist[(age_us >= 1750U) ? 7U : (age_us / 250U)];
    }
#endif
#if defined(MPU_INTEGRATION_BENCH)
    {
        MPU6050_Snapshot_t sample;
        if (MPU6050_GetSnapshot(control_mpu, &sample)) {
            uint32_t age_us = entry_us - sample.completed_us;
            ++integ_ticks;
            if (sample.sequence == integ_last_sequence) ++integ_reused;
            if (age_us > 1500U) ++integ_age_over1500;
            if (age_us > integ_age_max) integ_age_max = age_us;
            if (age_us > 3000U) ++integ_missing;
            integ_last_sequence = sample.sequence;
#if defined(MPU_PHASE_AB)
            {
                I2C_GyroCompletionTrace_t irq;
                if (!I2C_GetGyroCompletionTrace(&irq)) {
                    ++phase_contention; /* Never spin inside priority-0 ISR. */
                } else if (irq.sequence != 0U) {
                    uint32_t irq_age_us = entry_us - irq.completion_us;
                    uint32_t main_age_us = entry_us - sample.completed_us;
                    if (!phase_armed) {
                        /* Offset contains all gyro reads during calibration. */
                        phase_last_irq_seq = irq.sequence;
                        phase_last_published_seq = sample.sequence;
                        phase_armed = 1U;
                    } else {
                        ++phase_ticks;
                        if (irq.sequence == phase_last_irq_seq) ++phase_irq_reused;
                        if (sample.sequence == phase_last_published_seq)
                            ++phase_published_reused;
                        if (sample.source_irq_sequence != 0U &&
                            irq.sequence > sample.source_irq_sequence)
                            ++phase_irq_ahead_ticks;
                        phase_last_irq_seq = irq.sequence;
                        phase_last_published_seq = sample.sequence;
                    }
                    if (irq_age_us > phase_irq_age_max)
                        phase_irq_age_max = irq_age_us;
                    if (main_age_us > phase_main_age_max)
                        phase_main_age_max = main_age_us;
                }
            }
#endif
        } else {
            ++integ_missing;
        }
    }
#endif
#if defined(MOTOR_PWM_DIAG) || defined(MOTOR_AUTO_BENCH) || defined(MOTOR_STARTUP_BENCH)
    /* Opt-in physical driver bench: ONLY encoder + PWM + test watchdog.
     * Unlike Control_SetDiagnosticOnly(1), this path MUST service
     * Motor_Update1ms and MotorTest_Tick1ms, but never IR, IMU or Motion. */
    Encoder_Update(CONTROL_DT_S);
#if defined(MOTOR_STARTUP_HIGH_SPEED)
    {
        uint32_t motor_cycle_start = DWT->CYCCNT;
        Motor_Update1ms();
        MotorStartup_RecordCycles(DWT->CYCCNT - motor_cycle_start);
    }
#else
    Motor_Update1ms();
#endif
    MotorTest_Tick1ms();
#if defined(MOTOR_STARTUP_BENCH)
    MotorStartup_Tick1ms();
#endif
#if defined(MOTOR_AUTO_BENCH)
    MotorFunctionalTrace_Record1ms(entry_us, interval_us);
#endif
    Control_TimingFinish(start_cycles);
    return;
#endif
    if (control_diagnostic_only) {
#if defined(CONTROL_LOAD_DIAG)
        /* Benchmark computational load ONLY. Motor_Init is never called.
         * PID setpoint zero; outputs discarded, never written to TIM2/PWM.
         * Encoder GPIO/timers are initialized by dedicated main profile. */
        if (load_stage >= 1U) Encoder_Update(CONTROL_DT_S);
        if (load_stage >= 2U) {
            float l = PID_Compute(&load_pid_left, 0.0f,
                                  Encoder_GetLeftSpeedPPS(), CONTROL_DT_S);
            float r = PID_Compute(&load_pid_right, 0.0f,
                                  Encoder_GetRightSpeedPPS(), CONTROL_DT_S);
            load_pid_milli = (int32_t)((l + r) * 1000.0f);
        }
        {
            uint32_t cyc = DWT->CYCCNT - start_cycles;
            uint32_t ucycles = SystemCoreClock / 1000000UL;
            ++load_ticks;
            if (cyc > load_max_cycles) load_max_cycles = cyc;
            if (cyc > ucycles * 800UL) ++load_over_800us;
            if (cyc > ucycles * 1000UL) ++load_over_1000us;
        }
#endif
        Control_TimingFinish(start_cycles);
        return;
    }

    /*
     * Hard real-time path:
     * 1. Update encoder snapshot.
     * 2. Consume a coherent, completed MPU snapshot (no I2C in TIM11).
     * 3. Consume a completed IR scan and immediately schedule the next scan.
     * 4. Execute the motion controller using one coherent sensor snapshot.
     */
    Encoder_Update(CONTROL_DT_S);
    /* MPU6050_Update is executed only in main (I2C completion + recovery).
     * This ISR must NEVER launch/repair an I2C transaction. It only consumes
     * the most recently completed gyro/yaw snapshot in Motion_Update below. */

    if (IR_GetLatest(&sample)) {
        uint8_t i;

        for (i = 0; i < IR_SENSOR_COUNT; i++) {
            ir_latest.value[i] = sample.value[i];
        }
        ir_latest.scan_count = sample.scan_count;

        IR_StartScan();
    }

    Motor_Update1ms();
    MotorTrace_Record1ms();
    MotorTest_Tick1ms();
    if (MotorTest_IsRunning()) { Control_TimingFinish(start_cycles); return; }

    if (control_mpu != 0) {
        MPU6050_Snapshot_t imu_snapshot;
        if (MPU6050_GetSnapshot(control_mpu, &imu_snapshot) &&
            (uint32_t)(entry_us - imu_snapshot.completed_us) <= 3000U) {
            Motion_Update(CONTROL_DT_S,
                          Encoder_GetLeftCount(),
                          Encoder_GetRightCount(),
                          imu_snapshot.yaw_deg,
                          imu_snapshot.gyro_z_dps);
        } else {
            /* No valid/calibrated or sufficiently recent IMU data:
             * prevent motion from consuming an uninitialized/stale yaw. */
            Motion_Stop();
        }
    }
    Control_TimingFinish(start_cycles);
}
