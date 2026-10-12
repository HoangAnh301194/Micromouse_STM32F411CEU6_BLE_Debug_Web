#include "control.h"
#if defined(CONTROL_LOAD_DIAG)
#include "pid.h"
#endif

#include "board.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor_test.h"
#include "motor_trace.h"
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
    uint32_t interval_us;

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
    if (control_diagnostic_only) {
#if defined(MOTOR_PWM_DIAG)
        /* Opt-in KEY-only, time-limited motor diagnostic. No sensor
         * scanning, motion logic, BLE or UART inside 1kHz TIM11 ISR.
         * MotorTest_Tick1ms enforces independent pulse timeout. */
        Encoder_Update(CONTROL_DT_S);
        Motor_Update1ms();
        MotorTest_Tick1ms();
#endif
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
     * 2. Advance MPU6050 DMA/update state.
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
        Motion_Update(CONTROL_DT_S,
                      Encoder_GetLeftCount(),
                      Encoder_GetRightCount(),
                      MPU6050_GetYaw(control_mpu),
                      MPU6050_GetGyroZ(control_mpu));
    }
    Control_TimingFinish(start_cycles);
}
