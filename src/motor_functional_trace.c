#include "motor_functional_trace.h"
#if defined(MOTOR_AUTO_BENCH)

#include "encoder.h"
#include "motor.h"
#include "motor_test.h"
#include "pinout.h"
#include "uart.h"
#include "stm32f4xx.h"

/* 512 x 32B = 16 KiB maximum static SRAM, no UART work in TIM11 ISR.
 * Captured CCR values are CPU-visible preload registers, NOT the
 * physically-active timer compare latch or measured bridge waveforms. */
typedef struct {
    uint32_t us;
    int32_t enc_l;
    int32_t enc_r;
    uint16_t ccr_l, ccr_r;
    int16_t cmd_l, cmd_r;
    int16_t applied_l, applied_r;
    uint16_t interval_us;
    uint8_t pins;          /* l1 bit0, l2 bit1, r1 bit2, r2 bit3 */
    uint8_t state;         /* MotorTest_State_t */
    uint8_t reason;        /* MotorTest_EndReason_t */
    uint8_t reserved;
} MotorFunctionalSample_t;

enum { TRACE_IDLE = 0, TRACE_CAPTURING = 1, TRACE_READY = 2, TRACE_DUMPING = 3 };
static MotorFunctionalSample_t trace[MOTOR_FUNCTIONAL_TRACE_SAMPLES];
static volatile uint16_t capture_count;
static volatile uint8_t trace_state;
static uint16_t dump_cursor;
static uint32_t last_dump_ms;

void MotorFunctionalTrace_Init(void)
{
    trace_state = TRACE_IDLE;
    capture_count = 0U;
    dump_cursor = 0U;
    last_dump_ms = 0U;
}

uint8_t MotorFunctionalTrace_Begin(void)
{
    uint32_t saved = __get_PRIMASK();
    uint8_t accepted = 0U;
    __disable_irq();
    if (trace_state == TRACE_IDLE) {
        capture_count = 0U;
        dump_cursor = 0U;
        last_dump_ms = 0U;
        trace_state = TRACE_CAPTURING;
        accepted = 1U;
    }
    if (!saved) __enable_irq();
    return accepted;
}

void MotorFunctionalTrace_Record1ms(uint32_t entry_us, uint32_t interval_us)
{
    MotorFunctionalSample_t *p;
    MotorTest_Snapshot_t status;
    int16_t applied_l = 0, applied_r = 0;
    uint16_t index;
    uint32_t odr;
    if (trace_state != TRACE_CAPTURING) return;
    index = capture_count;
    if (index >= MOTOR_FUNCTIONAL_TRACE_SAMPLES) {
        trace_state = TRACE_READY;
        return;
    }
    p = &trace[index];
    MotorTest_GetSnapshot(&status);
    Motor_GetAppliedPairPermille(&applied_l, &applied_r);
    odr = GPIOB->ODR;
    p->us = entry_us;
    p->enc_l = Encoder_GetLeftCount();
    p->enc_r = Encoder_GetRightCount();
    p->ccr_l = (uint16_t)TIM2->CCR2;
    p->ccr_r = (uint16_t)TIM2->CCR1;
    p->cmd_l = status.pwm_left;
    p->cmd_r = status.pwm_right;
    p->applied_l = applied_l;
    p->applied_r = applied_r;
    p->interval_us = (uint16_t)(interval_us > 65535U ? 65535U : interval_us);
    p->pins = (uint8_t)(((odr >> MOTOR_L_IN1_PIN) & 1U) |
               (((odr >> MOTOR_L_IN2_PIN) & 1U) << 1) |
               (((odr >> MOTOR_R_IN1_PIN) & 1U) << 2) |
               (((odr >> MOTOR_R_IN2_PIN) & 1U) << 3));
    p->state = (uint8_t)status.state;
    p->reason = (uint8_t)status.reason;
    p->reserved = 0U;
    capture_count = (uint16_t)(index + 1U);
    if (capture_count == MOTOR_FUNCTIONAL_TRACE_SAMPLES) trace_state = TRACE_READY;
}

void MotorFunctionalTrace_Poll(uint32_t now_ms)
{
    const MotorFunctionalSample_t *p;
    if (trace_state == TRACE_READY) {
        trace_state = TRACE_DUMPING;
        dump_cursor = 0U;
        last_dump_ms = now_ms;
        UART_Printf("MFD_BEGIN,v=1,n=%u,arr=%lu,p0_locked=1,source=registers\r\n",
                    (unsigned)MOTOR_FUNCTIONAL_TRACE_SAMPLES,
                    (unsigned long)TIM2->ARR);
        return;
    }
    if (trace_state != TRACE_DUMPING) return;
    if ((uint32_t)(now_ms - last_dump_ms) < 25U) return;
    last_dump_ms = now_ms;
    if (dump_cursor >= MOTOR_FUNCTIONAL_TRACE_SAMPLES) {
        UART_Printf("MFD_END,n=%u,tx_drop=%lu\r\n",
                    (unsigned)MOTOR_FUNCTIONAL_TRACE_SAMPLES,
                    (unsigned long)UART_GetDroppedMessages());
        trace_state = TRACE_IDLE;
        return;
    }
    p = &trace[dump_cursor];
    UART_Printf("MFD,i=%u,us=%lu,dt=%u,cl=%u,cr=%u,al=%d,ar=%d,"
                "tl=%d,tr=%d,el=%ld,er=%ld,p=%u,s=%u,r=%u\r\n",
                (unsigned)dump_cursor,(unsigned long)p->us,
                (unsigned)p->interval_us,(unsigned)p->ccr_l,(unsigned)p->ccr_r,
                (int)p->applied_l,(int)p->applied_r,(int)p->cmd_l,(int)p->cmd_r,
                (long)p->enc_l,(long)p->enc_r,
                (unsigned)p->pins,(unsigned)p->state,(unsigned)p->reason);
    dump_cursor++;
}
#endif
