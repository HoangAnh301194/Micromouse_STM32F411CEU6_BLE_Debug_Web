#include "motor_trace.h"
#include "motor.h"
#include "motor_test.h"
#include "encoder.h"
#include "bt_debug.h"
#include "stm32f4xx.h"

typedef struct {
    int16_t cmd_l;
    int16_t cmd_r;
    int16_t applied_l;
    int16_t applied_r;
    int32_t enc_l;
    int32_t enc_r;
} MotorTraceSample_t;

static MotorTraceSample_t samples[MOTOR_TRACE_CAPACITY];
static volatile uint16_t head = 0;  /* next write index */
static volatile uint16_t count = 0; /* up to capacity */
static uint16_t dump_count = 0;
static uint16_t dump_cursor = 0;
static uint16_t dump_start = 0;
static uint8_t dumping = 0;
static uint32_t last_send_ms = 0;

void MotorTrace_Reset(void)
{
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    head = 0;
    count = 0;
    dumping = 0;
    if (!x) __enable_irq();
}
void MotorTrace_Record1ms(void)
{
    MotorTest_Snapshot_t s;
    int16_t l, r;
    MotorTraceSample_t *p;
    if (!MotorTest_IsRunning()) return;

    MotorTest_GetSnapshot(&s);
    Motor_GetAppliedPairPermille(&l, &r);
    p = &samples[head];
    p->cmd_l = s.pwm_left;
    p->cmd_r = s.pwm_right;
    p->applied_l = l;
    p->applied_r = r;
    p->enc_l = Encoder_GetLeftCount();
    p->enc_r = Encoder_GetRightCount();
    head = (uint16_t)((head + 1U) % MOTOR_TRACE_CAPACITY);
    if (count < MOTOR_TRACE_CAPACITY) count++;
}
void MotorTrace_RequestDump(void)
{
    uint32_t x = __get_PRIMASK();
    __disable_irq();
    dump_count = count;
    dump_cursor = 0;
    dump_start = (uint16_t)((head + MOTOR_TRACE_CAPACITY - count) % MOTOR_TRACE_CAPACITY);
    last_send_ms = 0;
    dumping = 1;
    if (!x) __enable_irq();
}
void MotorTrace_Poll(uint32_t now_ms)
{
    const MotorTraceSample_t *s;
    uint16_t idx;
    if (!dumping) return;
    if (last_send_ms != 0 && (uint32_t)(now_ms - last_send_ms) < 20U) return;
    last_send_ms = now_ms;
    if (dump_cursor >= dump_count) {
        BT_SendString("MTTRACEEND\r\n");
        dumping = 0;
        return;
    }
    idx = (uint16_t)((dump_start + dump_cursor) % MOTOR_TRACE_CAPACITY);
    s = &samples[idx];
    BT_Printf("MTTRACE,%u,%d,%d,%d,%d,%ld,%ld\r\n",
              (unsigned)dump_cursor,
              (int)s->cmd_l, (int)s->cmd_r,
              (int)s->applied_l, (int)s->applied_r,
              (long)s->enc_l, (long)s->enc_r);
    dump_cursor++;
}
