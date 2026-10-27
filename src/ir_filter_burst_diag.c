/* Motor-locked, PA0-triggered rolling IR trace. Exclusively diagnostic. */
#if defined(IR_BURST_DIAG)
#include "board.h"
#include "ir_filter.h"
#include "ir_sensor.h"
#include "motor.h"
#include "system_timer.h"
#include "uart.h"
#include <stdint.h>

/* 1024 * (8 + 60) = 69632 bytes of static SRAM, < 128KiB STM32F411.
 * Approx. 0.846s of preceding samples at observed 1210Hz.
 * NEVER print while sampling. Samples are frozen on PA0 press-release. */
#define BURST_CAPACITY 1024U
#define BUTTON_DEBOUNCE_MS 40U
#define BUTTON_LONG_MS 1500U
#define STATIC_DURATION_MS 30000U
#define STATIC_LOG_MS 25U
#define STATIC_ONOFF_MS 200U
#define PHASE_COUNT 8U
/* 1 NO_WALL / 2 SIDE_STATIC / 3 SIDE_EDGE / 4 SIDE_YAW
 * 5 SINGLE_WALL / 6 DIAGONAL_FRONT / 7 ROBUSTNESS / 8 LUT_HOLDOUT.
 * Only phase 1 and 2 record 30s static. Others record rolling burst.
 * No UART commands can advance phase or enable motor. */
#define UART_LINE_DRAIN_MS 25U
typedef struct {
    uint32_t us;
    uint32_t scan;
    IR_FilterFrame_t f;
} BurstSample;
static BurstSample ring[BURST_CAPACITY];
static uint16_t head, filled;
static uint32_t run_id;
static uint32_t last_healthy_ms;
static uint32_t last_retry_ms;
static uint8_t phase=1U;
static uint32_t trial=0U;
static uint8_t static_active=0U;
static uint32_t static_started,static_last_log,static_last_onoff,static_rows,static_drop_begin;

static void SayHello(void)
{
    UART_SendString("IB,HELLO,version=2,mode=MOTOR_LOCKED,button=PA0,led=PC13,channels=L90|L45|L0|R0|R45|R90,capacity=1024,phases=8,short=CAPTURE,long=NEXT_PHASE\r\n");
}
static void PutSample(const IR_Data_t *ir, const IR_FilterFrame_t *f)
{
    BurstSample *p=&ring[head];
    p->us=ir->completed_us;
    p->scan=ir->scan_count;
    p->f=*f;
    head=(uint16_t)((head+1U)%BURST_CAPACITY);
    if(filled<BURST_CAPACITY)filled++;
}
/* Debounced active-low button. Events generated on RELEASE only.
 * 1 short (<1500ms), 2 long (>=1500ms). No auto-repeat on hold. */
static uint8_t ButtonEvent(uint32_t now)
{
    static uint8_t initialized, raw_old, stable, armed;
    static uint32_t changed_ms, pressed_ms;
    uint8_t raw=Board_ButtonPressed();
    if(!initialized){
        initialized=1U;raw_old=stable=raw;changed_ms=now;armed=0U;
        return 0U;
    }
    if(raw!=raw_old){raw_old=raw;changed_ms=now;}
    if(raw!=stable && (uint32_t)(now-changed_ms)>=BUTTON_DEBOUNCE_MS){
        stable=raw;
        if(raw){pressed_ms=now;armed=1U;}
        else if(armed){
            armed=0U;
            return (uint8_t)(((uint32_t)(now-pressed_ms)>=BUTTON_LONG_MS)?2U:1U);
        }
    }
    return 0U;
}
/* N LED pulses every 4 seconds encode 1-based phase number.
 * Normal ready: PC13 active-low via Board_StatusLed. Busy: solid LED.
 * Fault: alternating every 100ms. */
static void RenderLed(uint32_t now,uint8_t busy,uint8_t fault)
{
    uint32_t t=now%4000U;
    uint8_t pulse=(uint8_t)((t/260U)<phase && (t%260U)<115U);
    Board_StatusLed(fault?(uint8_t)((now/100U)%2U):busy?1U:pulse);
}
static void LogPhase(void)
{
    UART_Printf("IB,PHASE,phase=%u,trial=%lu,locked=%u\r\n",
                (unsigned)phase,(unsigned long)trial,(unsigned)Motor_OutputsLocked());
}
static void StaticBegin(uint32_t now)
{
    static_active=1U;
    static_started=static_last_log=static_last_onoff=now;
    static_rows=0U;
    static_drop_begin=UART_GetDroppedMessages();
    ++trial;
    UART_Printf("IB,STATIC_BEGIN,id=%lu,phase=%u,duration_ms=%lu,locked=%u\r\n",
                (unsigned long)trial,(unsigned)phase,
                (unsigned long)STATIC_DURATION_MS,(unsigned)Motor_OutputsLocked());
}
static void StaticLog(uint32_t now,const IR_Data_t *ir,const IR_FilterFrame_t *f)
{
    UART_Printf("IBS,id=%lu,ms=%lu,scan=%lu,"
        "d=%u|%u|%u|%u|%u|%u,med=%u|%u|%u|%u|%u|%u,"
        "e10=%u|%u|%u|%u|%u|%u,e25=%u|%u|%u|%u|%u|%u,"
        "hy=%u|%u|%u|%u|%u|%u\r\n",
        (unsigned long)trial,(unsigned long)now,(unsigned long)ir->scan_count,
        (unsigned)f->raw[0],(unsigned)f->raw[1],(unsigned)f->raw[2],
        (unsigned)f->raw[3],(unsigned)f->raw[4],(unsigned)f->raw[5],
        (unsigned)f->median3[0],(unsigned)f->median3[1],(unsigned)f->median3[2],
        (unsigned)f->median3[3],(unsigned)f->median3[4],(unsigned)f->median3[5],
        (unsigned)f->ema10[0],(unsigned)f->ema10[1],(unsigned)f->ema10[2],
        (unsigned)f->ema10[3],(unsigned)f->ema10[4],(unsigned)f->ema10[5],
        (unsigned)f->ema25[0],(unsigned)f->ema25[1],(unsigned)f->ema25[2],
        (unsigned)f->ema25[3],(unsigned)f->ema25[4],(unsigned)f->ema25[5],
        (unsigned)f->median_ema25[0],(unsigned)f->median_ema25[1],
        (unsigned)f->median_ema25[2],(unsigned)f->median_ema25[3],
        (unsigned)f->median_ema25[4],(unsigned)f->median_ema25[5]);
    static_rows++;
}
static void StaticOnOff(uint32_t now,const IR_Data_t *ir,const IR_Diagnostics_t *d)
{
    UART_Printf("IBO,id=%lu,ms=%lu,scan=%lu,"
        "off=%u|%u|%u|%u|%u|%u,on=%u|%u|%u|%u|%u|%u\r\n",
        (unsigned long)trial,(unsigned long)now,(unsigned long)ir->scan_count,
        (unsigned)d->ambient_raw[0],(unsigned)d->ambient_raw[1],
        (unsigned)d->ambient_raw[2],(unsigned)d->ambient_raw[3],
        (unsigned)d->ambient_raw[4],(unsigned)d->ambient_raw[5],
        (unsigned)d->emitter_raw[0],(unsigned)d->emitter_raw[1],
        (unsigned)d->emitter_raw[2],(unsigned)d->emitter_raw[3],
        (unsigned)d->emitter_raw[4],(unsigned)d->emitter_raw[5]);
}
static void StaticEnd(void)
{
    static_active=0U;
    UART_Printf("IB,STATIC_END,id=%lu,phase=%u,rows=%lu,drop=%lu,locked=%u\r\n",
                (unsigned long)trial,(unsigned)phase,
                (unsigned long)static_rows,
                (unsigned long)(UART_GetDroppedMessages()-static_drop_begin),
                (unsigned)Motor_OutputsLocked());
}
static void PrintSample(uint32_t id,uint16_t seq,const BurstSample *s)
{
    const IR_FilterFrame_t *f=&s->f;
    UART_Printf("IBD,id=%lu,seq=%u,us=%lu,scan=%lu,"
        "d=%u|%u|%u|%u|%u|%u,med=%u|%u|%u|%u|%u|%u,"
        "e10=%u|%u|%u|%u|%u|%u,e25=%u|%u|%u|%u|%u|%u,"
        "hy=%u|%u|%u|%u|%u|%u\r\n",
        (unsigned long)id,(unsigned)seq,
        (unsigned long)s->us,(unsigned long)s->scan,
        (unsigned)f->raw[0],(unsigned)f->raw[1],(unsigned)f->raw[2],
        (unsigned)f->raw[3],(unsigned)f->raw[4],(unsigned)f->raw[5],
        (unsigned)f->median3[0],(unsigned)f->median3[1],(unsigned)f->median3[2],
        (unsigned)f->median3[3],(unsigned)f->median3[4],(unsigned)f->median3[5],
        (unsigned)f->ema10[0],(unsigned)f->ema10[1],(unsigned)f->ema10[2],
        (unsigned)f->ema10[3],(unsigned)f->ema10[4],(unsigned)f->ema10[5],
        (unsigned)f->ema25[0],(unsigned)f->ema25[1],(unsigned)f->ema25[2],
        (unsigned)f->ema25[3],(unsigned)f->ema25[4],(unsigned)f->ema25[5],
        (unsigned)f->median_ema25[0],(unsigned)f->median_ema25[1],
        (unsigned)f->median_ema25[2],(unsigned)f->median_ema25[3],
        (unsigned)f->median_ema25[4],(unsigned)f->median_ema25[5]);
}
/* Stops IR before transfer: UART can never starve the acquisition.
 * All motor outputs stay locked. Waits long enough for 115200 8N1. */
static void DumpFrozenTrace(void)
{
    uint16_t count=filled;
    uint16_t start=(uint16_t)((head+BURST_CAPACITY-count)%BURST_CAPACITY);
    uint16_t i;
    uint32_t drops_start=UART_GetDroppedMessages();
    uint32_t id=++run_id;
    IR_Diagnostics_t health;
    Motor_LockOutputs();
    IR_Stop();
    Board_StatusLed(1U);
    IR_GetDiagnostics(&health);
    UART_Printf("IB,BEGIN,id=%lu,count=%u,first_scan=%lu,last_scan=%lu,locked=%u,phase=%u,trial=%lu\r\n",
        (unsigned long)id,(unsigned)count,
        (unsigned long)ring[start].scan,
        (unsigned long)ring[(head+BURST_CAPACITY-1U)%BURST_CAPACITY].scan,
        (unsigned)Motor_OutputsLocked(),(unsigned)phase,(unsigned long)trial);
    delay_ms_blocking(60U);
    for(i=0;i<count;i++){
        uint16_t index=(uint16_t)((start+i)%BURST_CAPACITY);
        PrintSample(id,i,&ring[index]);
        delay_ms_blocking(UART_LINE_DRAIN_MS);
    }
    delay_ms_blocking(70U);
    UART_Printf("IB,END,id=%lu,count=%u,uart_drop_delta=%lu,dma_err=%lu,dma_to=%lu,ovr=%lu,fail=%lu,locked=%u\r\n",
        (unsigned long)id,(unsigned)count,
        (unsigned long)(UART_GetDroppedMessages()-drops_start),
        (unsigned long)health.dma_errors,(unsigned long)health.dma_timeouts,
        (unsigned long)health.adc_overruns,(unsigned long)health.scans_failed,
        (unsigned)Motor_OutputsLocked());
    delay_ms_blocking(120U);
    /* New acquisition session: avoid using stale pre-dump filter states. */
    head=filled=0U;
    IR_StartScan();
    last_healthy_ms=last_retry_ms=millis();
    Board_StatusLed(0U);
}
int IR_BurstDiag_Main(void)
{
    IR_Filter_t filter;
    IR_FilterFrame_t f={{0}};
    IR_Data_t ir={{0},0U,0U};
    IR_Diagnostics_t diag;
    uint32_t led_ms=0U,now;
    Board_Init();
    SystemTimer_Init();
    UART_Init();
    Motor_Init();
    Motor_LockOutputs();
    if(Board_GetClockFault()!=0U || !Motor_OutputsLocked()){
        UART_SendString("IB,FATAL,CLOCK_OR_MOTOR_LOCK\r\n");
        while(1){Board_StatusLed((uint8_t)((millis()/100U)%2U));}
    }
    IR_FilterReset(&filter);
    IR_Init();
    SayHello();
    LogPhase();
    IR_StartScan();
    last_healthy_ms=last_retry_ms=millis();
    for(;;){
        now=millis();
        if(IR_GetLatest(&ir)){
            IR_GetDiagnostics(&diag);
            if(diag.raw_scan_count==ir.scan_count){
                IR_FilterStep(&filter,ir.value,&f);
                PutSample(&ir,&f);
                last_healthy_ms=now;
                if(static_active){
                    if((uint32_t)(now-static_last_log)>=STATIC_LOG_MS){
                        static_last_log=now;StaticLog(now,&ir,&f);
                    }
                    if((uint32_t)(now-static_last_onoff)>=STATIC_ONOFF_MS){
                        static_last_onoff=now;StaticOnOff(now,&ir,&diag);
                    }
                }
            }
            IR_StartScan();
        }
        if((uint32_t)(now-last_healthy_ms)>=10U &&
           (uint32_t)(now-last_retry_ms)>=10U){
            last_retry_ms=now;
            IR_StartScan();
        }
        if((uint32_t)(now-led_ms)>=1000U){
            led_ms=now;
            SayHello();
            UART_Printf("IB,READY,ms=%lu,count=%u,scan=%lu,locked=%u,drop=%lu,phase=%u,trial=%lu,busy=%u\r\n",
                (unsigned long)now,(unsigned)filled,
                (unsigned long)ir.scan_count,(unsigned)Motor_OutputsLocked(),
                (unsigned long)UART_GetDroppedMessages(),
                (unsigned)phase,(unsigned long)trial,(unsigned)static_active);
        }
        if(static_active && (uint32_t)(now-static_started)>=STATIC_DURATION_MS){
            StaticEnd();
        }
        RenderLed(now,static_active,(uint8_t)(
            (uint32_t)(now-last_healthy_ms)>=100U || !Motor_OutputsLocked()));
        {
            uint8_t button=ButtonEvent(now);
            if(button && !static_active && Motor_OutputsLocked()){
                if(button==2U){
                    phase=(uint8_t)(phase%PHASE_COUNT+1U);
                    head=filled=0U;
                    IR_FilterReset(&filter);
                    LogPhase();
                }else if(phase<=2U){
                    StaticBegin(now);
                }else if(filled>=128U){
                    ++trial;
                    DumpFrozenTrace();
                    IR_FilterReset(&filter);
                }else{
                    UART_SendString("IB,REJECT,reason=NOT_READY\r\n");
                }
            }
        }
    }
}
#endif
