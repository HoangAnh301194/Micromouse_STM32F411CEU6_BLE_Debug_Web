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

static void SayHello(void)
{
    UART_SendString("IB,HELLO,version=1,mode=MOTOR_LOCKED,button=PA0,led=PC13,channels=L90|L45|L0|R0|R45|R90,capacity=1024\r\n");
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
/* Fires on a debounced press followed by release; release at boot ignored. */
static uint8_t ButtonReleased(uint32_t now)
{
    static uint8_t initialized, raw_old, stable, armed;
    static uint32_t changed_ms;
    uint8_t raw=Board_ButtonPressed();
    if(!initialized){
        initialized=1U;raw_old=stable=raw;changed_ms=now;
        armed=0U;
        return 0U;
    }
    if(raw!=raw_old){raw_old=raw;changed_ms=now;}
    if(raw!=stable && (uint32_t)(now-changed_ms)>=BUTTON_DEBOUNCE_MS){
        stable=raw;
        if(raw){armed=1U;}
        else if(armed){armed=0U;return 1U;}
    }
    return 0U;
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
    UART_Printf("IB,BEGIN,id=%lu,count=%u,first_scan=%lu,last_scan=%lu,locked=%u\r\n",
        (unsigned long)id,(unsigned)count,
        (unsigned long)ring[start].scan,
        (unsigned long)ring[(head+BURST_CAPACITY-1U)%BURST_CAPACITY].scan,
        (unsigned)Motor_OutputsLocked());
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
            UART_Printf("IB,READY,ms=%lu,count=%u,scan=%lu,locked=%u,drop=%lu\r\n",
                (unsigned long)now,(unsigned)filled,
                (unsigned long)ir.scan_count,(unsigned)Motor_OutputsLocked(),
                (unsigned long)UART_GetDroppedMessages());
        }
        if((uint32_t)(now-last_healthy_ms)>=100U || !Motor_OutputsLocked()){
            Board_StatusLed((uint8_t)(((now/100U)%2U)==0U));
        }else{
            /* LED one short pulse each second: waiting for PA0. */
            Board_StatusLed((uint8_t)((now%1000U)<120U));
        }
        if(ButtonReleased(now)){
            if(filled>=128U && Motor_OutputsLocked()){
                DumpFrozenTrace();
                IR_FilterReset(&filter);
            }else{
                UART_SendString("IB,REJECT,reason=NOT_READY_OR_UNLOCKED\r\n");
            }
        }
    }
}
#endif
