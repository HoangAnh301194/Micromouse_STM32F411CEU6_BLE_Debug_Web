/* Standalone motor-LOCKED six-channel IR acquisition and filter A/B.
 * IR driver runs its existing timed LED OFF + 3 paired ON ADC DMA cycles.
 * 1kHz-ish completed scans update filters; UART logs are DECIMATED
 * to 40Hz, with OFF/ON readings at 5Hz and health at 1Hz.
 *
 * No OLED, BLE, MPU I2C or motion ISR in this firmware environment.
 * Motor outputs are locked before IR is initialized. VM should be
 * disconnected physically for the measurement campaign.
 */
#if defined(IR_FILTER_DIAG)
#include "board.h"
#include "ir_filter.h"
#include "ir_sensor.h"
#include "motor.h"
#include "system_timer.h"
#include "uart.h"
#include <stdint.h>

#define IRF_LOG_PERIOD_MS 25U
#define IRF_ONOFF_PERIOD_MS 200U
#define IRF_HEALTH_PERIOD_MS 1000U

static void LogFilter(uint32_t ms, const IR_Data_t *ir,
                      const IR_FilterFrame_t *f)
{
    UART_Printf("IRF,ms=%lu,n=%lu,us=%lu,"
                "d=%u|%u|%u|%u|%u|%u,"
                "med=%u|%u|%u|%u|%u|%u,"
                "e10=%u|%u|%u|%u|%u|%u,"
                "e25=%u|%u|%u|%u|%u|%u,"
                "hy=%u|%u|%u|%u|%u|%u\r\n",
                (unsigned long)ms,(unsigned long)ir->scan_count,
                (unsigned long)ir->completed_us,
                (unsigned)f->raw[0],(unsigned)f->raw[1],
                (unsigned)f->raw[2],(unsigned)f->raw[3],
                (unsigned)f->raw[4],(unsigned)f->raw[5],
                (unsigned)f->median3[0],(unsigned)f->median3[1],
                (unsigned)f->median3[2],(unsigned)f->median3[3],
                (unsigned)f->median3[4],(unsigned)f->median3[5],
                (unsigned)f->ema10[0],(unsigned)f->ema10[1],
                (unsigned)f->ema10[2],(unsigned)f->ema10[3],
                (unsigned)f->ema10[4],(unsigned)f->ema10[5],
                (unsigned)f->ema25[0],(unsigned)f->ema25[1],
                (unsigned)f->ema25[2],(unsigned)f->ema25[3],
                (unsigned)f->ema25[4],(unsigned)f->ema25[5],
                (unsigned)f->median_ema25[0],(unsigned)f->median_ema25[1],
                (unsigned)f->median_ema25[2],(unsigned)f->median_ema25[3],
                (unsigned)f->median_ema25[4],(unsigned)f->median_ema25[5]);
}
static void LogOnOff(uint32_t ms,const IR_Data_t *ir,
                     const IR_Diagnostics_t *d)
{
    UART_Printf("IRB,ms=%lu,n=%lu,"
                "off=%u|%u|%u|%u|%u|%u,"
                "on=%u|%u|%u|%u|%u|%u\r\n",
                (unsigned long)ms,(unsigned long)ir->scan_count,
                (unsigned)d->ambient_raw[0],(unsigned)d->ambient_raw[1],
                (unsigned)d->ambient_raw[2],(unsigned)d->ambient_raw[3],
                (unsigned)d->ambient_raw[4],(unsigned)d->ambient_raw[5],
                (unsigned)d->emitter_raw[0],(unsigned)d->emitter_raw[1],
                (unsigned)d->emitter_raw[2],(unsigned)d->emitter_raw[3],
                (unsigned)d->emitter_raw[4],(unsigned)d->emitter_raw[5]);
}
int IR_FilterDiag_Main(void)
{
    IR_Filter_t filter;
    IR_FilterFrame_t filtered={{0}};
    IR_Data_t sample={{0},0U,0U};
    IR_Diagnostics_t diagnostics;
    uint32_t last_log=0U,last_raw=0U,last_health=0U;
    uint32_t last_complete_ms=0U,last_retry=0U;
    uint8_t have=0U;

    Board_Init();
    SystemTimer_Init();
    UART_Init();
    if(Board_GetClockFault()!=0U){
        UART_Printf("IR_FILTER,CLOCK_FAULT=%lu\r\n",
                    (unsigned long)Board_GetClockFault());
        Board_StatusLed(1U);
        while(1){__WFI();}
    }
    Motor_Init();
    Motor_LockOutputs();
    if(!Motor_OutputsLocked()){
        UART_SendString("IR_FILTER,FATAL,MOTOR_UNLOCKED\r\n");
        while(1){__WFI();}
    }
    IR_FilterReset(&filter);
    IR_Init();
    UART_SendString("IR_FILTER,HELLO,ir_filter_v1_motor_locked,channels=L90|L45|L0|R0|R45|R90,log_hz=40,unit=adc\r\n");
    IR_StartScan();
    last_complete_ms=last_log=last_raw=last_health=last_retry=millis();

    while(1){
        uint32_t now=millis();
        IR_Data_t next;
        if(IR_GetLatest(&next)){
            IR_GetDiagnostics(&diagnostics);
            if(diagnostics.raw_scan_count==next.scan_count){
                IR_FilterStep(&filter,next.value,&filtered);
                sample=next;have=1U;
                last_complete_ms=now;
                if((uint32_t)(now-last_log)>=IRF_LOG_PERIOD_MS){
                    last_log=now;
                    LogFilter(now,&sample,&filtered);
                }
                if((uint32_t)(now-last_raw)>=IRF_ONOFF_PERIOD_MS){
                    last_raw=now;
                    LogOnOff(now,&sample,&diagnostics);
                }
            }
            IR_StartScan();
        }
        /* Recovery if a DMA error leaves the state idle. The driver
         * ignores requests while busy; retry no faster than 10ms. */
        if((uint32_t)(now-last_complete_ms)>=10U &&
           (uint32_t)(now-last_retry)>=10U){
            last_retry=now;IR_StartScan();
        }
        if((uint32_t)(now-last_health)>=IRF_HEALTH_PERIOD_MS){
            last_health=now;
            IR_GetDiagnostics(&diagnostics);
            UART_Printf("IRH,ms=%lu,n=%lu,scans=%lu,fail=%lu,"
                        "dma_err=%lu,dma_to=%lu,ovr=%lu,drop=%lu,"
                        "locked=%u,age=%lu\r\n",
                        (unsigned long)now,(unsigned long)sample.scan_count,
                        (unsigned long)diagnostics.scans_completed,
                        (unsigned long)diagnostics.scans_failed,
                        (unsigned long)diagnostics.dma_errors,
                        (unsigned long)diagnostics.dma_timeouts,
                        (unsigned long)diagnostics.adc_overruns,
                        (unsigned long)UART_GetDroppedMessages(),
                        (unsigned)Motor_OutputsLocked(),
                        (unsigned long)(have?(now-last_complete_ms):0U));
        }
    }
}
#endif /* IR_FILTER_DIAG */
