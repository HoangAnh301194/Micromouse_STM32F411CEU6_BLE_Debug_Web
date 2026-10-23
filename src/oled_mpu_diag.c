/* A/B firmware: MPU 1 kHz + TIM11 + IR vs MPU with cooperative OLED.
 * USART1 PA9 TX, PA10 RX. '0'=baseline OFF; '1'=OLED ON; '?'=metric.
 * All motor outputs permanently disabled. Normal production is untouched. */
#if defined(OLED_MPU_DIAG) && defined(OLED_SHARED_I2C)
#include "board.h"
#include "control.h"
#include "encoder.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled_shared.h"
#include "ssd1306.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
static MPU6050_Handle_t mpu;
static uint32_t metric_seq;
static void Draw(void) {
    MPU6050_Snapshot_t snap;
    char line[24];
    uint8_t valid=MPU6050_GetSnapshot(&mpu,&snap);
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0U,0U);
    SSD1306_WriteString("MPU > OLED 5Hz",&Font_7x10,SSD1306_WHITE);
    SSD1306_SetCursor(0U,13U);
    (void)snprintf(line,sizeof(line),"GYRO:%lu",(unsigned long)mpu.good_reads);
    SSD1306_WriteString(line,&Font_7x10,SSD1306_WHITE);
    SSD1306_SetCursor(0U,26U);
    (void)snprintf(line,sizeof(line),"ERR:%lu",(unsigned long)mpu.error_reads);
    SSD1306_WriteString(line,&Font_7x10,SSD1306_WHITE);
    SSD1306_SetCursor(0U,39U);
    (void)snprintf(line,sizeof(line),"AGE:%lums",valid?
      (unsigned long)((micros()-snap.completed_us)/1000U):9999UL);
    SSD1306_WriteString(line,&Font_7x10,SSD1306_WHITE);
    SSD1306_SetCursor(0U,52U);
    SSD1306_WriteString("MOTOR LOCKED",&Font_7x10,SSD1306_WHITE);
}
static void Emit(void) {
    Control_DebugStats_t ctl;
    OLED_SharedStats oled;
    I2C_OledStats_t bus;
    MPU6050_Snapshot_t snap;
    uint32_t age;
    Control_GetDebugStats(&ctl);
    OLED_SharedGetStats(&oled);
    I2C_OledGetStats(&bus);
    age=MPU6050_GetSnapshot(&mpu,&snap)?
      (uint32_t)(micros()-snap.completed_us):999999UL;
    ++metric_seq;
    /* Split into three short frames; UART_Printf buffer is 256 bytes.
     * Host accepts only identical seq across OLED_C/G/O. */
    UART_Printf("OLED_C,seq=%lu,ms=%lu,on=%u,ready=%u,cal=%u,locked=%u,"
      "tick=%lu,late=%lu,gap=%lu,valid=%lu,invalid=%lu,uart_drop=%lu\r\n",
      (unsigned long)metric_seq,(unsigned long)millis(),
      (unsigned)OLED_SharedEnabled(),(unsigned)OLED_SharedReady(),
      (unsigned)mpu.calibrated,(unsigned)Motor_OutputsLocked(),
      (unsigned long)ctl.ticks,(unsigned long)ctl.late_gt50us,
      (unsigned long)ctl.gaps_gt1500us,
      (unsigned long)ctl.gyro_valid_ticks,
      (unsigned long)ctl.gyro_invalid_after_first_valid,
      (unsigned long)UART_GetDroppedMessages());
    UART_Printf("OLED_G,seq=%lu,good=%lu,err=%lu,to=%lu,mpubusy=%lu,age=%lu\r\n",
      (unsigned long)metric_seq,(unsigned long)mpu.good_reads,
      (unsigned long)mpu.error_reads,(unsigned long)mpu.timeout_reads,
      (unsigned long)mpu.busy_retries,(unsigned long)age);
    UART_Printf("OLED_O,seq=%lu,cmd=%lu,chunks=%lu,frames=%lu,bytes=%lu,"
      "skip=%lu,wait=%lu,oled_err=%lu,tx_start=%lu,tx_done=%lu,"
      "tx_err=%lu,tx_to=%lu,tx_max=%lu\r\n",
      (unsigned long)metric_seq,(unsigned long)oled.commands,
      (unsigned long)oled.chunks,(unsigned long)oled.frames,
      (unsigned long)oled.data_bytes,(unsigned long)oled.admission_skips,
      (unsigned long)oled.busy_skips,(unsigned long)oled.errors,
      (unsigned long)bus.started,(unsigned long)bus.completed,
      (unsigned long)bus.errors,(unsigned long)bus.timeouts,
      (unsigned long)bus.max_duration_us);
}
int OLED_MPU_DiagMain(void) {
    MPU6050_Status init;
    uint8_t oled_found,ch;
    uint32_t last_draw,last_log;
    Board_Init();SystemTimer_Init();UART_Init();
    if(Board_GetClockFault()) {
        UART_Printf("OLED_BOOT,CLOCK_FAIL,%lu\r\n",
         (unsigned long)Board_GetClockFault());
        while(1) {__WFI();}
    }
    Motor_Init();Motor_LockOutputs();Encoder_Init();IR_Init();
    /* Blocking display init only before MPU init, TIM11, acquisition. */
    I2C_Init(I2C_1,1U);
    oled_found=(SSD1306_Init()==SSD1306_OK)?1U:0U;
    init=MPU6050_Init(&mpu);
    if(init==MPU6050_OK) init=MPU6050_Calibrate(&mpu);
    if(init!=MPU6050_OK) {
        UART_Printf("OLED_BOOT,MPU_FAIL,status=%u,step=%u,i2c=%u\r\n",
         (unsigned)init,(unsigned)mpu.init_step,(unsigned)mpu.init_i2c_status);
        while(1) {__WFI();}
    }
    OLED_SharedInit(oled_found);
    Motion_Init();IR_StartScan();
    Control_Init(&mpu);Control_Start();
    UART_Printf("OLED_BOOT,READY,oled=%u,scl=%lu,locked=%u\r\n",
      (unsigned)oled_found,(unsigned long)I2C_GetConfiguredSclHz(),
      (unsigned)Motor_OutputsLocked());
    last_draw=millis();last_log=millis();
    for(;;) {
        uint32_t now=millis();
        uint8_t budget=12U;
        MPU6050_Update(&mpu);
        while(budget-- && UART_ReadByte(&ch)) {
            if(ch=='0') {
                OLED_SharedEnable(0U);
                UART_SendString("OLED_STATE,0\r\n");
            } else if(ch=='1') {
                OLED_SharedEnable(1U);
                UART_SendString("OLED_STATE,1\r\n");
            } else if(ch=='?') Emit();
        }
        if(OLED_SharedEnabled() && (uint32_t)(now-last_draw)>=200U) {
            last_draw=now;Draw();
        }
        OLED_SharedService(&mpu);
        if((uint32_t)(now-last_log)>=1000U) {
            last_log=now;Emit();
        }
        if(!MPU6050_NeedsImmediatePoll(&mpu)) __WFI();
    }
}
#endif
