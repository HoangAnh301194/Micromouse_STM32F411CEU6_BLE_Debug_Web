/* Standalone, motor-locked USART2/JDY-33 hardware isolation test.
 * Intentionally initializes NO MPU6050, OLED, IR, encoder, TIM11 or I2C.
 * Repeated TX heartbeat tests MCU -> JDY -> PC without any RX commands.
 * '?' round-trip tests PC -> JDY -> MCU and MCU -> JDY -> PC.
 */
#if defined(SPP_SMOKE_DIAG)
#include "board.h"
#include "motor.h"
#include "system_timer.h"
#include "bt_debug.h"
#include "stm32f4xx.h"
#include <stdint.h>

int SPP_Smoke_Main(void)
{
    uint32_t last_hb;
    uint8_t led = 0U;

    Board_Init();
    SystemTimer_Init();
    /* Do not touch the motor driver until the board clock is confirmed.
     * A clock failure is indicated by PC13 held ON (no UART assumptions). */
    if (Board_GetClockFault() != 0U) {
        Board_StatusLed(1U);
        for (;;) { __WFI(); }
    }
    Motor_Init();
    Motor_LockOutputs(); /* Irreversible until reset. */
    BT_Init();

    last_hb = millis();
    BT_Printf("SPP_SMOKE,BOOT,baud=%lu,pclk1=%lu,trim=%lu\r\n",
              (unsigned long)BT_BAUDRATE,
              (unsigned long)Board_GetAPB1ClockHz(),
              (unsigned long)Board_GetHsiTrim());
    for (;;) {
        uint8_t command;
        uint8_t budget = 16U;
        uint32_t now = millis();
        while (budget-- && BT_ReadByte(&command)) {
            if (command == '?') {
                BT_Profile_t profile;
                BT_GetProfile(&profile);
                BT_SendString("SPP CONNECTED\r\n");
                BT_Printf("SPP_SMOKE,PONG,ms=%lu,rx=%lu,fe=%lu,ne=%lu,ore=%lu,drop=%lu\r\n",
                          (unsigned long)now,
                          (unsigned long)profile.rx_bytes,
                          (unsigned long)profile.rx_fe,
                          (unsigned long)profile.rx_ne,
                          (unsigned long)profile.rx_ore,
                          (unsigned long)profile.tx_drop);
            }
        }
        if ((uint32_t)(now - last_hb) >= 1000U) {
            BT_Profile_t profile;
            last_hb = now;
            BT_GetProfile(&profile);
            BT_Printf("SPP_SMOKE,HB,ms=%lu,rx=%lu,fe=%lu,ne=%lu,ore=%lu,drop=%lu\r\n",
                      (unsigned long)now,
                      (unsigned long)profile.rx_bytes,
                      (unsigned long)profile.rx_fe,
                      (unsigned long)profile.rx_ne,
                      (unsigned long)profile.rx_ore,
                      (unsigned long)profile.tx_drop);
            led ^= 1U;
            Board_StatusLed(led);
        }
    }
}
#endif
