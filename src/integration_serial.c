/* USART1 PA9 DMA TX / PA10 RXNE -> bounded main-loop line parser.
 * Exact ASCII commands (CRLF or LF):
 *   HELP, STATUS, MODE OFF, MODE ON, START, PING, STOP
 * '!' is an asynchronous byte STOP, even without a line ending.
 * MODE ON permitted only for explicit motor-on firmware profile.
 * START in MODE ON requires explicit host command, ready calibrated MPU,
 * UART lease heartbeat, and independent external VM cutoff (no KEY button).
 * Single run per reset; DONE, STOP or fault latches motor outputs off.
 */
#include "integration_serial.h"
#if defined(INTEGRATION_DIAG)
#include "integration_sweep.h"
#include "uart.h"
#include "motor.h"
#include "board.h"
#include "system_timer.h"
#include <string.h>

#define UART_CMD_MAX_LENGTH 32U
#define UART_RX_BUDGET 48U

static char cmd[UART_CMD_MAX_LENGTH];
static uint8_t cmd_len;
static uint8_t cmd_overflow;
static uint8_t suppress_next_lf;

static void PrintStatus(const MPU6050_Handle_t *mpu, uint32_t now)
{
    SweepSnapshot snap;
    Motor_Diagnostics_t motor;
    Sweep_GetSnapshot(&snap);
    Motor_GetDiagnostics(&motor);
    UART_Printf("CMD_STATUS,ms=%lu,mode=%s,state=%u,fault=%u,armed=%u,stage=%lu,duty=%u,locked=%u,cal=%u,rx_ov=%lu,rx_err=%lu\r\n",
                (unsigned long)now,
                snap.motor_enabled ? "ON" : "OFF",
                (unsigned)snap.state, (unsigned)snap.fault,
                (unsigned)snap.armed_once,
                (unsigned long)snap.stage_id,
                (unsigned)snap.duty_permille,
                (unsigned)motor.outputs_locked,
                mpu ? (unsigned)mpu->cal_state : 0U,
                (unsigned long)UART_GetRxOverflow(),
                (unsigned long)UART_GetRxErrors());
}
static void Dispatch(const MPU6050_Handle_t *mpu, uint32_t now)
{
    if (strcmp(cmd,"HELP")==0) {
        UART_SendString("CMD_HELP,HELP|STATUS|MODE OFF|MODE ON|START|PING|STOP|!; CRLF/LF; MODE ON requires live build; START ON uses serial-only authorization; one shot per reset\r\n");
    } else if (strcmp(cmd,"STATUS")==0 || strcmp(cmd,"?")==0) {
        PrintStatus(mpu,now);
    } else if (strcmp(cmd,"MODE OFF")==0) {
        uint8_t ok = Sweep_SelectMode(0U);
        UART_Printf("CMD_MODE,requested=OFF,accepted=%u\r\n",(unsigned)ok);
    } else if (strcmp(cmd,"MODE ON")==0) {
        uint8_t ok = Sweep_SelectMode(1U);
        UART_Printf("CMD_MODE,requested=ON,accepted=%u\r\n",(unsigned)ok);
    } else if (strcmp(cmd,"START")==0) {
        uint8_t ok = Sweep_Arm(now,mpu,1U); /* Explicit START over UART is the host authorization. */
        UART_Printf("CMD_START,accepted=%u\r\n",(unsigned)ok);
    } else if (strcmp(cmd,"PING")==0) {
        Sweep_Heartbeat(now);
        UART_Printf("CMD_PING,ms=%lu\r\n",(unsigned long)now);
    } else if (strcmp(cmd,"STOP")==0 || strcmp(cmd,"S")==0) {
        Sweep_Abort(SWEEP_STOP_REQUEST);
        Motor_Stop();
        UART_SendString("CMD_STOP,ack=1\r\n");
    } else if (cmd_len) {
        UART_Printf("CMD_REJECT,reason=UNKNOWN,len=%u\r\n",(unsigned)cmd_len);
    }
}
void IntegrationSerial_Poll(const MPU6050_Handle_t *mpu, uint32_t now_ms)
{
    uint8_t value, budget = UART_RX_BUDGET;
    while (budget-- && UART_ReadByte(&value)) {
        if (value=='!') {
            Sweep_Abort(SWEEP_STOP_REQUEST);
            Motor_Stop();
            cmd_len = cmd_overflow = suppress_next_lf = 0U;
            UART_SendString("CMD_STOP,ack=1,source=BANG\r\n");
            continue;
        }
        if (value=='\r' || value=='\n') {
            if (value == '\n' && suppress_next_lf) {
                suppress_next_lf=0U; continue;
            }
            suppress_next_lf=(uint8_t)(value=='\r');
            if (cmd_overflow) UART_SendString("CMD_REJECT,reason=TOO_LONG\r\n");
            else {
                cmd[cmd_len]='\0';
                Dispatch(mpu,now_ms);
            }
            cmd_len=cmd_overflow=0U;
            continue;
        }
        suppress_next_lf=0U;
        if (cmd_overflow) continue;
        if (value < 32U || value > 126U) {
            cmd_overflow=1U; continue;
        }
        if (cmd_len >= UART_CMD_MAX_LENGTH-1U) {
            cmd_overflow=1U; continue;
        }
        cmd[cmd_len++]=(char)value;
    }
}
#endif
