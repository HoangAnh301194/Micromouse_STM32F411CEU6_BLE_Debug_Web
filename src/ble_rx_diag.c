/* Isolated JDY-33 RX qualification. No motion, no Flash writes.
 * A: RX enabled + unsolicited TX; B: RX disabled + same TX; C: RX enabled
 * plus BLE nonce challenge. UART1 PA9/PA10 is the independent observer
 * and the ONLY mode selector. Motor outputs locked until MCU reset.
 */
#include "board.h"
#include "bt_debug.h"
#include "motor.h"
#include "system_timer.h"
#include "uart.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if defined(BLE_RX_DIAG)
#if defined(INTEGRATION_DIAG_MOTOR_ON) || defined(ENCODER_SPEED_PID_DIAG)
#error "BLE_RX_DIAG may never use motor-on or PID firmware"
#endif

#define COMMAND_CAP 40U
#define RX_COMMAND_CAP 40U
#define TX_BEACON_MS 200UL
#define SERIAL_STATUS_MS 1000UL
static char serial_command[COMMAND_CAP];
static uint8_t serial_len, serial_bad;
static uint8_t rx_command[RX_COMMAND_CAP];
static uint8_t rx_len, rx_bad;
static uint8_t phase='B'; /* Safe default, receive disabled. */
static uint32_t beacon_seq, last_beacon, last_status;
static uint32_t valid_challenges, bad_challenges, rx_other;

static char Hex(uint8_t n) { return "0123456789ABCDEF"[n & 15U]; }
static int HexValue(uint8_t b)
{
    if (b>='0' && b<='9') return b-'0';
    if (b>='A' && b<='F') return b-'A'+10;
    if (b>='a' && b<='f') return b-'a'+10;
    return -1;
}
static void PrintStats(void)
{
    BT_Profile_t b;
    BT_GetProfile(&b);
    UART_Printf("RXS,mode=%c,ms=%lu,rx=%lu,ore=%lu,fe=%lu,ne=%lu,overflow=%lu,ok=%lu,bad=%lu,other=%lu,txaccept=%lu,txdr=%lu,drop=%lu,q=%u,peak=%u,locked=%u\r\n",
        phase,(unsigned long)millis(),(unsigned long)b.rx_bytes,
        (unsigned long)b.rx_ore,(unsigned long)b.rx_fe,
        (unsigned long)b.rx_ne,(unsigned long)b.rx_overflow,
        (unsigned long)valid_challenges,(unsigned long)bad_challenges,
        (unsigned long)rx_other,(unsigned long)b.tx_accepted,
        (unsigned long)b.tx_uart_dr_bytes,(unsigned long)b.tx_drop,
        (unsigned)b.tx_queue,(unsigned)b.tx_queue_peak,
        (unsigned)Motor_OutputsLocked());
}
static void SetMode(char next)
{
    uint8_t discard, budget=255U;
    uint32_t ignored;
    /* Always gate receiver first; clear UART overrun then discard any old
     * RX ring payload before enabling a new bounded phase. */
    USART2->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE);
    ignored=USART2->SR; ignored=USART2->DR; (void)ignored;
    while(budget-- && BT_ReadByte(&discard)) {}
    rx_len=rx_bad=0U;
    phase=(uint8_t)next;
    if(next!='B') USART2->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
    PrintStats();
    UART_Printf("RXMODE,mode=%c,enabled=%u,locked=%u\r\n",
                phase,(unsigned)(next!='B'),(unsigned)Motor_OutputsLocked());
    BT_Printf("RMODE,mode=%c,enabled=%u\r\n",phase,(unsigned)(next!='B'));
}
static void HandleSerial(void)
{
    serial_command[serial_len]='\0';
    if(serial_bad) UART_SendString("RXERR,BAD_UART1_COMMAND\r\n");
    else if(!strcmp(serial_command,"MODE A")) SetMode('A');
    else if(!strcmp(serial_command,"MODE B")) SetMode('B');
    else if(!strcmp(serial_command,"MODE C")) SetMode('C');
    else if(!strcmp(serial_command,"STATS")) PrintStats();
    else if(!strcmp(serial_command,"HELP"))
        UART_SendString("RXHELP,MODE A|MODE B|MODE C|STATS; motors=LOCKED; flash=NO_WRITES\r\n");
    else UART_SendString("RXERR,UNKNOWN_UART1_COMMAND\r\n");
}
static void PollSerial(void)
{
    uint8_t b, budget=32U;
    while(budget-- && UART_ReadByte(&b)) {
        if(b=='\r'||b=='\n') {
            if(serial_len||serial_bad) HandleSerial();
            serial_len=serial_bad=0U;
        } else if(!serial_bad) {
            if(b<32U||b>126U||serial_len>=COMMAND_CAP-1U) serial_bad=1U;
            else serial_command[serial_len++]=(char)b;
        }
    }
}
/* BLE request C,NNNN,HHHHHHHH\n — deterministic nonce echo;
 * 4 ASCII digits and 8 ASCII hex digits, no motor/PID parameter writes.
 * Independent UART1 RXACK is the ground truth of MCU acceptance. */
static void DecodeChallenge(void)
{
    uint16_t seq=0U;
    uint32_t value=0U;
    uint8_t i;
    if(phase!='C') { rx_other++; return; }
    if(rx_bad || rx_len!=15U || rx_command[0]!='C' ||
       rx_command[1]!=',' || rx_command[6]!=',') goto invalid;
    for(i=2U;i<6U;i++) {
        if(rx_command[i]<'0'||rx_command[i]>'9') goto invalid;
        seq=(uint16_t)(seq*10U+(uint16_t)(rx_command[i]-'0'));
    }
    for(i=7U;i<15U;i++) {
        int d=HexValue(rx_command[i]);
        if(d<0) goto invalid;
        value=(value<<4U)|(uint32_t)d;
    }
    valid_challenges++;
    UART_Printf("RXACK,seq=%u,value=%08lX,ms=%lu\r\n",
                (unsigned)seq,(unsigned long)value,(unsigned long)millis());
    BT_Printf("RACK,seq=%u,value=%08lX\r\n",
              (unsigned)seq,(unsigned long)value);
    return;
invalid:
    bad_challenges++;
    /* Short bounded diagnostic evidence for malformed/framing corruption. */
    {
        char raw[2U*RX_COMMAND_CAP+1U];
        uint8_t len=(rx_len<RX_COMMAND_CAP)?rx_len:RX_COMMAND_CAP;
        for(i=0U;i<len;i++) {
            raw[2U*i]=Hex(rx_command[i]>>4U);
            raw[2U*i+1U]=Hex(rx_command[i]);
        }
        raw[2U*len]='\0';
        UART_Printf("RXBAD,len=%u,hex=%s\r\n",(unsigned)len,raw);
    }
}
static void PollBleRx(void)
{
    uint8_t b,budget=48U;
    if(phase=='B') return;
    while(budget-- && BT_ReadByte(&b)) {
        if(b=='\r'||b=='\n') {
            if(rx_len||rx_bad) DecodeChallenge();
            rx_len=rx_bad=0U;
        } else if(rx_len<RX_COMMAND_CAP-1U) {
            rx_command[rx_len++]=b;
        } else rx_bad=1U;
    }
}
int BleRxDiag_Main(void)
{
    Board_Init();
    SystemTimer_Init();
    UART_Init();
    if(Board_GetClockFault()) {
        UART_SendString("RXBOOT,status=CLOCK_FAULT\r\n");
        Board_StatusLed(1U);
        while(1) { __WFI(); }
    }
    Motor_Init();
    Motor_LockOutputs();
    BT_Init();
    SetMode('B');
    UART_SendString("RXBOOT,ver=1,baud=115200,motors=LOCKED,flash=NO_WRITES,default=B\r\n");
    PrintStats();
    for(;;) {
        uint32_t ms=millis();
        PollSerial();
        PollBleRx();
        if((uint32_t)(ms-last_beacon)>=TX_BEACON_MS) {
            last_beacon=ms;
            beacon_seq++;
            BT_Printf("RHB,mode=%c,seq=%lu,ms=%lu\r\n",
                      phase,(unsigned long)beacon_seq,(unsigned long)ms);
        }
        if((uint32_t)(ms-last_status)>=SERIAL_STATUS_MS) {
            last_status=ms;
            PrintStats();
        }
    }
}
#else
typedef int ble_rx_diag_disabled_translation_unit;
#endif
