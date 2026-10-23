/* One-shot JDY-33 BLE qualification firmware.
 * Run with tools/ble_full_pipeline.py; not a production motor controller.
 * Dedicated build: integrates proven TIM11/IR/MPU/encoder and guarded
 * integration_sweep speed-PID without USART1 or USB-TTL attached to the car.
 */
#if defined(BLE_PROFILE_DIAG)
#if !defined(INTEGRATION_DIAG)
#error "BLE profiling requires existing TIM11/IR instrumentation"
#endif
#if !defined(INTEGRATION_DIAG_MOTOR_OFF) && !defined(INTEGRATION_DIAG_MOTOR_ON)
#error "BLE diagnostic firmware requires explicit motor mode"
#endif
#if defined(INTEGRATION_DIAG_MOTOR_ON) && (!defined(ENCODER_SPEED_PID_DIAG) || !defined(INTEGRATION_DIAG_LIMITED_PWM_30))
#error "BLE motor-on firmware MUST use the guarded 30pct speed PID"
#endif

#include "board.h"
#include "bt_debug.h"
#include "control.h"
#include "encoder.h"
#include "integration_sweep.h"
#include "ir_sensor.h"
#include "motor.h"
#include "motion.h"
#include "mpu6050_i2c_module.h"
#include "system_timer.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#define MAX_PACKET 128U
#define RX_CMD 48U
#define MAX_RUN_MS 12000U
#define MCU_PROFILE_VERSION 1U

static MPU6050_Handle_t mpu;
static char command[RX_CMD];
static uint8_t cmd_len, cmd_overflow, next_session;
static uint8_t running, session, packet_size;
static uint16_t requested_hz;
static uint32_t due_us, end_us, seq, generated, accepted, rejected, skipped;
static uint32_t encode_cycles, encode_max, status_cycles, last_status_ms;
static uint32_t base_tx_drop, base_rx_error;
static uint32_t run_start_ms;

static void Put16(uint8_t *d, int16_t v)
{
    uint16_t u=(uint16_t)v;
    d[0]=(uint8_t)u;
    d[1]=(uint8_t)(u>>8);
}
static void Put32(uint8_t *d, uint32_t v)
{
    d[0]=(uint8_t)v;
    d[1]=(uint8_t)(v>>8);
    d[2]=(uint8_t)(v>>16);
    d[3]=(uint8_t)(v>>24);
}
static uint16_t Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc=0xffffU, i;
    uint8_t n;
    for (i=0U;i<len;i++) {
        crc^=(uint16_t)data[i]<<8;
        for (n=0U;n<8U;n++) crc=(uint16_t)((crc&0x8000U) ?
                          (crc<<1)^0x1021U : crc<<1);
    }
    return crc;
}
static int16_t Clip(float v)
{
    if (v>32767.0f) return 32767;
    if (v< -32768.0f) return -32768;
    return (int16_t)v;
}
static uint32_t RxErrors(const BT_Profile_t *p)
{
    return p->rx_ore+p->rx_fe+p->rx_ne+p->rx_overflow;
}
static uint8_t SensorsReady(void)
{
    MPU6050_Snapshot_t s;
    return Board_GetClockFault()==0U && mpu.cal_state==3U &&
           mpu.calibrated &&
           MPU6050_GetSnapshot(&mpu,&s) && s.valid &&
           (uint32_t)(micros()-s.completed_us)<=3000U;
}
static void PrintStatus(uint8_t detailed)
{
    Control_DebugStats_t c;
    BT_Profile_t b;
    IR_Diagnostics_t ir;
    Motor_Diagnostics_t m;
    uint32_t started=DWT->CYCCNT;
    Control_GetDebugStats(&c);
    BT_GetProfile(&b);
    IR_GetDiagnostics(&ir);
    Motor_GetDiagnostics(&m);
    BT_Printf("!S,s=%u,ms=%lu,cal=%u,tick=%lu,late=%lu,gap=%lu,maxcy=%lu,ir=%lu,irfail=%lu,valid=%lu,stale=%lu,i2cerr=%lu,i2cto=%lu,txdrop=%lu,q=%u,peak=%u,txcy=%lu,rxerr=%lu,locked=%u\n",
        (unsigned)session,(unsigned long)millis(),(unsigned)mpu.cal_state,
        (unsigned long)c.ticks,(unsigned long)c.late_gt50us,
        (unsigned long)c.gaps_gt1500us,(unsigned long)c.max_irq_cycles,
        (unsigned long)ir.scans_completed,(unsigned long)ir.scans_failed,
        (unsigned long)c.gyro_valid_ticks,
        (unsigned long)c.gyro_invalid_after_first_valid,
        (unsigned long)mpu.error_reads,(unsigned long)mpu.timeout_reads,
        (unsigned long)b.tx_drop,(unsigned)b.tx_queue,(unsigned)b.tx_queue_peak,
        (unsigned long)b.irq_cycles,(unsigned long)RxErrors(&b),
        (unsigned)m.outputs_locked);
    if (detailed) {
        int16_t applied_l=0,applied_r=0;
        SweepSnapshot sw;
        Motor_GetAppliedPairPermille(&applied_l,&applied_r);
        Sweep_GetSnapshot(&sw);
        BT_Printf("!D,s=%u,encL=%ld,encR=%ld,ppsL=%ld,ppsR=%ld,appL=%d,appR=%d,sweep=%u,step=%lu,fault=%u,gen=%lu,accepted=%lu,reject=%lu,skip=%lu,ecy=%lu,emax=%lu\n",
            (unsigned)session,(long)Encoder_GetLeftCount(),
            (long)Encoder_GetRightCount(),
            (long)Encoder_GetLeftSpeedPPS(),(long)Encoder_GetRightSpeedPPS(),
            (int)applied_l,(int)applied_r,(unsigned)sw.state,
            (unsigned long)sw.stage_id,(unsigned)sw.fault,
            (unsigned long)generated,(unsigned long)accepted,
            (unsigned long)rejected,(unsigned long)skipped,
            (unsigned long)encode_cycles,(unsigned long)encode_max);
        BT_Printf("!T,s=%u,b0=%lu,b1=%lu,b2=%lu,b3=%lu,b4=%lu,b5=%lu,b6=%lu,b7=%lu\n",
            (unsigned)session,
            (unsigned long)c.irq_hist[0],(unsigned long)c.irq_hist[1],
            (unsigned long)c.irq_hist[2],(unsigned long)c.irq_hist[3],
            (unsigned long)c.irq_hist[4],(unsigned long)c.irq_hist[5],
            (unsigned long)c.irq_hist[6],(unsigned long)c.irq_hist[7]);
        BT_Printf("!U,s=%u,accb=%lu,uartdr=%lu,rxbytes=%lu,irqn=%lu,ore=%lu,fe=%lu,ne=%lu,overflow=%lu\n",
            (unsigned)session,
            (unsigned long)b.tx_accepted,(unsigned long)b.tx_uart_dr_bytes,
            (unsigned long)b.rx_bytes,(unsigned long)b.irq_count,
            (unsigned long)b.rx_ore,(unsigned long)b.rx_fe,
            (unsigned long)b.rx_ne,(unsigned long)b.rx_overflow);
    }
    status_cycles+=(uint32_t)(DWT->CYCCNT-started);
}
static void EndRun(void)
{
    BT_Profile_t b;
    BT_GetProfile(&b);
    running=0U;
    BT_Printf("!E,s=%u,gen=%lu,ok=%lu,reject=%lu,skip=%lu,size=%u,hz=%u,ms=%lu,ecy=%lu,emax=%lu,scy=%lu,q=%u,peak=%u,drop=%lu,rxerr=%lu\n",
         (unsigned)session,(unsigned long)generated,
         (unsigned long)accepted,(unsigned long)rejected,
         (unsigned long)skipped,(unsigned)packet_size,
         (unsigned)requested_hz,(unsigned long)(millis()-run_start_ms),
         (unsigned long)encode_cycles,(unsigned long)encode_max,
         (unsigned long)status_cycles,(unsigned)b.tx_queue,
         (unsigned)b.tx_queue_peak,
         (unsigned long)(b.tx_drop-base_tx_drop),
         (unsigned long)(RxErrors(&b)-base_rx_error));
}
static void MakePacket(uint8_t size)
{
    uint8_t frame[MAX_PACKET], i;
    uint32_t start=DWT->CYCCNT, stamp=micros();
    uint16_t crc, off;
    MPU6050_Snapshot_t imu;
    IR_Data_t ir;
    int16_t applied_l=0, applied_r=0;
    /* Fixed-size frame: B5 62 ver type length session seq32 t_us32 payload CRC16.
     * Payload bytes are deterministic for transport tests; first fields are real
     * sensors whenever size permits. No printf in the high-rate producer. */
    frame[0]=0xb5U; frame[1]=0x62U;
    frame[2]=MCU_PROFILE_VERSION; frame[3]='D';
    frame[4]=size; frame[5]=session;
    Put32(frame+6,seq);
    Put32(frame+10,stamp);
    for (i=14U;i<(uint8_t)(size-2U);i++)
        frame[i]=(uint8_t)(i+(uint8_t)seq);
    if (size>=20U) {
        Put16(frame+14,Clip(Encoder_GetLeftSpeedPPS()));
        Put16(frame+16,Clip(Encoder_GetRightSpeedPPS()));
    }
    if (size>=32U) {
        (void)MPU6050_GetSnapshot(&mpu,&imu);
        Put16(frame+18,Clip(imu.gyro_z_dps*10.0f));
        Put16(frame+20,Clip(imu.yaw_deg*10.0f));
        Motor_GetAppliedPairPermille(&applied_l,&applied_r);
        Put16(frame+22,applied_l);
        Put16(frame+24,applied_r);
        Put16(frame+26,Clip((float)Encoder_GetLeftCount()));
        Put16(frame+28,Clip((float)Encoder_GetRightCount()));
    }
    if (size>=64U) {
        Control_GetIRSnapshot(&ir);
        for (off=0U;off<6U;off++)
            Put16(frame+30U+2U*off,(int16_t)ir.value[off]);
        Put32(frame+42U,ir.scan_count);
        Put32(frame+46U,ir.completed_us);
        Put32(frame+50U,imu.completed_us);
        Put32(frame+54U,imu.sequence);
    }
    crc=Crc16(frame,(uint16_t)(size-2U));
    frame[size-2U]=(uint8_t)crc;
    frame[size-1U]=(uint8_t)(crc>>8);
    {
        uint32_t cycles=(uint32_t)(DWT->CYCCNT-start);
        encode_cycles+=cycles;
        if (cycles>encode_max) encode_max=cycles;
    }
    ++generated;
    ++seq; /* Attempt sequence; gaps reveal rejected packets. */
    if (BT_TrySendData(frame,size)) ++accepted;
    else ++rejected;
}
static void StartRun(uint8_t size, uint16_t hz, uint32_t duration_ms)
{
    BT_Profile_t b;
    if (running || size<16U || size>MAX_PACKET ||
        (size!=16U && size!=20U && size!=32U &&
         size!=64U && size!=128U) ||
        hz>400U || duration_ms<1000U || duration_ms>MAX_RUN_MS ||
        !SensorsReady()) {
        BT_SendString("!ERR,reason=RUN_REJECTED\n");
        return;
    }
#if defined(INTEGRATION_DIAG_MOTOR_ON)
    /* Motor-on samples only while host-authorized guarded sweep is active. */
    if (!Sweep_IsActive()) {
        BT_SendString("!ERR,reason=NOT_ARMED\n");
        return;
    }
#else
    if (!Motor_OutputsLocked()) {
        BT_SendString("!ERR,reason=NOT_LOCKED\n");
        return;
    }
#endif
    session=++next_session;
    generated=accepted=rejected=skipped=0U;
    encode_cycles=encode_max=status_cycles=0U;
    packet_size=size;
    requested_hz=hz;
    run_start_ms=millis();
    due_us=micros()+(hz ? 1000000UL/hz : 0U);
    end_us=micros()+duration_ms*1000UL;
    BT_GetProfile(&b);
    base_tx_drop=b.tx_drop;
    base_rx_error=RxErrors(&b);
    running=1U;
    BT_Printf("!B,s=%u,size=%u,hz=%u,ms=%lu\n",
        (unsigned)session,(unsigned)size,(unsigned)hz,
        (unsigned long)duration_ms);
}
static void Handle(const char *cmd)
{
    unsigned int size=0U,hz=0U,ms=0U;
    char extra;
    if (strcmp(cmd,"HELLO")==0 || strcmp(cmd,"STATUS")==0) {
        BT_Printf("!H,ver=1,mode=%s,cal=%u,baud=%lu,ready=%u,maxpkt=128\n",
#if defined(INTEGRATION_DIAG_MOTOR_ON)
            "ON",
#else
            "OFF",
#endif
            (unsigned)mpu.cal_state,(unsigned long)BT_BAUDRATE,
            (unsigned)SensorsReady());
        PrintStatus(1U);
    } else if (strcmp(cmd,"PING")==0) {
#if defined(INTEGRATION_DIAG_MOTOR_ON)
        Sweep_Heartbeat(millis());
#endif
        BT_SendString("!P,ok=1\n");
    } else if (strcmp(cmd,"STOP")==0 || strcmp(cmd,"!")==0) {
        if (running) EndRun();
#if defined(INTEGRATION_DIAG_MOTOR_ON)
        Sweep_Abort(SWEEP_STOP_REQUEST);
#endif
        BT_SendString("!STOP,ok=1\n");
    } else if (sscanf(cmd,"RUN %u %u %u %c",&size,&hz,&ms,&extra)==3) {
        if (size>255U || hz>65535U) BT_SendString("!ERR,reason=RUN_RANGE\n");
        else StartRun((uint8_t)size,(uint16_t)hz,(uint32_t)ms);
#if defined(INTEGRATION_DIAG_MOTOR_ON)
    } else if (strcmp(cmd,"ARM")==0) {
        if (!SensorsReady() || Motor_OutputsLocked() ||
            !Sweep_SelectMode(1U) || !Sweep_Arm(millis(),&mpu,1U)) {
            BT_SendString("!ERR,reason=ARM_DENIED\n");
        } else {
            BT_Printf("!ARM,ok=1,pwm_max=%u,stages=%u\n",
                       (unsigned)Sweep_MaxPwmPercent(),
                       (unsigned)Sweep_StageTotal());
        }
#endif
    } else {
        /* Diagnose corruption/baud mismatch/misframing on PA3. Hex encoding
         * escapes control/non-ASCII bytes; never execute unknown commands. */
        char hex[2U * RX_CMD + 1U];
        static const char digits[] = "0123456789ABCDEF";
        uint8_t n = 0U;
        while (cmd[n] && n < RX_CMD - 1U) {
            uint8_t v = (uint8_t)cmd[n];
            hex[2U*n] = digits[v >> 4];
            hex[2U*n + 1U] = digits[v & 15U];
            n++;
        }
        hex[2U*n] = '\0';
        BT_Printf("!ERR,reason=UNKNOWN_COMMAND,rxhex=%s,rxlen=%u\n",
                  hex, (unsigned)n);
    }
}
static void PumpCommands(void)
{
    uint8_t byte, budget=48U;
    while (budget-- && BT_ReadByte(&byte)) {
        if (byte=='\n' || byte=='\r') {
            if (cmd_len && !cmd_overflow) {
                command[cmd_len]='\0';
                Handle(command);
            }
            cmd_len=0U; cmd_overflow=0U;
        } else if (byte=='!') {
            /* Emergency STOP can be sent as a single byte. */
            cmd_len=0U; cmd_overflow=0U;
            Handle("!");
        } else if (!cmd_overflow) {
            if (cmd_len+1U<RX_CMD) command[cmd_len++]=(char)byte;
            else cmd_overflow=1U;
        }
    }
}
static void PollRun(void)
{
    uint32_t now;
    if (!running) return;
    now=micros();
    if ((int32_t)(now-end_us)>=0) {
        EndRun();
        return;
    }
    if (requested_hz && (int32_t)(now-due_us)>=0) {
        uint32_t step=1000000UL/requested_hz;
        if ((uint32_t)(now-due_us)>=step)
            skipped+=(uint32_t)(now-due_us)/step;
        due_us=now+step; /* No catch-up burst: protects MPU/main loop. */
        MakePacket(packet_size);
    }
}
int BleBenchmark_Main(void)
{
    MPU6050_Status init;
    Board_Init();
    SystemTimer_Init();
    if (Board_GetClockFault()) {
        Board_StatusLed(1U);
        for (;;) { __WFI(); }
    }
    Motor_Init();
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    Motor_LockOutputs(); /* Sticky, impossible to arm in OFF build. */
#endif
    Encoder_Init();
    BT_Init();
    IR_Init();
    init=MPU6050_Init(&mpu);
    if (init==MPU6050_OK) init=MPU6050_Calibrate(&mpu);
    Motion_Init();
    Sweep_Init();
    if (init!=MPU6050_OK) {
        Motor_LockOutputs();
        BT_SendString("!ERR,reason=MPU_BOOT\n");
        for (;;) { __WFI(); }
    }
    IR_StartScan();
    Control_Init(&mpu);
    Control_Start();
    BT_Printf("!BOOT,ver=1,mode=%s,baud=%lu\n",
#if defined(INTEGRATION_DIAG_MOTOR_ON)
            "ON",
#else
            "OFF",
#endif
            (unsigned long)BT_BAUDRATE);
    for (;;) {
        uint32_t now_ms;
        MPU6050_Update(&mpu);
        PumpCommands();
        now_ms=millis();
#if defined(INTEGRATION_DIAG_MOTOR_ON)
        Sweep_Update(now_ms,&mpu);
        if (Motor_OutputsLocked() && Sweep_IsActive()) {
            Sweep_Abort(SWEEP_MOTOR_WATCHDOG);
        }
        if (running && !Sweep_IsActive()) EndRun();
#endif
        PollRun();
        if ((uint32_t)(now_ms-last_status_ms)>=1000U) {
            last_status_ms=now_ms;
            PrintStatus(0U);
        }
    }
}
#endif /* BLE_PROFILE_DIAG */
