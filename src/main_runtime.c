/* Legacy production/integration orchestration extracted from main.c.
 * Body unchanged, to isolate motion and IR diagnostic entrypoints. */
#if !defined(MAZE_SENSOR_CAPTURE_MODE) && !defined(TURN_PID_DIAG) && !defined(WALL_PID_TEST_MODE) && !defined(SYSTEM_TEST_MODE) && \
    !defined(BLE_RX_DIAG) && !defined(OLED_MPU_DIAG) && \
    !defined(IR_OLED_TEST) && !defined(IR_FILTER_DIAG) && \
    !defined(BLE_PROFILE_DIAG) && !defined(FLASH_MAP_DIAG)
#include "board.h"
#include "bt_debug.h"
#if defined(ENCODER_PROFILE_DIAG) && (!defined(INTEGRATION_DIAG) || \
    !(defined(INTEGRATION_DIAG_MOTOR_OFF) || defined(INTEGRATION_DIAG_MOTOR_ON)))
#error "Encoder telemetry requires an explicit integration diagnostic mode"
#endif

#include "control.h"
#include "encoder.h"
#include "ir_sensor.h"
#include "motion.h"
#include "motor.h"
#include "mpu6050_i2c_module.h"
#include "system_timer.h"
#include "uart.h"
#include "integration_sweep.h"
#if defined(INTEGRATION_DIAG)
#include "integration_serial.h"
#if defined(UART_PROFILE_DIAG)
#include "uart_bench.h"
#endif
#endif

#include "stm32f4xx.h"
#if defined(MODE_MENU_ONLY)
#if !defined(OLED_SHARED_I2C) || defined(INTEGRATION_DIAG) || defined(OLED_PRODUCTION_STATUS)
#error "MODE_MENU_ONLY requires shared-I2C OLED and cannot combine with motion diagnostics"
#endif
#include "mode_menu.h"
#include "mode_test_resources.h"
#include "encoder_onebox.h"
#include "ir_calibration_360.h"
#include "oled_shared.h"
#include "ssd1306.h"
#endif
#if defined(ENCODER_SPEED_PID_DIAG)
#include "speed_pid_diag.h"
#endif

#if !defined(MODE_MENU_ONLY)
#define DEBUG_PERIOD_MS 100UL
#endif

static MPU6050_Handle_t mpu;

#if defined(MODE_MENU_ONLY)
/* PID gain protocol: p + six 5-digit unsigned fixed-point coefficients + ';'.
 * All bytes are consumed as a frame, never dispatched as motion commands.
 * '?' resynchronizes; 'P' always remains available as an abort. */
static uint8_t pid_rx_active, pid_rx_bad, pid_rx_count;
static uint32_t pid_rx_started, pid_rx_values[6];
static void PidMenu_ReportGains(const char *tag)
{
    OneBoxGains l, r;
    EncoderOneBox_GetGains(&l, &r);
    BT_Printf("PID,%s,%lu,%lu,%lu,%lu,%lu,%lu\r\n", tag,
              (unsigned long)l.p, (unsigned long)l.i, (unsigned long)l.d,
              (unsigned long)r.p, (unsigned long)r.i, (unsigned long)r.d);
}
static uint8_t PidMenu_Consume(char c, const ModeMenu *menu, uint32_t now)
{
    if (c == '?' || c == 'P') { pid_rx_active = 0U; return 0U; }
    if (c == 'p') {
        uint8_t i;
        pid_rx_active = 1U; pid_rx_bad = 0U; pid_rx_count = 0U; pid_rx_started = now;
        for (i=0U; i<6U; ++i) pid_rx_values[i] = 0U;
        return 1U;
    }
    if (!pid_rx_active) return 0U;
    if ((uint32_t)(now - pid_rx_started) > 1000U) {
        pid_rx_bad = 1U;
        if (c == 'N' || c == 'K') { pid_rx_active = 0U; return 0U; }
    }
    if (c == ';') {
        OneBoxGains l = {pid_rx_values[0], pid_rx_values[1], pid_rx_values[2]};
        OneBoxGains r = {pid_rx_values[3], pid_rx_values[4], pid_rx_values[5]};
        pid_rx_active = 0U;
        if (!pid_rx_bad && pid_rx_count == 30U &&
            menu->page == MODE_MENU_PAGE_ENCODER &&
            menu->active_encoder_option == MODE_MENU_ENCODER_PID &&
            EncoderOneBox_SetGains(&l, &r)) PidMenu_ReportGains("APPLIED");
        else BT_SendString("PID,REJECT,INVALID_OR_NOT_IDLE\r\n");
    } else if (c >= '0' && c <= '9' && pid_rx_count < 30U) {
        uint8_t slot = pid_rx_count / 5U;
        pid_rx_values[slot] = pid_rx_values[slot] * 10U + (uint32_t)(c - '0');
        ++pid_rx_count;
    } else pid_rx_bad = 1U;
    return 1U;
}
/* Main-context framebuffer edits only. I2C transfers are owned by the
 * admission-controlled OLED_SharedService, never by TIM11 or USART2 IRQ. */
static void ModeMenu_Draw(const ModeMenu *menu, uint8_t confirmed)
{
    uint8_t first, row, i;
    if (!menu) return;

    SSD1306_Fill(SSD1306_BLACK);
    if (menu->page == MODE_MENU_PAGE_MPU) {
        SSD1306_SetCursor(0U,0U); SSD1306_WriteString("MPU TEST", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U,20U); SSD1306_WriteString((char *)ModeMenu_MpuName(menu->encoder_focus), &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U,40U); SSD1306_WriteString("Next / Select", &Font_7x10, SSD1306_WHITE);
        return;
    }
    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        /* Scroll four visible options for 128x64 OLED; OLED remains
         * optional/off by default in the motor-locked SPP profile. */
        SSD1306_SetCursor(0U, 0U);
        SSD1306_WriteString("ENCODER TEST", &Font_7x10, SSD1306_WHITE);
        SSD1306_DrawLine(0U, 12U, 127U, 12U, SSD1306_WHITE);
        if (menu->active_encoder_option != MODE_MENU_NO_SELECTION) {
            SSD1306_SetCursor(0U, 20U);
            SSD1306_WriteString((char *)ModeMenu_EncoderName(menu->active_encoder_option),
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 38U);
            SSD1306_WriteString(menu->active_encoder_option ==
                                MODE_MENU_ENCODER_LOG_RECEIVE ?
                                (menu->stream_running ? "LOG ACTIVE" : "LOG PAUSED") :
                                (menu->active_encoder_option == MODE_MENU_ENCODER_PID ? "EDIT PID ON PC" :
                                 menu->active_encoder_option == MODE_MENU_ENCODER_1BOX ? "START ON PC" : "NO MOTION YET"),
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 52U);
            SSD1306_WriteString("Next: browse", &Font_7x10, SSD1306_WHITE);
            return;
        }
        if (menu->encoder_focus == MODE_MENU_NO_SELECTION) {
            SSD1306_SetCursor(0U, 23U);
            SSD1306_WriteString("Next: choose", &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 40U);
            SSD1306_WriteString("Hold: select", &Font_7x10, SSD1306_WHITE);
            return;
        }
        first = menu->encoder_focus >= 3U ?
                (uint8_t)(menu->encoder_focus - 3U) : 0U;
        for (row = 0U; row < 4U; ++row) {
            i = (uint8_t)(first + row);
            if (i >= MODE_MENU_ENCODER_COUNT) break;
            SSD1306_SetCursor(0U, (uint8_t)(15U + row * 11U));
            SSD1306_WriteString((i == menu->encoder_focus) ? ">" : " ",
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(10U, (uint8_t)(15U + row * 11U));
            SSD1306_WriteString((char *)ModeMenu_EncoderName(i),
                                &Font_7x10, SSD1306_WHITE);
        }
        return;
    }
    if (menu->page == MODE_MENU_PAGE_TEST) {
        SSD1306_SetCursor(0U, 0U);
        SSD1306_WriteString("7.SYSTEM TEST", &Font_7x10, SSD1306_WHITE);
        SSD1306_DrawLine(0U, 12U, 127U, 12U, SSD1306_WHITE);
        if (menu->active_test != MODE_MENU_NO_SELECTION) {
            SSD1306_SetCursor(0U, 20U);
            SSD1306_WriteString((char *)ModeMenu_TestName(menu->active_test),
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 36U);
            SSD1306_WriteString(menu->stream_running ?
                                "STREAM ACTIVE" : "STREAM PAUSED",
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 52U);
            SSD1306_WriteString("Next: stop", &Font_7x10, SSD1306_WHITE);
            return;
        }
        if (menu->test_focus == MODE_MENU_NO_SELECTION) {
            SSD1306_SetCursor(0U, 23U);
            SSD1306_WriteString("Next: choose test", &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(0U, 40U);
            SSD1306_WriteString("Hold: select", &Font_7x10, SSD1306_WHITE);
            return;
        }
        for (row = 0U; row < MODE_MENU_TEST_COUNT; ++row) {
            SSD1306_SetCursor(0U, (uint8_t)(16U + row * 11U));
            SSD1306_WriteString((row == menu->test_focus) ? ">" : " ",
                                &Font_7x10, SSD1306_WHITE);
            SSD1306_SetCursor(10U, (uint8_t)(16U + row * 11U));
            SSD1306_WriteString((char *)ModeMenu_TestName(row),
                                &Font_7x10, SSD1306_WHITE);
        }
        return;
    }

    if (confirmed && menu->selected != MODE_MENU_NO_SELECTION) {
        SSD1306_SetCursor(0U, 0U);
        SSD1306_WriteString("MODE SELECTED", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U, 20U);
        SSD1306_WriteString((char *)ModeMenu_Label(menu->selected),
                            &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U, 39U);
        SSD1306_WriteString("NO ACTION YET", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U, 52U);
        SSD1306_WriteString("Short: next", &Font_7x10, SSD1306_WHITE);
        return;
    }
    SSD1306_SetCursor(10U, 0U);
    SSD1306_WriteString("SELECT MODE", &Font_7x10, SSD1306_WHITE);
    SSD1306_DrawLine(0U, 12U, 127U, 12U, SSD1306_WHITE);
    if (menu->focus == MODE_MENU_NO_SELECTION) {
        SSD1306_SetCursor(0U, 25U);
        SSD1306_WriteString("Press PA0: Next", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0U, 43U);
        SSD1306_WriteString("Hold: Select", &Font_7x10, SSD1306_WHITE);
        return;
    }
    first = menu->focus >= 3U ? (uint8_t)(menu->focus - 3U) : 0U;
    for (row = 0U; row < 4U; ++row) {
        i = (uint8_t)(first + row);
        if (i >= MODE_MENU_ITEM_COUNT) break;
        SSD1306_SetCursor(0U, (uint8_t)(15U + row * 11U));
        SSD1306_WriteString((i == menu->focus) ? ">" : " ",
                            &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(10U, (uint8_t)(15U + row * 11U));
        SSD1306_WriteString((char *)ModeMenu_Label(i),
                            &Font_7x10, SSD1306_WHITE);
    }
}

static void ModeMenu_Report(const ModeMenu *menu, ModeMenuEvent event)
{
    switch (event) {
    case MODE_MENU_EVENT_FOCUS:
        BT_Printf("Mode %u: %s\r\n", (unsigned)(menu->focus + 1U),
                  ModeMenu_Name(menu->focus));
        break;
    case MODE_MENU_EVENT_SELECT:
        BT_Printf("Selected mode %u: %s\r\n",
                  (unsigned)(menu->selected + 1U),
                  ModeMenu_Name(menu->selected));
        break;
    case MODE_MENU_EVENT_TEST_ENTER:
        BT_SendString("Selected mode 7: System Test\r\n");
        BT_SendString("System Test: choose IR / Encoder / MPU / Back\r\n");
        break;
    case MODE_MENU_EVENT_TEST_FOCUS:
        BT_Printf("System Test %u: %s\r\n",
                  (unsigned)(menu->test_focus + 1U),
                  ModeMenu_TestName(menu->test_focus));
        break;
    case MODE_MENU_EVENT_TEST_SELECT:
        BT_Printf("Selected test %u: %s\r\n",
                  (unsigned)(menu->active_test + 1U),
                  ModeMenu_TestName(menu->active_test));
        if (menu->active_test == MODE_MENU_TEST_MPU)
            BT_SendString("TEST,MPU,ANGLE_ONLY,MOTOR_LOCKED,KEEP_STILL_FOR_CALIBRATION,START_TO_LOG\r\n");
        break;
    case MODE_MENU_EVENT_TEST_EXIT:
        BT_SendString("System Test: back to main menu\r\n");
        break;
    case MODE_MENU_EVENT_MPU_ENTER:
        BT_SendString("MPU Test: choose MPU Log / Pivot / Back\r\n"); break;
    case MODE_MENU_EVENT_MPU_FOCUS:
        BT_Printf("MPU Test option %u: %s\r\n",(unsigned)(menu->encoder_focus+1U),ModeMenu_MpuName(menu->encoder_focus)); break;
    case MODE_MENU_EVENT_MPU_SELECT:
        BT_Printf("Selected mpu option %u: %s\r\n",(unsigned)(menu->active_encoder_option+1U),ModeMenu_MpuName(menu->active_encoder_option));
        if (menu->active_encoder_option==0U) BT_SendString("Selected test 3: MPU Test\r\n");
        break;
    case MODE_MENU_EVENT_MPU_EXIT:
        BT_SendString("MPU Test: back to System Test\r\n"); break;
    case MODE_MENU_EVENT_ENCODER_ENTER:
        BT_SendString("Encoder Test: choose Log Receive / motion tests / Back\r\n");
        break;
    case MODE_MENU_EVENT_ENCODER_FOCUS:
        BT_Printf("Encoder Test option %u: %s\r\n",
                  (unsigned)(menu->encoder_focus + 1U),
                  ModeMenu_EncoderName(menu->encoder_focus));
        break;
    case MODE_MENU_EVENT_ENCODER_SELECT:
        if (menu->active_encoder_option == MODE_MENU_ENCODER_LOG_RECEIVE) {
            BT_SendString("Selected encoder option 1: Log Receive\r\n");
        } else if (menu->active_encoder_option == MODE_MENU_ENCODER_1BOX) {
            BT_SendString("Selected encoder option 2: 1 Box Forward (ARMED; press Start)\r\n");
        } else if (menu->active_encoder_option == MODE_MENU_ENCODER_PID) {
            BT_SendString("Selected encoder option 8: PID\r\n");
            PidMenu_ReportGains("VALUES");
        } else {
            BT_Printf("Selected encoder option %u: %s (NOT IMPLEMENTED; MOTOR LOCKED)\r\n",
                      (unsigned)(menu->active_encoder_option + 1U),
                      ModeMenu_EncoderName(menu->active_encoder_option));
        }
        break;
    case MODE_MENU_EVENT_ENCODER_EXIT:
        BT_SendString("Encoder Test: back to System Test\r\n");
        break;
    default:
        break;
    }
}

/* Runs in main context on every menu state change. Initializes every
 * driver once at boot, but actively samples only the selected sensors.
 * The encoder timers and 1kHz watchdog remain alive for safety.
 * Motor permission is a capability, not an automatic actuator unlock. */
static void ModeMenu_ApplyResources(const ModeMenu *menu,
                                    MPU6050_Handle_t *mpu,
                                    MPU6050_Status mpu_status,
                                    uint8_t *last_mask)
{
    uint8_t next = ModeTest_Resources(menu);
    if (next == *last_mask) return;
    Control_SetTestResources(next);
    if ((next & TEST_RES_MPU) && !(*last_mask & TEST_RES_MPU) &&
        mpu_status == MPU6050_OK && !mpu->calibrated && !mpu->calibrating) {
        (void)MPU6050_Calibrate(mpu); /* nonblocking MPU/All test only */
    }
    *last_mask = next;
    BT_Printf("TEST,RESOURCES,IR=%u,MPU=%u,ENC=%u,MOTOR=%u\r\n",
              (unsigned)((next & TEST_RES_IR) != 0U),
              (unsigned)((next & TEST_RES_MPU) != 0U),
              (unsigned)((next & TEST_RES_ENCODER) != 0U),
              (unsigned)((next & TEST_RES_MOTOR) != 0U));
}

#endif

#if !defined(MODE_MENU_ONLY)
static void HandleBleCommand(char c)
{
#if defined(INTEGRATION_DIAG)
    /* Diagnostic profiles exclude other BLE motion commands. */
    if (c != 's' && c != 'S') return;
    Sweep_Abort(SWEEP_STOP_REQUEST);
#endif
    MPU6050_Snapshot_t snap;
    uint8_t ready = MPU6050_GetSnapshot(&mpu, &snap) &&
                    (uint32_t)(micros() - snap.completed_us) <= 3000U;
    float yaw = ready ? snap.yaw_deg : 0.0f;

    switch (c) {
    case 'f':
    case 'F':
        if (ready && Motion_IsDone()) Motion_CommandStraight(180.0f, yaw);
        break;

    case 'l':
    case 'L':
        if (ready && Motion_IsDone()) Motion_CommandTurn(90.0f, yaw);
        break;

    case 'r':
    case 'R':
        if (ready && Motion_IsDone()) Motion_CommandTurn(-90.0f, yaw);
        break;

    case 's':
    case 'S':
        Motion_Stop();
        break;

    case 'z':
    case 'Z':
        if (ready && Motion_IsDone()) MPU6050_ResetYaw(&mpu);
        break;

    default:
        break;
    }
}
#endif /* !MODE_MENU_ONLY */

#if defined(ENCODER_ALPHA_AB_DIAG)
/* Only called in main, never from TIM11. Six short metrics frames per stage,
 * scheduled across subsequent main iterations to protect UART DMA queue. */
static void Encoder_LogShadowMetric(uint32_t now, uint32_t stage,
                                    uint8_t state, uint8_t line,
                                    const Encoder_ShadowSnapshot_t *snap)
{
    const uint32_t *arr;
    uint32_t n;
    uint8_t wheel = line / 3U;
    uint8_t metric = line % 3U;
    if (wheel == 0U) {
        arr = metric == 0U ? snap->window_abs_error_left :
              metric == 1U ? snap->transient_abs_error_left :
                             snap->variation_left;
    } else {
        arr = metric == 0U ? snap->window_abs_error_right :
              metric == 1U ? snap->transient_abs_error_right :
                             snap->variation_right;
    }
    n = metric == 1U ? snap->valid_reference_ticks : snap->window_ticks;
    UART_Printf("SYS_AM,ms=%lu,id=%lu,state=%u,w=%u,m=%u,n=%lu,x0=%lu,x1=%lu,x2=%lu,x3=%lu,x4=%lu,x5=%lu,x6=%lu,x7=%lu,x8=%lu,x9=%lu\r\n",
                (unsigned long)now, (unsigned long)stage,
                (unsigned)state, (unsigned)wheel, (unsigned)metric,
                (unsigned long)n, (unsigned long)arr[0],
                (unsigned long)arr[1], (unsigned long)arr[2],
                (unsigned long)arr[3], (unsigned long)arr[4],
                (unsigned long)arr[5], (unsigned long)arr[6],
                (unsigned long)arr[7], (unsigned long)arr[8],
                (unsigned long)arr[9]);
}
static void Encoder_LogShadowInstant(uint32_t now,
                                     const Encoder_ShadowSnapshot_t *snap)
{
    uint8_t w;
    for (w = 0U; w < 2U; ++w) {
        const int32_t *a = w == 0U ? snap->alpha_left : snap->alpha_right;
        const int32_t *box = w == 0U ? snap->box_left : snap->box_right;
        UART_Printf("SYS_AB,ms=%lu,w=%u,a05=%ld,a08=%ld,a10=%ld,a12=%ld,a15=%ld,a20=%ld,a30=%ld,a40=%ld,a60=%ld,a80=%ld,b5=%ld,b10=%ld,b20=%ld\r\n",
                    (unsigned long)now, (unsigned)w,
                    (long)a[0], (long)a[1], (long)a[2],
                    (long)a[3], (long)a[4], (long)a[5], (long)a[6],
                    (long)a[7], (long)a[8], (long)a[9],
                    (long)box[0], (long)box[1], (long)box[2]);
    }
}
#endif

int Main_Runtime(void)
{
#if !defined(MODE_MENU_ONLY)
    uint32_t last_debug_ms = 0;
#else
    ModeMenu mode_menu;
    uint8_t active_test_previous = MODE_MENU_NO_SELECTION;
    uint32_t last_test_ms = 0U;
#endif
#if defined(INTEGRATION_DIAG)
    uint32_t last_system_diag_ms = 0U;
#if defined(ENCODER_PROFILE_DIAG)
    uint32_t last_encoder_diag_ms = 0U;
#if defined(ENCODER_ALPHA_AB_DIAG)
    uint32_t last_shadow_diag_ms = 0U, shadow_last_emit_ms = 0U;
    uint32_t shadow_last_i2c_diag_ms = 0U;
    uint32_t shadow_completed_stage = 0U;
    uint8_t shadow_completed_state = 0U;
    uint8_t shadow_metric_next = 6U;
    Encoder_ShadowSnapshot_t shadow_completed;
#endif
#endif
    uint32_t last_ir_value_ms = 0U;
    uint32_t last_ir_value_scan = 0U;
    uint8_t last_cal_state = 0U;
    uint32_t last_sweep_stage = 0xFFFFFFFFUL;
    uint8_t last_sweep_state = 0xFFU;
    int32_t last_sweep_enc_l = 0, last_sweep_enc_r = 0;
    uint32_t last_sweep_log_ms = 0U;
    uint32_t coast_start_ms = 0U, coast_last_ms = 0U;
#if defined(ENCODER_SPEED_PID_DIAG)
    uint32_t last_pid_log_ms = 0U;
    int32_t last_pid_count_l = 0, last_pid_count_r = 0;
#endif
    int32_t coast_start_l = 0, coast_start_r = 0;
#endif
    MPU6050_Status mpu_status;
#if defined(MODE_MENU_ONLY)
    uint8_t test_resource_mask = 0U;
    uint32_t last_onebox_log_ms = 0U;
    IRC_Context ir_calibration;
    uint8_t mpu_ready_reported = 0U;
    uint32_t mpu_health_ms = 0U;
#endif

    Board_Init();
    SystemTimer_Init();
    UART_Init(); /* USART1 PA9 DMA TX and PA10 IRQ RX. */

    /* Fail closed BEFORE Motor_Init on HSI/PLL readiness or clock-profile
     * failure. Peripheral baud/timer setup below depends on the selected
     * 96MHz profile having been initialized successfully. */
    if (Board_GetClockFault() != 0U) {
        Board_StatusLed(1U);
        UART_Printf("BOOT_FAIL,CLOCK,fault=%lu,sysclk_nominal=%lu,trim=%lu\r\n",
                    (unsigned long)Board_GetClockFault(),
                    (unsigned long)SystemCoreClock,
                    (unsigned long)Board_GetHsiTrim());
        while (1) { __WFI(); } /* Motors NEVER initialized/enabled. */
    }

    UART_Printf("CLOCK_PROFILE,nominal_hz=%lu,trim=%lu,hsical=%lu,pclk1=%lu,pclk2=%lu,tim2clk=%lu,tim11clk=%lu\r\n",
                (unsigned long)SystemCoreClock,
                (unsigned long)Board_GetHsiTrim(),
                (unsigned long)Board_GetHsiCal(),
                (unsigned long)Board_GetAPB1ClockHz(),
                (unsigned long)Board_GetAPB2ClockHz(),
                (unsigned long)Board_GetAPB1TimerClockHz(),
                (unsigned long)Board_GetAPB2TimerClockHz());

    Motor_Init();
#if defined(MODE_MENU_ONLY)
    /* Unified runtime: boot LOCKED; only Start in 1 Box Forward can
     * unlock a single bounded encoder motion. Other modes never actuate. */
    Motor_LockOutputs();
#endif
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    /* Hard latch for motor-OFF sensor/clock smoke test: no rearm until reset. */
    Motor_LockOutputs();
#endif
    Encoder_Init();

    /* MPU6050_Init owns RC2 I2C bus probing and initialization. */
    BT_Init();
    IR_Init();

    mpu_status = MPU6050_Init(&mpu);
#if defined(MODE_MENU_ONLY)
    ModeMenu_Init(&mode_menu, Board_ButtonPressed(), millis());
    {
        uint8_t oled_found = 0U;
        /* SPP-first safety: OLED is optional and not yet validated on the
         * robot. Never let OLED I2C probing block the menu/BT diagnostic.
         * An opt-in build may restore it after I2C hardware validation. */
#if defined(MODE_MENU_ENABLE_OLED)
        if (mpu_status == MPU6050_OK) {
            oled_found = (SSD1306_Init() == SSD1306_OK) ? 1U : 0U;
            if (oled_found) {
                ModeMenu_Draw(&mode_menu, 0U);
                SSD1306_UpdateScreen();
            }
        }
#endif
        OLED_SharedInit(oled_found);
        OLED_SharedEnable(oled_found);
        /* Windows may connect long after power-up: respond to each '?' in
         * main even when MPU6050 failed. No one-shot boot-only handshake. */
    }
#endif
    /* Start zero-rate calibration at boot, with motor outputs locked.
     * MPU acquisition stays live; only selected tests consume its data. */
#if defined(MODE_MENU_ONLY)
    if (mpu_status == MPU6050_OK) (void)MPU6050_Calibrate(&mpu);
#endif

    Motion_Init();
#if defined(INTEGRATION_DIAG)
    Sweep_Init();
#if defined(UART_PROFILE_DIAG)
    UART_Bench_Init();
#endif
#endif

    if (mpu_status != MPU6050_OK) {
        Motor_Stop();
        Board_StatusLed(1);
#if defined(MODE_MENU_ONLY)
        /* Degraded diagnostic profile: IR/encoder tests and SPP stay usable.
         * MPU Test explicitly reports valid=0; no I2C or motor commands. */
        BT_Printf("BOOT,DEGRADED,MPU_STATUS=%u,STEP=%u,I2C=%u\r\n",
                  (unsigned)mpu_status, (unsigned)mpu.init_step,
                  (unsigned)mpu.init_i2c_status);
#else
        BT_SendString("[BOOT] MPU6050 init/calibration failed\r\n");
#if defined(INTEGRATION_DIAG)
        UART_Printf("MPU_BOOT_FAIL,status=%u,step=%u,reg=%u,i2c=%u,got=%u\r\n",
                    (unsigned)mpu_status, (unsigned)mpu.init_step,
                    (unsigned)mpu.init_register, (unsigned)mpu.init_i2c_status,
                    (unsigned)mpu.init_read_value);
#endif
        while (1) {
        }
#endif
    }

    /*
     * TIM10 belongs to ir_sensor.c.
     * TIM11 belongs to control.c.
     * main.c only performs system orchestration.
     */
#if !defined(MODE_MENU_ONLY)
    IR_StartScan();
#endif
    Control_Init(&mpu);
#if defined(MODE_MENU_ONLY)
    EncoderOneBox_Init(); /* motor remains locked until guarded Start */
    IRC_Init(&ir_calibration);
#endif
    Control_Start();

#if !defined(MODE_MENU_ONLY)
    BT_SendString("\r\n[BOOT] MPU RC2 calibration started\r\n");
    BT_SendString("Commands: F=straight 180mm, L=left 90, R=right 90, S=stop, Z=zero yaw\r\n");
#endif
    UART_SendString("[BOOT] USART1 PA9 DMA TX; MPU RC2 sampling enabled\r\n");
#if defined(INTEGRATION_DIAG_MOTOR_OFF)
    UART_SendString("INTEGRATION_DIAG,MOTOR_OFF_SHADOW,OUTPUTS_LOCKED\r\n");
#elif defined(INTEGRATION_DIAG_MOTOR_ON)
#if defined(ENCODER_SPEED_PID_DIAG)
    UART_SendString("INTEGRATION_DIAG,SPEED_PID_LIVE,MAX_PWM_30_PERCENT,HOST_LEASE_750MS\r\n");
#else
    UART_SendString("INTEGRATION_DIAG,MOTOR_ON,UART_MODE_ON_AND_START,0_TO_100_PERCENT\r\n");
#endif
#endif

    while (1) {
        uint32_t now = millis();
#if defined(MODE_MENU_ONLY)
        if (mpu_status == MPU6050_OK)
            MPU6050_Update(&mpu); /* Keep async acquisition alive from boot. */
#else
        MPU6050_Update(&mpu); /* Main-context IRQ-driven Policy B. */
#endif

        if (BT_Available()) {
#if defined(MODE_MENU_ONLY)
            /* Safe SPP commands: ?, N/K, P/G, E/Y/I for sensor diagnostics.
             * Motors remain locked regardless of Bluetooth traffic. */
            char command = BT_ReceiveChar();
            ModeMenuEvent remote = MODE_MENU_EVENT_NONE;
            if (PidMenu_Consume(command, &mode_menu, now)) {
                /* gain configuration never enters the single-byte command dispatcher */
            } else if (command == '?') {
                BT_SendString("SPP CONNECTED\r\n");
                if (mpu_status == MPU6050_OK)
                    BT_SendString("BOOT,READY,MPU=OK\r\n");
                else
                    BT_Printf("BOOT,DEGRADED,MPU_STATUS=%u,STEP=%u,I2C=%u\r\n",
                              (unsigned)mpu_status, (unsigned)mpu.init_step,
                              (unsigned)mpu.init_i2c_status);
                BT_SendString("ONEBOX,PROFILE,ENABLED\r\n");
                BT_SendString("TEST,PROFILE,UNIFIED,IR,MPU,ENC,MOTOR\r\n");
                if (mode_menu.page == MODE_MENU_PAGE_ENCODER &&
                    mode_menu.active_encoder_option == MODE_MENU_ENCODER_PID) {
                    BT_SendString("Selected encoder option 8: PID\r\n");
                    PidMenu_ReportGains("VALUES");
                }
            } else if (command == 'N') {
                if (IRC_IsMotionActive(&ir_calibration)) IRC_Abort(&ir_calibration,"MENU_STOP");
                else {
                    if (EncoderOneBox_IsRunning()) EncoderOneBox_Abort();
                    remote = ModeMenu_Next(&mode_menu);
                }
            } else if (command == 'K') {
                if (IRC_IsMotionActive(&ir_calibration)) IRC_Abort(&ir_calibration,"MENU_STOP");
                else {
                    if (EncoderOneBox_IsRunning()) EncoderOneBox_Abort();
                    remote = ModeMenu_Select(&mode_menu);
                }
            } else if (command == 'H') {
                EncoderOneBox_HostPing(); /* lease only */
            } else if (command == 'P') {
                if (IRC_IsMotionActive(&ir_calibration)) IRC_Abort(&ir_calibration,"USER_STOP");
                else if (EncoderOneBox_IsRunning()) EncoderOneBox_Abort();
                else if (ModeMenu_PauseTest(&mode_menu)) {
                    BT_Printf("TEST STREAM STOPPED: %s\r\n",
                              ModeMenu_TestName(mode_menu.active_test));
                    ModeMenu_Draw(&mode_menu, 0U);
                }
            } else if (command == 'G') {
                if (ModeMenu_PivotTarget(&mode_menu)) {
                    int16_t target=ModeMenu_PivotTarget(&mode_menu);
                    if (mpu_status==MPU6050_OK && !mpu.calibrated && !mpu.calibrating) {
                        (void)MPU6050_Calibrate(&mpu);
                        mpu_ready_reported=0U;
                        BT_SendString("TEST,MPU,CALIB_RESTART,KEEP_STILL\r\n");
                    }
                    if (mpu_status==MPU6050_OK && mpu.calibrated &&
                        (test_resource_mask & (TEST_RES_MPU|TEST_RES_ENCODER|TEST_RES_MOTOR)) ==
                        (TEST_RES_MPU|TEST_RES_ENCODER|TEST_RES_MOTOR) &&
                        EncoderOneBox_StartPivot(&mpu,target))
                        BT_Printf("PIVOT,START,target=%d,ref_pps=%d,pwm_max=%d\r\n",(int)target,ONEBOX_SPEED_PPS,ONEBOX_MAX_PWM);
                    else BT_SendString("PIVOT,REJECT,reason=NOT_READY\r\n");
                } else if (ModeTest_OneBoxArmed(&mode_menu) &&
                    (test_resource_mask & (TEST_RES_ENCODER | TEST_RES_MOTOR)) ==
                    (TEST_RES_ENCODER | TEST_RES_MOTOR)) {
                    if (EncoderOneBox_Start()) {
                        BT_Printf("ONEBOX,START,target_l=%ld,target_r=%ld,pwm_max=%d,ref_pps=%d\r\n",
                                  (long)ONEBOX_TARGET_L, (long)ONEBOX_TARGET_R,
                                  (int)ONEBOX_MAX_PWM, (int)ONEBOX_SPEED_PPS);
                    } else {
                        BT_SendString("ONEBOX,REJECT,reason=MOTOR_OR_ENCODER_NOT_READY\r\n");
                    }
                } else if (mode_menu.active_test == MODE_MENU_TEST_MPU &&
                           !mpu.calibrated) {
                    if (mpu_status == MPU6050_OK && !mpu.calibrating) {
                        (void)MPU6050_Calibrate(&mpu);
                        mpu_ready_reported = 0U;
                        BT_SendString("TEST,MPU,CALIB_RESTART,KEEP_STILL\r\n");
                    }
                    BT_SendString("TEST,MPU,START_REJECTED,WAIT_FOR_READY\r\n");
                } else if (ModeMenu_ResumeTest(&mode_menu)) {
                    last_test_ms = now; /* next measurement in ~50 ms */
                    BT_Printf("TEST STREAM STARTED: %s\r\n",
                              ModeMenu_TestName(mode_menu.active_test));
                    ModeMenu_Draw(&mode_menu, 0U);
                }
            } else if (command == 'V') {
                if (!IRC_Save(&ir_calibration))
                    BT_SendString("CAL,SAVE_REJECT,reason=NOT_READY_OR_UNSAFE\r\n");
            } else if (command == 'E' && ModeMenu_CanResetSensor(&mode_menu, 1U)) {
                Encoder_Reset(); /* Atomic relative to TIM11. Motor locked. */
                BT_SendString("ENCODER RESET OK\r\n");
            } else if (command == 'Y' && ModeMenu_CanResetSensor(&mode_menu, 2U)) {
                if (mpu_status == MPU6050_OK && mpu.calibrated) {
                    MPU6050_ResetYaw(&mpu);
                    BT_SendString("MPU YAW RESET OK\r\n");
                } else {
                    if (mpu_status == MPU6050_OK && !mpu.calibrating) {
                        (void)MPU6050_Calibrate(&mpu);
                        mpu_ready_reported = 0U;
                        BT_SendString("TEST,MPU,CALIB_RESTART,KEEP_STILL\r\n");
                    }
                    BT_SendString("MPU YAW RESET REJECTED: NOT READY\r\n");
                }
            } else if (command == 'I' && ModeMenu_CanResetSensor(&mode_menu, 0U)) {
                /* Restart a live IR acquisition; this is NOT a calibration.
                 * Keep motor outputs locked and sampling independent of SPP. */
                IR_Stop();
                IR_StartScan();
                BT_SendString("IR SCAN RESTARTED\r\n");
            }
            if (remote != MODE_MENU_EVENT_NONE) {
                if (remote == MODE_MENU_EVENT_TEST_SELECT ||
                    (remote == MODE_MENU_EVENT_ENCODER_SELECT &&
                     mode_menu.active_test == MODE_MENU_TEST_ENCODER))
                    last_test_ms = now;
                if (remote==MODE_MENU_EVENT_MPU_SELECT) mpu_ready_reported=0U;
                ModeMenu_Report(&mode_menu, remote);
                ModeMenu_ApplyResources(&mode_menu, &mpu, mpu_status,
                                        &test_resource_mask);
                if (remote == MODE_MENU_EVENT_SELECT && mode_menu.page==MODE_MENU_PAGE_MAIN &&
                    mode_menu.selected==0U) IRC_Arm(&ir_calibration,now);
                ModeMenu_Draw(&mode_menu, remote == MODE_MENU_EVENT_SELECT);
            }
#else
            HandleBleCommand(BT_ReceiveChar());
#endif
        }
#if defined(MODE_MENU_ONLY)
        {
            uint8_t menu_key = Board_ButtonPressed();
            if (EncoderOneBox_SuppressMenuButton()) menu_key = 0U;
            ModeMenuEvent event = ModeMenu_Poll(&mode_menu, menu_key, now);
            if (event != MODE_MENU_EVENT_NONE) {
                if (EncoderOneBox_IsRunning()) EncoderOneBox_Abort();
                if (event == MODE_MENU_EVENT_TEST_SELECT ||
                    (event == MODE_MENU_EVENT_ENCODER_SELECT &&
                     mode_menu.active_test == MODE_MENU_TEST_ENCODER))
                    last_test_ms = now;
                if (event==MODE_MENU_EVENT_MPU_SELECT) mpu_ready_reported=0U;
                ModeMenu_Report(&mode_menu, event);
                ModeMenu_ApplyResources(&mode_menu, &mpu, mpu_status,
                                        &test_resource_mask);
                if (event == MODE_MENU_EVENT_SELECT && mode_menu.page==MODE_MENU_PAGE_MAIN &&
                    mode_menu.selected==0U) IRC_Arm(&ir_calibration,now);
                ModeMenu_Draw(&mode_menu, event == MODE_MENU_EVENT_SELECT);
            }
            /* IR, MPU, All, and Encoder Log Receive stream only their
             * selected telemetry. One-box motor remains guarded by Start. */
            if (active_test_previous != mode_menu.active_test) {
                active_test_previous = mode_menu.active_test;
                last_test_ms = now;
                mpu_ready_reported = 0U;
            }
            if ((mode_menu.active_test == MODE_MENU_TEST_MPU || ModeMenu_PivotTarget(&mode_menu)) &&
                !mpu.calibrated && (uint32_t)(now - mpu_health_ms) >= 1000U) {
                mpu_health_ms = now;
                BT_Printf("TEST,MPU,CAL,state=%u,samples=%u,retries=%u,reason=%u,fresh=%lu,errors=%lu,timeouts=%lu,busy=%lu\r\n",
                          (unsigned)mpu.cal_state, (unsigned)mpu.calibration_count,
                          (unsigned)mpu.cal_retries, (unsigned)mpu.cal_failure_reason,
                          (unsigned long)mpu.fresh_gyro_reads, (unsigned long)mpu.error_reads,
                          (unsigned long)mpu.timeout_reads, (unsigned long)mpu.busy_retries);
            }
            if ((mode_menu.active_test == MODE_MENU_TEST_MPU || ModeMenu_PivotTarget(&mode_menu)) && !mpu_ready_reported &&
                mpu_status == MPU6050_OK && mpu.calibrated) {
                mpu_ready_reported = 1U;
                BT_SendString(ModeMenu_PivotTarget(&mode_menu) ? "TEST,MPU,READY,PIVOT,START_ON_PC\r\n" : "TEST,MPU,READY,ANGLE_ONLY,ZERO_YAW_THEN_START\r\n");
            }
            if (mode_menu.active_test != MODE_MENU_NO_SELECTION &&
                mode_menu.stream_running &&
                (uint32_t)(now - last_test_ms) >= 50U) {
                last_test_ms = now;
                if (mode_menu.active_test == 0U) {
                    IR_Data_t ir;
                    uint32_t age_us;
                    Control_GetIRSnapshot(&ir);
                    age_us = ir.scan_count ? (uint32_t)(micros() - ir.completed_us) : 0U;
                    BT_Printf("IR,ms=%lu,valid=%u,age_us=%lu,scan=%lu,L0=%u,L45=%u,L90=%u,R0=%u,R45=%u,R90=%u\r\n",
                              (unsigned long)now,
                              (unsigned)(ir.scan_count != 0U && age_us <= 5000U),
                              (unsigned long)age_us, (unsigned long)ir.scan_count,
                              (unsigned)ir.value[IR_L0], (unsigned)ir.value[IR_L45],
                              (unsigned)ir.value[IR_L90], (unsigned)ir.value[IR_R0],
                              (unsigned)ir.value[IR_R45], (unsigned)ir.value[IR_R90]);
                } else if (mode_menu.active_test == 1U) {
                    /* On-wire CSV: sensor tag, ENC_L, ENC_R.
                     * Only the two signed cumulative quadrature counts.
                     * The dashboard adds host timestamp and strips the tag. */
                    int32_t left = Encoder_GetLeftCount();
                    int32_t right = Encoder_GetRightCount();
                    BT_Printf("ENC,%ld,%ld\r\n", (long)left, (long)right);
                } else if (mode_menu.active_test == MODE_MENU_TEST_ALL) {
                    IR_Data_t ir;
                    MPU6050_Snapshot_t gyro;
                    uint32_t age_us;
                    uint8_t imu_valid = 0U;
                    int32_t yaw_cd = 0, gz_cd = 0;
                    Control_GetIRSnapshot(&ir);
                    age_us = ir.scan_count ? (uint32_t)(micros() - ir.completed_us) : 0U;
                    if (mpu_status == MPU6050_OK &&
                        MPU6050_GetSnapshot(&mpu, &gyro) && gyro.valid &&
                        (uint32_t)(micros() - gyro.completed_us) <= 5000U) {
                        imu_valid = 1U;
                        yaw_cd = (int32_t)(gyro.yaw_deg * 100.0f);
                        gz_cd = (int32_t)(gyro.gyro_z_dps * 100.0f);
                    }
                    BT_Printf("ALL,ms=%lu,enc_l=%ld,enc_r=%ld,ir_valid=%u,L0=%u,L45=%u,L90=%u,R0=%u,R45=%u,R90=%u,mpu_valid=%u,yaw_cd=%ld,gz_cd=%ld\r\n",
                              (unsigned long)now,
                              (long)Encoder_GetLeftCount(),
                              (long)Encoder_GetRightCount(),
                              (unsigned)(ir.scan_count != 0U && age_us <= 5000U),
                              (unsigned)ir.value[IR_L0], (unsigned)ir.value[IR_L45],
                              (unsigned)ir.value[IR_L90], (unsigned)ir.value[IR_R0],
                              (unsigned)ir.value[IR_R45], (unsigned)ir.value[IR_R90],
                              (unsigned)imu_valid, (long)yaw_cd, (long)gz_cd);
                } else if (mode_menu.active_test == MODE_MENU_TEST_MPU) {
                    MPU6050_Snapshot_t gyro;
                    uint8_t valid = (mpu_status == MPU6050_OK) ?
                                    MPU6050_GetSnapshot(&mpu, &gyro) : 0U;
                    uint32_t age_us = valid ?
                                      (uint32_t)(micros() - gyro.completed_us) : 0U;
                    if (valid && gyro.valid && age_us <= 3000U) {
                        BT_Printf("MPU,ms=%lu,valid=1,age_us=%lu,yaw_cd=%ld,gz_cd=%ld\r\n",
                                  (unsigned long)now, (unsigned long)age_us,
                                  (long)(gyro.yaw_deg * 100.0f),
                                  (long)(gyro.gyro_z_dps * 100.0f));
                    } else {
                        BT_Printf("MPU,ms=%lu,valid=0,age_us=%lu\r\n",
                                  (unsigned long)now, (unsigned long)age_us);
                    }
                }
            }
            IRC_Update(&ir_calibration,&mpu,now);
            {
                EncoderOneBoxTelemetry motion;
                EncoderOneBoxResult result;
                uint8_t has_motion = EncoderOneBox_GetTelemetry(&motion);
                uint8_t has_result = EncoderOneBox_PopResult(&result);
                /* Only print from MAIN at <=50Hz; no UART in TIM11.
                 * Emit an additional final zero-PWM sample before RESULT,
                 * ensuring Stop, done and all faults close a complete CSV. */
                if (has_result && !has_motion) {
                    motion.pivot_target = result.pivot_target;
                    motion.imu_valid=0U; motion.age_us=0U; motion.yaw=motion.angle=motion.gz=0.0f;
                    motion.elapsed_ms = result.elapsed_ms;
                    motion.left = Encoder_GetLeftCount();
                    motion.right = Encoder_GetRightCount();
                    motion.pwm_l = 0;
                    motion.pwm_r = 0;
                    motion.ref_l = motion.ref_r = 0;
                    motion.speed_l = (int32_t)Encoder_GetLeftSpeedPPS();
                    motion.speed_r = (int32_t)Encoder_GetRightSpeedPPS();
                    has_motion = 1U;
                }
                if (has_motion && (has_result ||
                    (uint32_t)(now - last_onebox_log_ms) >= 20U)) {
                    last_onebox_log_ms = now;
                    if (motion.pivot_target) {
                        BT_Printf("PIVOT_CSV,ms=%lu,valid=%u,age_us=%lu,yaw_cd=%ld,angle_cd=%ld,gz_cd=%ld,target=%d,left=%ld,right=%ld,pwm_l=%d,pwm_r=%d,ref_l=%ld,ref_r=%ld,speed_l=%ld,speed_r=%ld\r\n",
                            (unsigned long)now,(unsigned)motion.imu_valid,(unsigned long)motion.age_us,
                            (long)(motion.yaw*100),(long)(motion.angle*100),(long)(motion.gz*100),(int)motion.pivot_target,
                            (long)motion.left,(long)motion.right,(int)motion.pwm_l,(int)motion.pwm_r,
                            (long)motion.ref_l,(long)motion.ref_r,(long)motion.speed_l,(long)motion.speed_r);
                    } else BT_Printf("ONEBOX_CSV,ms=%lu,left=%ld,right=%ld,pwm_l=%d,pwm_r=%d,ref_l=%ld,ref_r=%ld,speed_l=%ld,speed_r=%ld\r\n",
                              (unsigned long)motion.elapsed_ms,
                              (long)motion.left, (long)motion.right,
                              (int)motion.pwm_l, (int)motion.pwm_r,
                              (long)motion.ref_l, (long)motion.ref_r,
                              (long)motion.speed_l, (long)motion.speed_r);
                }
                if (has_result) {
                    IRC_OnPivotResult(&ir_calibration,&result,now);
                    BT_Printf("%s,RESULT,reason=%s,L=%ld,R=%ld,ms=%lu,locked=%u\r\n",
                              result.pivot_target ? "PIVOT" : "ONEBOX",
                              EncoderOneBox_Reason(result.reason),
                              (long)result.count_l, (long)result.count_r,
                              (unsigned long)result.elapsed_ms,
                              (unsigned)result.motor_locked);
                }
            }
            OLED_SharedService(&mpu);
            /* Idle menu: no periodic status; active test: 20 Hz CSV data. */
        }
#endif
#if defined(INTEGRATION_DIAG)
        /* RX IRQ only queues bytes; command parsing is budgeted in main. */
        IntegrationSerial_Poll(&mpu, now);
        /* Accept any already-buffered PING before the unchanged 750-ms lease. */
        Sweep_Update(now, &mpu);
#if defined(ENCODER_ALPHA_AB_DIAG)
        /* No shadow EMA operations run at high interrupt priority. */
        Encoder_ShadowService(12U);
#endif
#if defined(UART_PROFILE_DIAG)
        /* Main-only TX generator, never in TIM11 or USART ISR. */
        UART_Bench_Poll(now, &mpu);
#endif
#else
        /* Production UART diagnostic remains STOP/status only. */
        {
            uint8_t ch, budget = 16U;
            while (budget-- && UART_ReadByte(&ch)) {
                if (ch == 'S' || ch == 's') Motion_Stop();
                if (ch == '?')
                    UART_Printf("DBG,rx_overflow=%lu,rx_error=%lu,tx_drop=%lu\r\n",
                                (unsigned long)UART_GetRxOverflow(),
                                (unsigned long)UART_GetRxErrors(),
                                (unsigned long)UART_GetDroppedMessages());
            }
        }
#endif

#if defined(INTEGRATION_DIAG)
        /* One diagnostic bundle per second. UART prints only in main, never
         * from TIM11/I2C/TIM10 interrupts. Host must analyze *deltas* of
         * monotonic counters; clock values remain RCC-derived nominal. */
        {
            SweepSnapshot sw;
            Sweep_GetSnapshot(&sw);
            if (sw.state != SWEEP_DISARMED &&
                (sw.stage_id != last_sweep_stage || sw.state != last_sweep_state)) {
                int32_t left = Encoder_GetLeftCount();
                int32_t right = Encoder_GetRightCount();
                int16_t applied_l = 0, applied_r = 0;
                uint32_t dt_ms = last_sweep_log_ms ?
                    (uint32_t)(now - last_sweep_log_ms) : 0U;
                Motor_GetAppliedPairPermille(&applied_l, &applied_r);
#if defined(ENCODER_ALPHA_AB_DIAG)
                /* Consume queued 1-ms deltas before stage metric snapshot. */
                Encoder_ShadowService(32U);
                Encoder_ShadowMarkStageBoundary();
                if (last_sweep_stage != 0xFFFFFFFFUL) {
                    Encoder_GetShadowSnapshot(&shadow_completed);
                    shadow_completed_stage = last_sweep_stage;
                    shadow_completed_state = last_sweep_state;
                    shadow_metric_next = 0U;
                    shadow_last_emit_ms = now - 25U;
                }
                Encoder_ShadowResetMetrics();
#endif
                /* state=2 denotes END OF DRIVE; state=1 with id>0
                 * denotes END OF PREVIOUS STAGE'S COAST. Both dl/dr
                 * and elapsed dt_ms refer to the preceding interval. */
                UART_Printf("SYS_STEP,ms=%lu,state=%u,fault=%u,mode=%u,id=%lu,group=%u,step=%u,duty=%u,dt=%lu,dl=%ld,dr=%ld,vL=%ld,vR=%ld,appL=%d,appR=%d\r\n",
                            (unsigned long)now, (unsigned)sw.state,
                            (unsigned)sw.fault, (unsigned)sw.motor_enabled,
                            (unsigned long)sw.stage_id, (unsigned)sw.group,
                            (unsigned)sw.step, (unsigned)sw.duty_permille,
                            (unsigned long)dt_ms,
                            (long)(left - last_sweep_enc_l),
                            (long)(right - last_sweep_enc_r),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS(),
                            (int)applied_l, (int)applied_r);
                last_sweep_stage = sw.stage_id;
                last_sweep_state = sw.state;
                last_sweep_enc_l = left;
                last_sweep_enc_r = right;
                last_sweep_log_ms = now;
#if defined(ENCODER_ALPHA_AB_DIAG)
                /* Completed interval is queued above; first frame is sent
                 * outside this transition bundle at the normal main cadence. */
#endif
                if (sw.state == SWEEP_COAST) {
                    coast_start_ms = coast_last_ms = now;
                    coast_start_l = left;
                    coast_start_r = right;
                }
            }
            /* 500-ms samples during each 2–3s coast interval. These
             * capture transient encoder movement and the filtered speed
             * approaching zero without doing I/O in TIM11. */
            if (sw.state == SWEEP_COAST &&
                (uint32_t)(now - coast_last_ms) >= 500U) {
                coast_last_ms = now;
                UART_Printf("SYS_COAST,ms=%lu,id=%lu,dt=%lu,dl=%ld,dr=%ld,ppsL=%ld,ppsR=%ld\r\n",
                            (unsigned long)now,
                            (unsigned long)sw.stage_id,
                            (unsigned long)(now - coast_start_ms),
                            (long)(Encoder_GetLeftCount() - coast_start_l),
                            (long)(Encoder_GetRightCount() - coast_start_r),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS());
            }
        }
#if defined(ENCODER_SPEED_PID_DIAG)
        /* 20Hz log, discrete encoder count delta is a separate temporal
         * measurement from the 1-kHz EMA used by the PID feedback. */
        if ((uint32_t)(now - last_pid_log_ms) >= 50U) {
            SweepSnapshot sw;
            SpeedPidDiagSnapshot pid;
            int16_t applied_l = 0, applied_r = 0;
            int32_t count_l = Encoder_GetLeftCount();
            int32_t count_r = Encoder_GetRightCount();
            uint32_t dt = last_pid_log_ms ?
                          (uint32_t)(now - last_pid_log_ms) : 0U;
            Sweep_GetSnapshot(&sw);
            SpeedPidDiag_Get(&pid);
            Motor_GetAppliedPairPermille(&applied_l, &applied_r);
            if (last_pid_log_ms && sw.state == SWEEP_DRIVE && dt > 0U) {
                UART_Printf("SYS_PID,ms=%lu,id=%lu,group=%u,step=%u,target=%u,dt=%lu,refL=%ld,refR=%ld,emaL=%ld,emaR=%ld,pwmL=%u,pwmR=%u,appL=%d,appR=%d\r\n",
                    (unsigned long)now, (unsigned long)sw.stage_id,
                    (unsigned)sw.group, (unsigned)sw.step,
                    (unsigned)pid.target_pps, (unsigned long)dt,
                    (long)((count_l-last_pid_count_l)*1000L/(int32_t)dt),
                    (long)((count_r-last_pid_count_r)*1000L/(int32_t)dt),
                    (long)Encoder_GetLeftSpeedPPS(),
                    (long)Encoder_GetRightSpeedPPS(),
                    (unsigned)pid.last_pwm_left, (unsigned)pid.last_pwm_right,
                    (int)applied_l, (int)applied_r);
            }
            last_pid_log_ms = now;
            last_pid_count_l = count_l;
            last_pid_count_r = count_r;
        }
#endif
        if ((uint32_t)(now - last_system_diag_ms) >= 1000U) {
            Control_DebugStats_t sd;
            Motor_Diagnostics_t md;
            MPU6050_Snapshot_t snap = {0};
            uint8_t valid = MPU6050_GetSnapshot(&mpu, &snap);
            uint32_t age = valid ?
                (uint32_t)(micros() - snap.completed_us) : 0U;
            last_system_diag_ms = now;
            Control_GetDebugStats(&sd);
            Motor_GetDiagnostics(&md);

            UART_Printf("SYS_T,ms=%lu,ticks=%lu,late=%lu,gap=%lu,maxcy=%lu,encmaxcy=%lu,motmaxcy=%lu,motticks=%lu,valid=%lu,stale=%lu,ir=%lu\r\n",
                        (unsigned long)now, (unsigned long)sd.ticks,
                        (unsigned long)sd.late_gt50us,
                        (unsigned long)sd.gaps_gt1500us,
                        (unsigned long)sd.max_irq_cycles,
                        (unsigned long)sd.max_encoder_cycles,
                        (unsigned long)sd.max_motor_service_cycles,
                        (unsigned long)sd.motor_service_ticks,
                        (unsigned long)sd.gyro_valid_ticks,
                        (unsigned long)sd.gyro_invalid_after_first_valid,
                        (unsigned long)sd.ir_scan_count);
            UART_Printf("SYS_IR,ms=%lu,new=%lu,old=%lu,age_us=%lu,age_max_us=%lu\r\n",
                        (unsigned long)now,
                        (unsigned long)sd.ir_fresh_ticks,
                        (unsigned long)sd.ir_no_new_ticks,
                        (unsigned long)sd.ir_last_age_us,
                        (unsigned long)sd.ir_max_age_us);
            {
                IR_Diagnostics_t ir;
                IR_GetDiagnostics(&ir);
                UART_Printf("SYS_IRD,ms=%lu,start=%lu,done=%lu,fail=%lu,busy=%lu,dma_start=%lu,dma_done=%lu,dma_err=%lu,dma_to=%lu,adc_ovr=%lu,scan_us=%lu,scan_max_us=%lu,dma_us=%lu,dma_max_us=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)ir.scans_started,
                            (unsigned long)ir.scans_completed,
                            (unsigned long)ir.scans_failed,
                            (unsigned long)ir.start_busy,
                            (unsigned long)ir.dma_started,
                            (unsigned long)ir.dma_completed,
                            (unsigned long)ir.dma_errors,
                            (unsigned long)ir.dma_timeouts,
                            (unsigned long)ir.adc_overruns,
                            (unsigned long)ir.scan_total_us,
                            (unsigned long)ir.scan_max_us,
                            (unsigned long)ir.dma_total_us,
                            (unsigned long)ir.dma_max_us);
                UART_Printf("SYS_IRC,ms=%lu,tim_irq=%lu,tim_cy=%lu,tim_maxcy=%lu,dma_irq=%lu,dma_cy=%lu,dma_maxcy=%lu,adc_clk=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)ir.tim10_irq_count,
                            (unsigned long)ir.tim10_irq_total_cycles,
                            (unsigned long)ir.tim10_irq_max_cycles,
                            (unsigned long)ir.dma_irq_count,
                            (unsigned long)ir.dma_irq_total_cycles,
                            (unsigned long)ir.dma_irq_max_cycles,
                            (unsigned long)(Board_GetAPB2ClockHz() / 4U));
            }
            UART_Printf("SYS_M,ms=%lu,cal=%u,n=%u,seq=%lu,age=%lu,i2cok=%lu,i2cerr=%lu,to=%lu,nr=%lu,busy=%lu,locked=%u,wd=%u\r\n",
                        (unsigned long)now,
                        (unsigned)mpu.cal_state,
                        (unsigned)mpu.calibration_count,
                        (unsigned long)snap.sequence,
                        (unsigned long)age,
                        (unsigned long)mpu.fresh_gyro_reads,
                        (unsigned long)mpu.error_reads,
                        (unsigned long)mpu.timeout_reads,
                        (unsigned long)mpu.status_not_ready_reads,
                        (unsigned long)mpu.busy_retries,
                        (unsigned)md.outputs_locked,
                        (unsigned)md.command_timeout_latched);
            UART_Printf("SYS_U,ms=%lu,tx_drop=%lu,dma_err=%lu,rx_err=%lu,rx_ov=%lu,trim=%lu,clkfault=%lu,iclk=%lu\r\n",
                        (unsigned long)now,
                        (unsigned long)UART_GetDroppedMessages(),
                        (unsigned long)UART_GetDmaErrors(),
                        (unsigned long)UART_GetRxErrors(),
                        (unsigned long)UART_GetRxOverflow(),
                        (unsigned long)Board_GetHsiTrim(),
                        (unsigned long)Board_GetClockFault(),
                        (unsigned long)I2C_GetConfiguredSclHz());
#if defined(ENCODER_ALPHA_AB_DIAG)
            {
                Encoder_ShadowQueueStats_t q;
                Encoder_GetShadowQueueStats(&q);
                UART_Printf("SYS_AQ,ms=%lu,in=%lu,out=%lu,lost=%lu,edge=%lu,q=%u,peak=%u\r\n",
                            (unsigned long)now,
                            (unsigned long)q.produced,
                            (unsigned long)q.consumed,
                            (unsigned long)q.overflow,
                            (unsigned long)q.stage_boundary_lag,
                            (unsigned)q.pending, (unsigned)q.peak_pending);
            }
            {
                UART_ProfileStats_t diag;
                UART_GetProfileStats(&diag);
                UART_Printf("SYS_UR,ms=%lu,err=%lu,ore=%lu,fe=%lu,ne=%lu,ov=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)diag.rx_errors,
                            (unsigned long)diag.rx_overrun_errors,
                            (unsigned long)diag.rx_framing_errors,
                            (unsigned long)diag.rx_noise_errors,
                            (unsigned long)diag.rx_overflow);
            }
#endif
#if defined(ENCODER_PROFILE_DIAG)
#if defined(ENCODER_ALPHA_AB_DIAG)
            /* Boot probe is retained, not changing each second. Emit every
             * 10s to prevent an annual 1Hz burst with 50Hz SYS_EN. */
            if (shadow_last_i2c_diag_ms == 0U ||
                (uint32_t)(now - shadow_last_i2c_diag_ms) >= 10000U) {
                shadow_last_i2c_diag_ms = now;
#endif
            /* Report retained MPU init probe result periodically.
             * For a 300k fallback, reveals whether 400k failed address ACK,
             * WHO_AM_I read, or WHO_AM_I value. Observation only. */
            UART_Printf("SYS_IPB,ms=%lu,idx=%u,mode=%u,a0=%u,w0=%u,id0=%u,a1=%u,w1=%u,id1=%u,a2=%u,w2=%u,id2=%u,a3=%u,w3=%u,id3=%u\r\n",
                        (unsigned long)now,
                        (unsigned)mpu.selected_speed_index,
                        (unsigned)mpu.selected_speed_mode,
                        (unsigned)mpu.probe_address_status[0],
                        (unsigned)mpu.probe_who_status[0],
                        (unsigned)mpu.probe_who_value[0],
                        (unsigned)mpu.probe_address_status[1],
                        (unsigned)mpu.probe_who_status[1],
                        (unsigned)mpu.probe_who_value[1],
                        (unsigned)mpu.probe_address_status[2],
                        (unsigned)mpu.probe_who_status[2],
                        (unsigned)mpu.probe_who_value[2],
                        (unsigned)mpu.probe_address_status[3],
                        (unsigned)mpu.probe_who_status[3],
                        (unsigned)mpu.probe_who_value[3]);
            {
                I2C_BusyDiag_t ib;
                I2C_GetBusyDiag(&ib);
                UART_Printf("SYS_IB,ms=%lu,n=%lu,rec=%lu,sr1=%lu,sr2=%lu,pin=%lu,after=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)ib.busy_timeouts,
                            (unsigned long)ib.bus_recoveries,
                            (unsigned long)ib.last_sr1,
                            (unsigned long)ib.last_sr2,
                            (unsigned long)ib.last_pb8_pb9_levels,
                            (unsigned long)ib.last_after_recovery_levels);
            }
#if defined(ENCODER_ALPHA_AB_DIAG)
            }
#endif
#endif
            /* Optional UART profiling also runs in Motor OFF/ON diagnostics.
             * The production image has no instrumentation or this log. */
            {
                UART_ProfileStats_t tx;
                UART_GetProfileStats(&tx);
                UART_Printf("SYS_UQ,ms=%lu,q=%lu,peak=%lu,accb=%lu,doneb=%lu,prncy=%lu,dmacy=%lu,rxcy=%lu,rxb=%lu\r\n",
                            (unsigned long)now,
                            (unsigned long)tx.queued_bytes,
                            (unsigned long)tx.queue_peak,
                            (unsigned long)tx.accepted_bytes,
                            (unsigned long)tx.completed_bytes,
                            (unsigned long)tx.printf_cycles,
                            (unsigned long)tx.tx_dma_isr_cycles,
                            (unsigned long)tx.rx_isr_cycles,
                            (unsigned long)tx.rx_bytes);
            }
            {
                int16_t app_l = 0, app_r = 0;
                Motor_GetAppliedPairPermille(&app_l, &app_r);
                UART_Printf("SYS_E,ms=%lu,encL=%ld,encR=%ld,ppsL=%ld,ppsR=%ld,appL=%d,appR=%d\r\n",
                            (unsigned long)now,
                            (long)Encoder_GetLeftCount(),
                            (long)Encoder_GetRightCount(),
                            (long)Encoder_GetLeftSpeedPPS(),
                            (long)Encoder_GetRightSpeedPPS(),
                            (int)app_l, (int)app_r);
            }
            if (mpu.cal_state != last_cal_state) {
                UART_Printf("SYS_CAL,ms=%lu,state=%u,retries=%u,reason=%u\r\n",
                            (unsigned long)now,
                            (unsigned)mpu.cal_state,
                            (unsigned)mpu.cal_retries,
                            (unsigned)mpu.cal_failure_reason);
                last_cal_state = mpu.cal_state;
            }
        }
#if defined(ENCODER_ALPHA_AB_DIAG)
        /* Ten candidate EMA filters still run at 1kHz in TIM11. UART
         * visualization is throttled to 5Hz and not allowed to crowd the
         * command/health TX queue. Do NOT throttle 1kHz updates themselves. */
        if ((uint32_t)(now - last_shadow_diag_ms) >= 200U) {
            UART_ProfileStats_t stats;
            UART_GetProfileStats(&stats);
            if (stats.queued_bytes < 768U) {
                Encoder_ShadowSnapshot_t snap;
                last_shadow_diag_ms = now;
                Encoder_GetShadowSnapshot(&snap);
                Encoder_LogShadowInstant(now, &snap);
            }
        }
        if (shadow_metric_next < 6U &&
            (uint32_t)(now - shadow_last_emit_ms) >= 25U) {
            UART_ProfileStats_t stats;
            UART_GetProfileStats(&stats);
            if (stats.queued_bytes < 768U) {
                Encoder_LogShadowMetric(now, shadow_completed_stage,
                                        shadow_completed_state,
                                        shadow_metric_next, &shadow_completed);
                shadow_last_emit_ms = now;
                ++shadow_metric_next;
            }
        }
#endif
#if defined(ENCODER_PROFILE_DIAG)
        /* Opt-in encoder telemetry at ~50Hz for OFF/ON diagnostic builds.
         * Sample a coherent pair in main,
         * never format UART text inside TIM11 and never touch motor commands.
         * This is decimated observation of TIM11's 1kHz encoder updates. */
        if ((uint32_t)(now - last_encoder_diag_ms) >= 20U) {
            uint32_t primask = __get_PRIMASK();
            int32_t left_count, right_count;
            int32_t left_pps, right_pps;
            last_encoder_diag_ms = now;
            __disable_irq();
            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            left_pps = (int32_t)Encoder_GetLeftSpeedPPS();
            right_pps = (int32_t)Encoder_GetRightSpeedPPS();
            if (!primask) __enable_irq();
            UART_Printf("SYS_EN,ms=%lu,l=%ld,r=%ld,vL=%ld,vR=%ld\r\n",
                        (unsigned long)now,
                        (long)left_count, (long)right_count,
                        (long)left_pps, (long)right_pps);
        }
#endif
        if ((uint32_t)(now - last_ir_value_ms) >= 1000U) {
            IR_Data_t ir_snapshot;
            last_ir_value_ms = now;
            Control_GetIRSnapshot(&ir_snapshot);
            if (ir_snapshot.scan_count != 0U &&
                ir_snapshot.scan_count != last_ir_value_scan) {
                IR_Diagnostics_t ir;
                uint32_t age = micros() - ir_snapshot.completed_us;
                IR_GetDiagnostics(&ir);
                if (ir.raw_scan_count == ir_snapshot.scan_count) {
                    UART_Printf("SYS_IRV,ms=%lu,scan=%lu,age_us=%lu,ambient_reads=%lu,pair1_reads=%lu,pair2_reads=%lu,pair3_reads=%lu,a0=%u,a1=%u,a2=%u,a3=%u,a4=%u,a5=%u,l0=%u,l1=%u,l2=%u,l3=%u,l4=%u,l5=%u,d0=%u,d1=%u,d2=%u,d3=%u,d4=%u,d5=%u\r\n",
                                (unsigned long)now,
                                (unsigned long)ir_snapshot.scan_count,
                                (unsigned long)age,
                                (unsigned long)ir.ambient_reads,
                                (unsigned long)ir.pair1_reads,
                                (unsigned long)ir.pair2_reads,
                                (unsigned long)ir.pair3_reads,
                                (unsigned)ir.ambient_raw[0],
                                (unsigned)ir.ambient_raw[1],
                                (unsigned)ir.ambient_raw[2],
                                (unsigned)ir.ambient_raw[3],
                                (unsigned)ir.ambient_raw[4],
                                (unsigned)ir.ambient_raw[5],
                                (unsigned)ir.emitter_raw[0],
                                (unsigned)ir.emitter_raw[1],
                                (unsigned)ir.emitter_raw[2],
                                (unsigned)ir.emitter_raw[3],
                                (unsigned)ir.emitter_raw[4],
                                (unsigned)ir.emitter_raw[5],
                                (unsigned)ir_snapshot.value[0],
                                (unsigned)ir_snapshot.value[1],
                                (unsigned)ir_snapshot.value[2],
                                (unsigned)ir_snapshot.value[3],
                                (unsigned)ir_snapshot.value[4],
                                (unsigned)ir_snapshot.value[5]);
                    last_ir_value_scan = ir_snapshot.scan_count;
                }
            }
        }

#endif

#if !defined(MODE_MENU_ONLY)
        if ((now - last_debug_ms) >= DEBUG_PERIOD_MS) {
            int32_t left_count;
            int32_t right_count;
            float yaw;
            float gyro;
            float distance;
            float angle_error;
            Motion_State_t state;
            uint32_t primask;

            last_debug_ms = now;

            /* Snapshot shared state quickly; formatting/transmission stays outside ISR. */
            primask = __get_PRIMASK();
            __disable_irq();

            left_count = Encoder_GetLeftCount();
            right_count = Encoder_GetRightCount();
            yaw = MPU6050_GetYaw(&mpu);
            gyro = MPU6050_GetGyroZ(&mpu);
            distance = Motion_GetDistanceMm();
            angle_error = Motion_GetAngleErrorDeg();
            state = Motion_GetState();

            if (!primask) {
                __enable_irq();
            }

            BT_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f d=%.1f e=%.2f\r\n",
                      (unsigned long)now,
                      (unsigned int)state,
                      (long)left_count,
                      (long)right_count,
                      yaw,
                      gyro,
                      distance,
                      angle_error);
#if !defined(INTEGRATION_DIAG)
            UART_Printf("t=%lu state=%u encL=%ld encR=%ld yaw=%.2f gz=%.2f dropped=%lu\r\n",
                        (unsigned long)now,(unsigned int)state,(long)left_count,
                        (long)right_count,yaw,gyro,(unsigned long)UART_GetDroppedMessages());
#endif
        }
#endif /* !MODE_MENU_ONLY */
    }
}


#endif
