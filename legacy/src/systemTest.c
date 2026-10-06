/**
 * @file systemTest.c
 * @brief System Test - FIXED: Continuous MPU update
 */

#include "fonts.h"
#include "hardware.h"
#include "ir_sensor.h"
#include "ir_simple_calib.h"
#include "motion_controller.h"
#include "mpu6050.h"
#include "sensor_fusion.h"
#include "ssd1306.h"
#include "system_timer.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

#define BTN_NONE 0
#define BTN_SHORT 1
#define BTN_LONG 2
#define LONG_PRESS_MS 400

/* T1WT tuning: extra advance after boundary = k * NUM / DEN (default = 1/3). */
#define T1WT_EXTRA_NUM 1.0f
#define T1WT_EXTRA_DEN 3.0f

extern Motion_Controller_t motion_ctrl;
extern Sensor_Fusion_t sensor_fusion;
extern IR_Simple_Calib_t ir_calib;

/* ================= HELPER FUNCTIONS ================= */

static uint8_t Check_Button_Action(void) {
  uint32_t start_time;

  if (Hardware_ReadButton(0)) {
    start_time = millis();
    delay_ms_blocking(20);
    if (!Hardware_ReadButton(0))
      return BTN_NONE;

    while (Hardware_ReadButton(0)) {
      if ((millis() - start_time) >= LONG_PRESS_MS) {
        return BTN_LONG;
      }
    }
    return BTN_SHORT;
  }
  return BTN_NONE;
}

static uint8_t Check_Button_Exit(void) {
  if (Hardware_ReadButton(0)) {
    delay_ms_blocking(20);
    if (Hardware_ReadButton(0)) {
      while (Hardware_ReadButton(0))
        ;
      return 1;
    }
  }
  return 0;
}

static void Perform_Countdown(void) {
  int8_t i;
  char buf[2];

  for (i = 3; i > 0; i--) {
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(30, 10);
    SSD1306_WriteString("Starting...", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "%d", i);
    SSD1306_SetCursor(60, 30);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    delay_ms_blocking(1000);
  }

  SSD1306_Fill(SSD1306_BLACK);
  SSD1306_SetCursor(40, 25);
  SSD1306_WriteString("Start !", &Font_11x18, SSD1306_WHITE);
  SSD1306_UpdateScreen();
  delay_ms_blocking(500);
}

static void Show_Header(const char *title) {
  SSD1306_Fill(SSD1306_BLACK);
  SSD1306_SetCursor(0, 0);
  SSD1306_WriteString((char *)title, &Font_7x10, SSD1306_WHITE);
  SSD1306_UpdateScreen();
}

static void Force_MPU_Calibration(void) {
  char buf[64];

  Show_Header("MPU6050 CALIB");
  SSD1306_SetCursor(0, 20);
  SSD1306_WriteString("Keep STILL!", &Font_11x18, SSD1306_WHITE);
  SSD1306_UpdateScreen();

  UART_SendString("\r\n=== MPU6050 Calibration ===\r\n");
  UART_SendString("Robot must be still!\r\n");

  MPU6050_Calibrate(&sensor_fusion.mpu_handle);
  MPU6050_ResetYaw(&sensor_fusion.mpu_handle);

  sprintf(buf, "Offset: %.2f deg/s\r\n",
          sensor_fusion.mpu_handle.gyro_z_offset);
  UART_SendString(buf);
  UART_SendString("Calibration done!\r\n");

  Show_Header("CALIB DONE!");
  delay_ms_blocking(1000);
}

/* ================= TEST MODULES ================= */

static void Test_MPU6050(void) {
  char buf[64];
  uint32_t last_display = 0;

  Perform_Countdown();
  Show_Header("Calibrating...");
  MPU6050_Calibrate(&sensor_fusion.mpu_handle);
  MPU6050_ResetYaw(&sensor_fusion.mpu_handle);

  UART_SendString("\r\n=== MPU6050 TEST ===\r\n");
  Motion_StartPID();

  while (1) {
    if (Check_Button_Exit()) {
      Motion_StopPID();
      UART_SendString("MPU test ended\r\n");
      return;
    }

    /* [CLEANUP] Sensors updated by TIM11 @ 1kHz */

    /* Non-blocking Display Output at 10Hz (100ms) */
    if (millis() - last_display > 100) {
      last_display = millis();

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("1. MPU6050", &Font_7x10, SSD1306_WHITE);

      sprintf(buf, "Yaw: %.2f", sensor_fusion.heading);
      SSD1306_SetCursor(0, 20);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);

      sprintf(buf, "Gz: %.2f", sensor_fusion.gyro_z);
      SSD1306_SetCursor(0, 45);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

      SSD1306_UpdateScreen();

      sprintf(buf, "Yaw=%.2f Gz=%.2f\r\n", sensor_fusion.heading,
              sensor_fusion.gyro_z);
      UART_SendString(buf);
    }
  }
}

static void Test_Encoders(void) {
  char buf[64];
  int32_t left, right;
  float speed_l, speed_r;

  Perform_Countdown();
  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  UART_SendString("\r\n=== ENCODER TEST ===\r\n");
  Motion_StartPID();

  while (1) {
    if (Check_Button_Exit()) {
      Motion_StopPID();
      UART_SendString("Encoder test ended\r\n");
      return;
    }

    /* [CLEANUP] Encoders updated by TIM11 @ 1kHz */

    left = Hardware_GetEncoderCount(1);
    right = Hardware_GetEncoderCount(0);
    speed_l = Hardware_GetEncoderSpeed(1);
    speed_r = Hardware_GetEncoderSpeed(0);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("2. ENCODER", &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "L: %ld", left);
    SSD1306_SetCursor(0, 15);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "R: %ld", right);
    SSD1306_SetCursor(0, 30);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "Spd: %.0f %.0f", speed_l, speed_r);
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    SSD1306_UpdateScreen();

    sprintf(buf, "L=%ld R=%ld SpdL=%.0f SpdR=%.0f\r\n", left, right, speed_l,
            speed_r);
    UART_SendString(buf);

    delay_ms_blocking(100);
  }
}

static void Test_IR_Sensors(void) {
  uint16_t ir[6];
  char buf[80];

  Perform_Countdown();

  if (!IR_Sensor_IsReady())
    IR_Sensor_StartScan();

  UART_SendString("\r\n=== IR SENSOR TEST (Raw + EMA Filtered) ===\r\n");
  Motion_StartPID(); /* Enable 1kHz loop for EMA filtering */

  while (1) {
    uint16_t filt[6];
    uint8_t i;
    float fsum, fdiff;

    if (Check_Button_Exit()) {
      Motion_StopPID();
      UART_SendString("IR test ended\r\n");
      return;
    }

    if (IR_Sensor_IsReady()) {
      IR_Sensor_GetResults(ir);
      IR_Sensor_StartScan();
    }

    /* Get filtered values from sensor fusion */
    for (i = 0; i < 6; i++) {
      filt[i] = (uint16_t)sensor_fusion.ir_sensors[i];
    }
    fsum = sensor_fusion.ir_sensors[0] + sensor_fusion.ir_sensors[5];
    fdiff = sensor_fusion.ir_sensors[0] - sensor_fusion.ir_sensors[5];

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("3.IR Raw/Filt", &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "L90:%4d/%4d", ir[0], filt[0]);
    SSD1306_SetCursor(0, 13);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "R90:%4d/%4d", ir[5], filt[5]);
    SSD1306_SetCursor(0, 26);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "L0:%4d R0:%4d", filt[2], filt[3]);
    SSD1306_SetCursor(0, 39);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    sprintf(buf, "L45:%d R45:%d", filt[1], filt[4]);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

    SSD1306_UpdateScreen();

    sprintf(buf, "L90=%d R90=%d L0=%d R0=%d L45=%d R45=%d\r\n", filt[0],
            filt[5], filt[2], filt[3], filt[1], filt[4]);
    UART_SendString(buf);

    delay_ms_blocking(100);
  }
}

static void Test_Straight_MPU(void) {
  char buf[64];
  uint32_t debug_timer = 0;
  uint8_t run_count = 0;
  uint8_t btn;

  Perform_Countdown();
  Force_MPU_Calibration();

  Show_Header("Init Motion...");
  Motion_Init(&motion_ctrl);
  delay_ms_blocking(500);

  /* Copy IR calibration into sensor_fusion for wall steering */
  sensor_fusion.ir_calib = ir_calib;

  /* Enable wall steering correction */
  motion_ctrl.wall_steer_enable = 1;

  UART_SendString("\r\n=== STRAIGHT + WALL STEER TEST ===\r\n");
  UART_SendString("Short press: Start/Stop | Long press: Exit\r\n");

  if (ir_calib.is_calibrated) {
    sprintf(buf, "WallPID: Kp=%.1f Ki=%.1f Kd=%.2f\r\n",
            motion_ctrl.pid_wall.kp, motion_ctrl.pid_wall.ki,
            motion_ctrl.pid_wall.kd);
    UART_SendString(buf);
    sprintf(buf, "  L0_sp=%d R0_sp=%d\r\n", ir_calib.center_l0,
            ir_calib.center_r0);
    UART_SendString(buf);
  } else {
    UART_SendString("WallSteer: OFF (not calibrated)\r\n");
  }

  /* Outer loop: repeated runs without re-calibrating */
  while (1) {
    /* === WAIT FOR START === */
    run_count++;
    sprintf(buf, "Run #%d", run_count);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("4. WALL STEER", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("Short:Start", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Long :Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- Ready for run #%d ---\r\n", run_count);
    UART_SendString(buf);

    /* Wait for button action */
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        /* Long press = exit straight test mode */
        UART_SendString("Exiting straight test mode\r\n");
        Show_Header("EXIT");
        delay_ms_blocking(1000);
        return;
      }
      if (btn == BTN_SHORT) {
        break; /* Start running */
      }
      delay_ms_blocking(10);
    }

    delay_ms_blocking(300);

    /* === RUNNING === */
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("RUNNING", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 50);
    SSD1306_WriteString("Press btn STOP", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    /* Reset encoders and yaw at this position */
    SensorFusion_ResetOdometry(&sensor_fusion);

    Motion_StartPID();
    /* Run for 3 cells to see wall steering effect over distance */
    Motion_Straight(&motion_ctrl, 3.0f * CELL_SIZE_MM);

    debug_timer = millis();

    sprintf(buf, "Run #%d started (3 cells, wall steer)\r\n", run_count);
    UART_SendString(buf);

    /* Inner loop: run straight until completed or button pressed */
    while (1) {
      /* 1. Check manual stop */
      if (Check_Button_Exit()) {
        Motion_Stop(&motion_ctrl);
        Motion_StopPID(); /* Stop the motor immediately! */
        UART_SendString("Manual Stop!\r\n");
        break;
      }

      /* 2. Check automatic completion */
      if (Motion_IsComplete(&motion_ctrl)) {
        Motion_StopPID();
        UART_SendString("Target reached (3 cells)!\r\n");
        break;
      }

      /* [CLEANUP] Motion_Update, Encoders, MPU updated by TIM11 @ 1kHz */

      if (millis() - debug_timer > 500) {
        sprintf(buf, "Yaw=%.1f Dist=%.0f WM=%d WE=%.2f WC=%.1f\r\n",
                sensor_fusion.heading, motion_ctrl.traveled_distance,
                sensor_fusion.wall_steer_mode,
                sensor_fusion.wall_steering_error, motion_ctrl.wall_correction);
        UART_SendString(buf);
        debug_timer = millis();
      }
    }

    /* === SHOW RESULT === */
    sprintf(buf, "Final: Yaw=%.2f Dist=%.0f WM=%d\r\n",
            MPU6050_GetYaw(&sensor_fusion.mpu_handle),
            motion_ctrl.traveled_distance, sensor_fusion.wall_steer_mode);
    UART_SendString(buf);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("STOPPED", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Yaw:%.1f", MPU6050_GetYaw(&sensor_fusion.mpu_handle));
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    sprintf(buf, "Dist:%.0fmm", motion_ctrl.traveled_distance);
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Short:Again Long:Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    delay_ms_blocking(500); /* Debounce */
  }
}

static void Test_Turn_90(void) {
  char buf[64];
  uint32_t last_update;
  uint32_t now;
  float dt;
  uint32_t debug_timer = 0;
  uint8_t run_count = 0;
  uint8_t btn;
  uint8_t press_count;
  uint32_t wait_start;
  float turn_angle;
  const char *turn_dir;

  Perform_Countdown();
  Force_MPU_Calibration();

  Show_Header("Init Motion...");
  Motion_Init(&motion_ctrl);
  delay_ms_blocking(500);

  UART_SendString("\r\n=== TURN 90 TEST ===\r\n");
  UART_SendString("1x press: Right | 2x press: Left | Long: Exit\r\n");

  /* Outer loop: repeated turns without re-calibrating */
  while (1) {
    /* === WAIT FOR INPUT === */
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("5. Turn90", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Turn #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("1x:Right 2x:Left", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Long:Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- Ready for turn #%d ---\r\n", run_count);
    UART_SendString(buf);

    /* Wait for button: detect 1 press vs 2 presses vs long */
    press_count = 0;
    turn_angle = 0;

    while (1) {
      btn = Check_Button_Action();

      if (btn == BTN_LONG) {
        UART_SendString("Exiting turn test mode\r\n");
        Show_Header("EXIT");
        delay_ms_blocking(1000);
        Motion_StopPID();
        return;
      }

      if (btn == BTN_SHORT) {
        press_count = 1;
        /* Wait 400ms to see if second press comes */
        wait_start = millis();
        while (millis() - wait_start < 400) {
          btn = Check_Button_Action();
          if (btn == BTN_SHORT) {
            press_count = 2;
            break;
          }
          delay_ms_blocking(10);
        }
        break;
      }
      delay_ms_blocking(10);
    }

    /* Determine direction */
    if (press_count == 1) {
      turn_angle = -90.0f; /* Right */
      turn_dir = "RIGHT";
    } else {
      turn_angle = 90.0f; /* Left */
      turn_dir = "LEFT";
    }

    delay_ms_blocking(300);

    /* === TURNING === */
    sprintf(buf, "Turning %s 90 deg...\r\n", turn_dir);
    UART_SendString(buf);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("TURNING", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "%s 90", turn_dir);
    SSD1306_SetCursor(0, 25);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    /* Reset before turn */
    MPU6050_ResetYaw(&sensor_fusion.mpu_handle);
    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);

    Motion_StartPID();
    Motion_Turn(&motion_ctrl, turn_angle);

    last_update = micros();
    debug_timer = millis();

    /* CSV header for PID tuning analysis */
    UART_SendString("T_ms,Yaw,Error,GyroZ,PID_out\r\n");

    /* Inner loop: execute turn until complete or button abort */
    while (!Motion_IsComplete(&motion_ctrl)) {
      if (Check_Button_Exit()) {
        Motion_Stop(&motion_ctrl);
        Motion_StopPID();
        UART_SendString("Aborted!\r\n");
        break;
      }

      /* [CLEANUP] TIM11 handles sensor and PID updates @ 1kHz */

      /* Fast logging every 50ms for PID tuning */
      if (millis() - debug_timer > 40) {
        sprintf(buf, "%lu,%.1f,%.1f,%.1f,%.1f\r\n", millis() - debug_timer,
                sensor_fusion.heading,
                motion_ctrl.target_angle - sensor_fusion.heading,
                sensor_fusion.gyro_z, motion_ctrl.pid_angle.output);
        UART_SendString(buf);
        debug_timer = millis();
      }
    }

    /* === SHOW RESULT === */
    Motion_StopPID();
    sprintf(buf, "%s turn done! Yaw=%.2f\r\n", turn_dir,
            MPU6050_GetYaw(&sensor_fusion.mpu_handle));
    UART_SendString(buf);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("DONE", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "%s 90", turn_dir);
    SSD1306_SetCursor(0, 14);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    sprintf(buf, "Yaw:%.1f", MPU6050_GetYaw(&sensor_fusion.mpu_handle));
    SSD1306_SetCursor(0, 38);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    delay_ms_blocking(500); /* Debounce */
  }
}

/* ================= TEST FRONT WALL ALIGNMENT ================= */

static void Test_FrontAlign(void) {
  char buf[80];
  uint32_t log_timer;
  uint8_t run_count = 0;
  uint8_t btn;

  UART_SendString("\r\n=== FRONT WALL ALIGN TEST ===\r\n");

  /* Check calibration */
  if (!ir_calib.is_calibrated || ir_calib.front_fsum_target == 0) {
    Show_Header("NO CALIB!");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("Run IR calib", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 35);
    SSD1306_WriteString("first!", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    UART_SendString("ERROR: IR not calibrated! front_fsum_target = 0\r\n");
    delay_ms_blocking(2000);
    return;
  }

  sprintf(buf, "Fsum_target: %lu  (L90=%d R90=%d)\r\n",
          (unsigned long)ir_calib.front_fsum_target, ir_calib.center_l90,
          ir_calib.center_r90);
  UART_SendString(buf);

  /* Outer loop: repeated alignment runs */
  while (1) {
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("6. FrontAlign", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 42);
    SSD1306_WriteString("Short:Start", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 54);
    SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- Align run #%d: Place robot facing wall ---\r\n",
            run_count);
    UART_SendString(buf);

    /* Wait for button */
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        UART_SendString("Exiting align test\r\n");
        return;
      }
      if (btn == BTN_SHORT)
        break;
      delay_ms_blocking(10);
    }

    delay_ms_blocking(300);

    /* === START ALIGNMENT via Motion API === */
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("ALIGNING...", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    /* TEST MODE: run until button press, with 30s safety timeout */
    motion_ctrl.align.timeout_ms = 30000;  /* Safety: auto-stop after 30s */
    motion_ctrl.align.stable_needed = 255; /* Never auto-stop by stability */

    Motion_StartPID();
    Motion_FrontAlign(&motion_ctrl, &ir_calib);

    log_timer = millis();

    UART_SendString("T_ms,Fsum,Fdiff,FdRef,eD,eY,pL,pR,Stab\r\n");

    /* Run alignment continuously until button press or safety timeout */
    while (1) {
      /* Short press = stop this run */
      btn = Check_Button_Action();
      if (btn == BTN_SHORT || btn == BTN_LONG) {
        Motion_Stop(&motion_ctrl);
        Hardware_StopMotors(); /* Double-ensure motors off */
        UART_SendString("Stopped by button\r\n");
        break;
      }

      /* If motion controller self-terminated (lost wall or safety timeout) */
      if (Motion_IsComplete(&motion_ctrl)) {
        Hardware_StopMotors(); /* Double-ensure motors off */
        UART_SendString("Controller stopped\r\n");
        break;
      }

      /* Log every 100ms - integers only, no floats in sprintf */
      if (millis() - log_timer > 100) {
        int i_fs =
            (int)sensor_fusion.ir_sensors[0] + (int)sensor_fusion.ir_sensors[5];
        int i_fd =
            (int)sensor_fusion.ir_sensors[0] - (int)sensor_fusion.ir_sensors[5];
        int i_ref = (int)motion_ctrl.align.fdiff_ref;
        int i_ed = (int)motion_ctrl.align.fsum_target - i_fs;
        int i_ey = i_fd - i_ref;
        int est_pl = (int)(motion_ctrl.align.kp_dist * i_ed -
                           motion_ctrl.align.kp_yaw * i_ey);
        int est_pr = (int)(motion_ctrl.align.kp_dist * i_ed +
                           motion_ctrl.align.kp_yaw * i_ey);

        /* Deadzone + offset estimate (match controller logic) */
        if (est_pl > 0 && est_pl < motion_ctrl.align.min_pwm)
          est_pl = motion_ctrl.align.min_pwm;
        if (est_pl < 0 && est_pl > -motion_ctrl.align.min_pwm)
          est_pl = -motion_ctrl.align.min_pwm;
        if (est_pr > 0 && est_pr < motion_ctrl.align.min_pwm)
          est_pr = motion_ctrl.align.min_pwm;
        if (est_pr < 0 && est_pr > -motion_ctrl.align.min_pwm)
          est_pr = -motion_ctrl.align.min_pwm;
        if (est_pr > 0)
          est_pr += motion_ctrl.align.motor_offset_R;
        if (est_pr < 0)
          est_pr -= motion_ctrl.align.motor_offset_R;
        if (i_ed > -(int)(motion_ctrl.align.tolerance_d * 0.25f) &&
            i_ed < (int)(motion_ctrl.align.tolerance_d * 0.25f) &&
            i_ey > -(int)(motion_ctrl.align.tolerance_h * 0.25f) &&
            i_ey < (int)(motion_ctrl.align.tolerance_h * 0.25f)) {
          est_pl = 0;
          est_pr = 0;
        }

        sprintf(buf, "%lu,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                millis() - motion_ctrl.align.start_time, i_fs, i_fd, i_ref,
                i_ed, i_ey, est_pl, est_pr, motion_ctrl.align.stable_count);
        UART_SendString(buf);

        /* NOTE: NO OLED update here!
         * OLED (SSD1306) and MPU6050 share I2C1.
         * SSD1306_UpdateScreen() blocks I2C1 for ~20ms.
         * TIM11 ISR calls MPU6050_Update() on I2C1 at 1kHz.
         * → I2C bus collision → Hard Fault → MCU freeze.
         * OLED is only updated AFTER alignment stops. */

        log_timer = millis();
      }
    }

    /* Restore defaults for production use */
    motion_ctrl.align.timeout_ms = 1500;
    motion_ctrl.align.stable_needed = 5;

    /* === SHOW RESULT === */
    {
      const char *result_str;
      switch (motion_ctrl.align.result) {
      case 1:
        result_str = "OK";
        break;
      case 2:
        result_str = "TIMEOUT";
        break;
      case 3:
        result_str = "LOST WALL";
        break;
      default:
        result_str = "STOPPED";
        break;
      }

      sprintf(buf, "Result: %s t=%lums\r\n", result_str,
              millis() - motion_ctrl.align.start_time);
      UART_SendString(buf);

      {
        int final_fsum =
            (int)sensor_fusion.ir_sensors[0] + (int)sensor_fusion.ir_sensors[5];
        int final_fdiff =
            (int)sensor_fusion.ir_sensors[0] - (int)sensor_fusion.ir_sensors[5];
        sprintf(buf, "Final: Fsum=%d(tgt=%d) Fdiff=%d\r\n", final_fsum,
                (int)motion_ctrl.align.fsum_target, final_fdiff);
        UART_SendString(buf);
      }

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      sprintf(buf, "ALIGN: %s", result_str);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      {
        int f_fsum =
            (int)sensor_fusion.ir_sensors[0] + (int)sensor_fusion.ir_sensors[5];
        int f_fdiff =
            (int)sensor_fusion.ir_sensors[0] - (int)sensor_fusion.ir_sensors[5];
        sprintf(buf, "Fs:%d/%d", f_fsum, (int)motion_ctrl.align.fsum_target);
        SSD1306_SetCursor(0, 18);
        SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
        sprintf(buf, "Fd:%d", f_fdiff);
        SSD1306_SetCursor(0, 40);
        SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      }
      SSD1306_SetCursor(0, 54);
      SSD1306_WriteString("Short:Again Long:Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();
    }

    delay_ms_blocking(500);
  }
}

/* ================= TEST SMOOTH TURN ================= */

/* ================= TEST CELL CENTER (multi-axis) ================= */

/**
 * @brief Test multi-axis cell centering.
 *
 * Sub-menu selects wall config: L-F / F-R / L-F-R
 * Robot drives forward until front wall detected,
 * then: front-align → snap → side-turn → side-align → snap → return → snap.
 */
static void Test_CellCenter(void) {
  typedef enum {
    CC_IDLE,
    CC_DRIVE_FWD,   /* Driving forward until front wall */
    CC_FRONT_ALIGN, /* Aligning with front wall */
    CC_SIDE_TURN,   /* Turning to face side wall */
    CC_SIDE_ALIGN,  /* Aligning with side wall */
    CC_SIDE_RETURN, /* Turning back to original dir */
    CC_DONE         /* Finished */
  } CC_State_t;

  char buf[80];
  uint8_t btn;
  uint8_t sub_mode = 0; /* 0=L-F, 1=F-R, 2=L-F-R */
  float side_angle = 0.0f;
  const char *mode_names[] = {"L-F", "F-R", "L-F-R"};
  CC_State_t cc_state;
  uint32_t log_timer;
  uint16_t ir_snap[6];
  uint8_t run_count = 0;

  UART_SendString("\r\n=== CELL CENTER TEST ===\r\n");

  /* Check calibration */
  if (!ir_calib.is_calibrated || ir_calib.front_fsum_target == 0) {
    Show_Header("NO CALIB!");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("Run IR calib", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 35);
    SSD1306_WriteString("first!", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    UART_SendString("ERROR: IR not calibrated!\r\n");
    delay_ms_blocking(2000);
    return;
  }

  /* === Sub-menu: select wall config === */
  while (1) {
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("Cell Center", &Font_7x10, SSD1306_WHITE);
    SSD1306_DrawLine(0, 12, 128, 12, SSD1306_WHITE);
    sprintf(buf, "< %s >", mode_names[sub_mode]);
    SSD1306_SetCursor(0, 25);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Short:Next Long:OK", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    btn = Check_Button_Action();
    if (btn == BTN_SHORT) {
      sub_mode++;
      if (sub_mode > 2)
        sub_mode = 0;
    } else if (btn == BTN_LONG) {
      break;
    }
    delay_ms_blocking(10);
  }

  /* Determine side turn angle based on mode */
  switch (sub_mode) {
  case 0:
    side_angle = 90.0f;
    break; /* L-F: turn left to face left wall */
  case 1:
    side_angle = -90.0f;
    break; /* F-R: turn right to face right wall */
  case 2:
    side_angle = -90.0f;
    break; /* L-F-R: turn right (fixed) */
  }

  sprintf(buf, "Mode: %s  side_angle=%.0f\r\n", mode_names[sub_mode],
          side_angle);
  UART_SendString(buf);

  /* === Main test loop: repeated runs === */
  while (1) {
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    sprintf(buf, "CC %s #%d", mode_names[sub_mode], run_count);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString("Short:Go", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 30);
    SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- CC run #%d [%s] ---\r\n", run_count,
            mode_names[sub_mode]);
    UART_SendString(buf);

    /* Wait for button */
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        UART_SendString("Exiting cell center test\r\n");
        return;
      }
      if (btn == BTN_SHORT)
        break;
      delay_ms_blocking(10);
    }

    delay_ms_blocking(500);

    /* Reset odometry & encoders */
    SensorFusion_ResetOdometry(&sensor_fusion);
    MPU6050_ResetYaw(&sensor_fusion.mpu_handle);
    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);

    Motion_StartPID();

    /* Start driving forward slowly (long distance, will stop manually) */
    Motion_StraightConstant(&motion_ctrl,
                            500.0f); /* 500mm max, will stop early */
    cc_state = CC_DRIVE_FWD;
    log_timer = millis();

    UART_SendString("Phase: DRIVE_FWD\r\n");

    /* === State machine loop === */
    while (cc_state != CC_DONE) {
      /* Emergency exit */
      btn = Check_Button_Action();
      if (btn == BTN_SHORT || btn == BTN_LONG) {
        Motion_Stop(&motion_ctrl);
        Hardware_StopMotors();
        UART_SendString("STOPPED by button\r\n");
        cc_state = CC_DONE;
        break;
      }

      switch (cc_state) {
      case CC_DRIVE_FWD:
        /* Check for front wall while driving */
        if (IR_Sensor_IsReady()) {
          IR_Sensor_GetResults(ir_snap);
          if (ir_calib.front_fsum_target > 0 &&
              (ir_snap[0] + ir_snap[5]) >
                  (uint16_t)(ir_calib.front_fsum_target * 0.5f)) {
            /* Front wall detected! Start front align */
            sprintf(buf, "FWall detected: L90=%d R90=%d sum=%d\r\n", ir_snap[0],
                    ir_snap[5], ir_snap[0] + ir_snap[5]);
            UART_SendString(buf);

            Motion_FrontAlign(&motion_ctrl, &ir_calib);
            UART_SendString("Phase: FRONT_ALIGN\r\n");
            log_timer = millis();
            cc_state = CC_FRONT_ALIGN;
          }
        }

        /* Log drive progress every 200ms */
        if (millis() - log_timer > 200) {
          int enc_l = (int)Hardware_GetEncoderCount(1);
          int enc_r = (int)Hardware_GetEncoderCount(0);
          int i_l90 = (int)sensor_fusion.ir_sensors[0];
          int i_r90 = (int)sensor_fusion.ir_sensors[5];
          sprintf(buf, "DRV: eL=%d eR=%d L90=%d R90=%d\r\n", enc_l, enc_r,
                  i_l90, i_r90);
          UART_SendString(buf);
          log_timer = millis();
        }

        /* Safety: max distance reached */
        if (Motion_IsComplete(&motion_ctrl)) {
          UART_SendString("WARN: Max distance, no front wall!\r\n");
          Motion_Stop(&motion_ctrl);
          cc_state = CC_DONE;
        }
        break;

      case CC_FRONT_ALIGN:
        if (Motion_IsComplete(&motion_ctrl)) {
          sprintf(buf, "FRONT: result=%d\r\n", motion_ctrl.align.result);
          UART_SendString(buf);

          if (motion_ctrl.align.result == 1) {
            Motion_SnapHeading();
            UART_SendString("Phase: SIDE_TURN\r\n");
            Motion_Turn(&motion_ctrl, side_angle);
            cc_state = CC_SIDE_TURN;
          } else {
            /* Front align failed, abort */
            UART_SendString("FRONT align failed, aborting\r\n");
            cc_state = CC_DONE;
          }
        }

        /* Log front-align every 100ms */
        if (millis() - log_timer > 100) {
          int i_fs = (int)sensor_fusion.ir_sensors[0] +
                     (int)sensor_fusion.ir_sensors[5];
          int i_fd = (int)sensor_fusion.ir_sensors[0] -
                     (int)sensor_fusion.ir_sensors[5];
          int i_ed = (int)motion_ctrl.align.fsum_target - i_fs;
          int i_ey = i_fd - (int)motion_ctrl.align.fdiff_ref;
          sprintf(buf, "FA: Fs=%d eD=%d eY=%d stab=%d\r\n", i_fs, i_ed, i_ey,
                  motion_ctrl.align.stable_count);
          UART_SendString(buf);
          log_timer = millis();
        }
        break;

      case CC_SIDE_TURN:
        if (Motion_IsComplete(&motion_ctrl)) {
          Motion_SnapHeading();

          /* Check if side wall (now in front) is visible */
          if (IR_Sensor_IsReady()) {
            IR_Sensor_GetResults(ir_snap);
          }
          if (ir_calib.front_fsum_target > 0 &&
              (ir_snap[0] + ir_snap[5]) >
                  (uint16_t)(ir_calib.front_fsum_target * 0.3f)) {
            sprintf(buf, "Side wall OK: L90=%d R90=%d\r\n", ir_snap[0],
                    ir_snap[5]);
            UART_SendString(buf);
            Motion_FrontAlign(&motion_ctrl, &ir_calib);
            UART_SendString("Phase: SIDE_ALIGN\r\n");
            log_timer = millis();
            cc_state = CC_SIDE_ALIGN;
          } else {
            UART_SendString("Side wall NOT visible, skip\r\n");
            Motion_Turn(&motion_ctrl, -side_angle);
            cc_state = CC_SIDE_RETURN;
          }
        }
        break;

      case CC_SIDE_ALIGN:
        if (Motion_IsComplete(&motion_ctrl)) {
          sprintf(buf, "SIDE: result=%d\r\n", motion_ctrl.align.result);
          UART_SendString(buf);

          if (motion_ctrl.align.result == 1) {
            Motion_SnapHeading();
          }

          /* Turn back to original direction */
          UART_SendString("Phase: SIDE_RETURN\r\n");
          Motion_Turn(&motion_ctrl, -side_angle);
          cc_state = CC_SIDE_RETURN;
        }

        /* Log side-align every 100ms */
        if (millis() - log_timer > 100) {
          int i_fs = (int)sensor_fusion.ir_sensors[0] +
                     (int)sensor_fusion.ir_sensors[5];
          int i_fd = (int)sensor_fusion.ir_sensors[0] -
                     (int)sensor_fusion.ir_sensors[5];
          int i_ed = (int)motion_ctrl.align.fsum_target - i_fs;
          int i_ey = i_fd - (int)motion_ctrl.align.fdiff_ref;
          sprintf(buf, "SA: Fs=%d eD=%d eY=%d stab=%d\r\n", i_fs, i_ed, i_ey,
                  motion_ctrl.align.stable_count);
          UART_SendString(buf);
          log_timer = millis();
        }
        break;

      case CC_SIDE_RETURN:
        if (Motion_IsComplete(&motion_ctrl)) {
          Motion_SnapHeading();
          UART_SendString("Phase: DONE\r\n");
          cc_state = CC_DONE;
        }
        break;

      case CC_DONE:
      case CC_IDLE:
        break;
      }
    }

    /* Stop everything */
    Motion_Stop(&motion_ctrl);
    Motion_StopPID();
    Hardware_StopMotors();

    /* Show result on OLED */
    {
      int f_fsum =
          (int)sensor_fusion.ir_sensors[0] + (int)sensor_fusion.ir_sensors[5];
      float yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      sprintf(buf, "CC %s DONE", mode_names[sub_mode]);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      sprintf(buf, "Fs:%d Yaw:%.1f", f_fsum, yaw);
      SSD1306_SetCursor(0, 18);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(0, 42);
      SSD1306_WriteString("Short:Again", &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(0, 54);
      SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();

      sprintf(buf, "Final: Fsum=%d Yaw=%.1f\r\n", f_fsum, yaw);
      UART_SendString(buf);
    }

    delay_ms_blocking(500);
  }
}

static void Test_Smooth_Turn(void) {
  char buf[80];
  uint32_t phase_start;
  int run_count = 0;
  float turn_angle;
  const char *turn_dir;

  UART_SendString("\r\n=== PIVOT TURN TEST (3-cell) ===\r\n");
  UART_SendString("1x press: Right | 2x press: Left | Long: Exit\r\n");

  /* Countdown + calibrate */
  Perform_Countdown();
  Force_MPU_Calibration();

  Show_Header("Init Motion...");
  Motion_Init(&motion_ctrl);
  delay_ms_blocking(500);

  /* Outer loop: multiple runs */
  while (1) {
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("7. Pivot", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 42);
    SSD1306_WriteString("1x:R 2x:L", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 54);
    SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    UART_SendString("\r\n--- Ready for pivot turn ---\r\n");

    /* Wait for button input */
    {
      uint8_t press_count = 0;
      uint8_t btn;
      uint32_t wait_start;

      while (1) {
        btn = Check_Button_Action();

        if (btn == BTN_LONG) {
          UART_SendString("Exiting pivot turn test\r\n");
          Motion_StopPID();
          return;
        }

        if (btn == BTN_SHORT) {
          press_count = 1;
          wait_start = millis();
          while (millis() - wait_start < 400) {
            btn = Check_Button_Action();
            if (btn == BTN_SHORT) {
              press_count = 2;
              break;
            }
            delay_ms_blocking(10);
          }
          break;
        }
        delay_ms_blocking(10);
      }

      if (press_count == 1) {
        turn_angle = -90.0f;
        turn_dir = "RIGHT";
      } else {
        turn_angle = 90.0f;
        turn_dir = "LEFT";
      }
    }

    delay_ms_blocking(300);

    sprintf(buf, "Pivot %s: Straight->Pivot->Straight\r\n", turn_dir);
    UART_SendString(buf);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    sprintf(buf, "Pivot %s", turn_dir);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 14);
    SSD1306_WriteString("Phase 1/3", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    /* Reset before test */
    MPU6050_ResetYaw(&sensor_fusion.mpu_handle);
    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);

    Motion_StartPID();

    /* ========== PHASE 1: Straight 1 cell ========== */
    UART_SendString("Phase1: Straight 1 cell\r\n");
    Motion_Straight(&motion_ctrl, CELL_SIZE_MM);
    phase_start = millis();

    while (!Motion_IsComplete(&motion_ctrl)) {
      if (Check_Button_Exit()) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Aborted!\r\n");
        goto test_done;
      }
      if (millis() - phase_start > 5000) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Phase1 TIMEOUT!\r\n");
        goto test_done;
      }
    }

    sprintf(buf, "Phase1 done: yaw=%.1f dist=%.0f t=%lums\r\n",
            MPU6050_GetYaw(&sensor_fusion.mpu_handle),
            motion_ctrl.traveled_distance, millis() - phase_start);
    UART_SendString(buf);

    delay_ms_blocking(100); /* Brief settle */

    /* ========== PHASE 2: Pivot turn 90° ========== */
    sprintf(buf, "Phase2: Pivot %s 90 deg\r\n", turn_dir);
    UART_SendString(buf);

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    sprintf(buf, "Pivot %s", turn_dir);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 14);
    SSD1306_WriteString("Phase 2/3", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    Motion_SmoothTurn(&motion_ctrl, turn_angle);
    phase_start = millis();

    UART_SendString("T_ms,Yaw,Error,GyroZ,PWM\r\n");

    while (!Motion_IsComplete(&motion_ctrl)) {
      if (Check_Button_Exit()) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Aborted!\r\n");
        goto test_done;
      }
      if (millis() - phase_start > 5000) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Phase2 TIMEOUT!\r\n");
        goto test_done;
      }

      /* Log every 50ms */
      {
        static uint32_t log_t = 0;
        if (millis() - log_t > 50) {
          float yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
          float gz = sensor_fusion.gyro_z;
          sprintf(buf, "%lu,%.1f,%.1f,%.1f,%d\r\n", millis() - phase_start, yaw,
                  motion_ctrl.pid_angle.prev_error, gz,
                  (int)motion_ctrl.pivot_turn_speed);
          UART_SendString(buf);
          log_t = millis();
        }
      }
    }

    sprintf(buf, "Phase2 done: yaw=%.1f t=%lums\r\n",
            MPU6050_GetYaw(&sensor_fusion.mpu_handle), millis() - phase_start);
    UART_SendString(buf);

    delay_ms_blocking(100); /* Brief settle */

    /* ========== PHASE 3: Straight 1 cell (with decel to stop) ========== */
    UART_SendString("Phase3: Straight 1 cell (stop)\r\n");

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    sprintf(buf, "Pivot %s", turn_dir);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 14);
    SSD1306_WriteString("Phase 3/3", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    Motion_Straight(&motion_ctrl, CELL_SIZE_MM);
    phase_start = millis();

    while (!Motion_IsComplete(&motion_ctrl)) {
      if (Check_Button_Exit()) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Aborted!\r\n");
        goto test_done;
      }
      if (millis() - phase_start > 5000) {
        Motion_Stop(&motion_ctrl);
        UART_SendString("Phase3 TIMEOUT!\r\n");
        goto test_done;
      }
    }

    sprintf(buf, "Phase3 done: yaw=%.1f dist=%.0f t=%lums\r\n",
            MPU6050_GetYaw(&sensor_fusion.mpu_handle),
            motion_ctrl.traveled_distance, millis() - phase_start);
    UART_SendString(buf);

  test_done:
    /* Show result */
    Motion_StopPID();
    Hardware_StopMotors();

    {
      float final_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
      sprintf(buf, "RESULT: %s pivot Yaw=%.1f\r\n", turn_dir, final_yaw);
      UART_SendString(buf);

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("DONE", &Font_7x10, SSD1306_WHITE);
      sprintf(buf, "%s 90", turn_dir);
      SSD1306_SetCursor(0, 14);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
      sprintf(buf, "Yaw:%.1f", final_yaw);
      SSD1306_SetCursor(0, 40);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
      SSD1306_UpdateScreen();
    }

    delay_ms_blocking(500);
  }
}

/* ================= TRANSITION STRAIGHT TEST (2-threshold hysteresis)
 * ================= */
/* Drive 4 cells straight, detect wall/post appearance and disappearance, reset
 * encoder at each transition to minimize accumulated error. Tests calibration
 * capability. */

#define TR_NUM_CELLS 3            /* Number of cells to drive */
#define TR_DETECT_THRESH 80       /* L45/R45 ADC > this = seeing wall/post */
#define TR_FADEOFF_THRESH 20      /* L45/R45 ADC < this = transition done  */
#define TR_WALL_LEVEL 350         /* peak > this = wall (vs post)          */
#define TR_TRAILING_POS_MM 137.0f /* True pos when trailing edge triggers */
#define TR_LEADING_POS_MM 163.0f  /* True pos when leading edge triggers  */
#define TR_COOLDOWN_MM 50.0f      /* Min distance between events           */
#define TR_MIN_DIST_MM 40.0f      /* Ignore events before this distance    */

static void Test_TransitionStraight(void) {
  char buf[96];
  uint8_t run_count = 0;
  uint8_t btn;

  UART_SendString("\r\n=== TRANSITION STRAIGHT TEST (2-thresh) ===\r\n");
  sprintf(buf, "Cells=%d Detect=%d Fade=%d WallLvl=%d\r\n", TR_NUM_CELLS,
          TR_DETECT_THRESH, TR_FADEOFF_THRESH, TR_WALL_LEVEL);
  UART_SendString(buf);

  Perform_Countdown();
  Force_MPU_Calibration();
  Motion_Init(&motion_ctrl);
  sensor_fusion.ir_calib = ir_calib;
  motion_ctrl.wall_steer_enable = 1;

  while (1) {
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("9. TransStr", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 42);
    SSD1306_WriteString("Short:Start", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 54);
    SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- Trans straight run #%d (%d cells) ---\r\n", run_count,
            TR_NUM_CELLS);
    UART_SendString(buf);

    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        UART_SendString("Exiting transition straight test\r\n");
        LED_OFF();
        return;
      }
      if (btn == BTN_SHORT)
        break;
      delay_ms_blocking(10);
    }
    delay_ms_blocking(300);

    /* === RUN: drive straight with transition detection === */
    {
      /* Init seen from current sensor values to avoid false triggers */
      uint8_t seen_l45 =
          (sensor_fusion.ir_sensors[1] > TR_DETECT_THRESH) ? 1 : 0;
      uint8_t seen_r45 =
          (sensor_fusion.ir_sensors[4] > TR_DETECT_THRESH) ? 1 : 0;
      uint16_t max_l45 = 0, max_r45 = 0;
      uint8_t corrected = 0;
      float cooldown_dist = -100.0f;
      uint8_t cell_count = 0;
      uint8_t event_count = 0;
      uint32_t log_timer, led_off_time = 0;

      SensorFusion_ResetOdometry(&sensor_fusion);
      Motion_StartPID();
      Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);
      log_timer = millis();

      UART_SendString("dist,L45,R45,event\r\n");

      while (cell_count < TR_NUM_CELLS) {
        SSD1306_Process_DMA();

        if (Check_Button_Exit()) {
          Motion_Stop(&motion_ctrl);
          Motion_StopPID();
          LED_OFF();
          UART_SendString("Aborted!\r\n");
          goto str_done;
        }

        /* Auto LED off */
        if (led_off_time && millis() >= led_off_time) {
          LED_OFF();
          led_off_time = 0;
        }

        {
          uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
          uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
          float dist = motion_ctrl.traveled_distance;
          uint8_t disappear = 0, appear = 0;
          const char *evt_name = "";

          /* L45: appearance */
          if (l45 > TR_DETECT_THRESH && !seen_l45)
            appear = 1;
          if (l45 > TR_DETECT_THRESH) {
            seen_l45 = 1;
            if (l45 > max_l45)
              max_l45 = l45;
          }
          if (l45 < TR_FADEOFF_THRESH && seen_l45) {
            seen_l45 = 0;
            disappear = 1;
          }

          /* R45: appearance */
          if (r45 > TR_DETECT_THRESH && !seen_r45)
            appear = 1;
          if (r45 > TR_DETECT_THRESH) {
            seen_r45 = 1;
            if (r45 > max_r45)
              max_r45 = r45;
          }
          if (r45 < TR_FADEOFF_THRESH && seen_r45) {
            seen_r45 = 0;
            disappear = 1;
          }

          /* Disappearance (trailing edge): RESET encoder, drive remaining */
          if (disappear && !corrected && dist > TR_MIN_DIST_MM &&
              (dist - cooldown_dist > TR_COOLDOWN_MM)) {
            uint16_t peak = (max_l45 > max_r45) ? max_l45 : max_r45;
            float remaining = CELL_SIZE_MM - TR_TRAILING_POS_MM;
            evt_name = (peak > TR_WALL_LEVEL) ? "WALL_DN" : "POST";

            /* Reset encoder to eliminate accumulated drift */
            Hardware_ResetEncoder(0);
            Hardware_ResetEncoder(1);
            motion_ctrl.traveled_distance = 0.0f;
            motion_ctrl.target_distance = remaining;
            motion_ctrl.start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

            cooldown_dist = 0.0f;
            event_count++;
            corrected = 1;

            sprintf(buf, "*** %s #%d: d=%.0f rem=%.0f pk=%d\r\n", evt_name,
                    event_count, dist, remaining, peak);
            UART_SendString(buf);
            LED_ON();
            led_off_time = millis() + 80;
            max_l45 = 0;
            max_r45 = 0;
          }
          /* Appearance (leading edge): RESET encoder, drive remaining */
          else if (appear && !disappear && !corrected &&
                   (dist - cooldown_dist > TR_COOLDOWN_MM) &&
                   dist > TR_MIN_DIST_MM) {
            float remaining = CELL_SIZE_MM - TR_LEADING_POS_MM;
            evt_name = "WALL_UP";

            /* Reset encoder to eliminate accumulated drift */
            Hardware_ResetEncoder(0);
            Hardware_ResetEncoder(1);
            motion_ctrl.traveled_distance = 0.0f;
            motion_ctrl.target_distance = remaining;
            motion_ctrl.start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

            cooldown_dist = 0.0f;
            event_count++;
            corrected = 1;

            sprintf(buf, "*** %s #%d: d=%.0f rem=%.0f\r\n", evt_name,
                    event_count, dist, remaining);
            UART_SendString(buf);
            LED_ON();
            led_off_time = millis() + 80;
          }

          /* Regular log every 50ms */
          if (millis() - log_timer > 50) {
            sprintf(buf, "%.0f,%d,%d,%s\r\n", dist, l45, r45, evt_name);
            UART_SendString(buf);
            log_timer = millis();
            evt_name = "";
          }

          /* Cell crossing: chain next cell */
          if (motion_ctrl.traveled_distance >=
              (motion_ctrl.target_distance - 1.0f)) {
            cell_count++;
            if (cell_count < TR_NUM_CELLS) {
              /* Reset transition state for next cell.
               * Init seen from current sensor readings. */
              corrected = 0;
              seen_l45 =
                  (sensor_fusion.ir_sensors[1] > TR_DETECT_THRESH) ? 1 : 0;
              seen_r45 =
                  (sensor_fusion.ir_sensors[4] > TR_DETECT_THRESH) ? 1 : 0;
              max_l45 = 0;
              max_r45 = 0;
              Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);
              sprintf(buf, "CELL %d/%d\r\n", cell_count + 1, TR_NUM_CELLS);
              UART_SendString(buf);
            }
          }
        }
      }

      /* Stop at end */
      Motion_Stop(&motion_ctrl);
      while (!Motion_IsComplete(&motion_ctrl)) {
        SSD1306_Process_DMA();
      }
      Motion_StopPID();
      LED_OFF();

      sprintf(buf, "\r\nRESULT: %d events over %d cells\r\n", event_count,
              TR_NUM_CELLS);
      UART_SendString(buf);
      sprintf(buf, "Final dist: %.1fmm (target %.0fmm)\r\n",
              motion_ctrl.traveled_distance,
              (float)TR_NUM_CELLS * CELL_SIZE_MM);
      UART_SendString(buf);

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("TRANS STR", &Font_7x10, SSD1306_WHITE);
      sprintf(buf, "%d evt", event_count);
      SSD1306_SetCursor(0, 18);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
      sprintf(buf, "D:%.0fmm", motion_ctrl.traveled_distance);
      SSD1306_SetCursor(0, 42);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(0, 54);
      SSD1306_WriteString("Short:Again Long:Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();
    }
  str_done:
    delay_ms_blocking(500);
  }
}

/* ================= TRANSITION TURN TEST ================= */
/* Drive straight until transition event detected, then pivot turn 90°.
 * Repeats for 4 cells to test transition-triggered turning accuracy. */

static void Test_TransitionTurn(void) {
  char buf[96];
  uint8_t run_count = 0;
  uint8_t btn;
  float turn_angle;
  const char *turn_dir;

  UART_SendString("\r\n=== TRANSITION TURN TEST ===\r\n");

  Perform_Countdown();
  Force_MPU_Calibration();
  Motion_Init(&motion_ctrl);
  sensor_fusion.ir_calib = ir_calib;
  motion_ctrl.wall_steer_enable = 1;

  while (1) {
    uint8_t press_count;
    uint32_t wait_start;
    run_count++;

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("10.TransTurn", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 42);
    SSD1306_WriteString("1x:R 2x:L", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 54);
    SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- Trans turn run #%d ---\r\n", run_count);
    UART_SendString(buf);

    /* Wait for button: 1x = right, 2x = left, long = exit */
    press_count = 0;
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        UART_SendString("Exiting transition turn test\r\n");
        return;
      }
      if (btn == BTN_SHORT) {
        press_count = 1;
        wait_start = millis();
        while (millis() - wait_start < 400) {
          btn = Check_Button_Action();
          if (btn == BTN_SHORT) {
            press_count = 2;
            break;
          }
          delay_ms_blocking(10);
        }
        break;
      }
      delay_ms_blocking(10);
    }

    turn_angle = (press_count == 1) ? -90.0f : 90.0f;
    turn_dir = (press_count == 1) ? "R" : "L";
    delay_ms_blocking(300);

    /* === RUN: straight(1 cell with transition) → stop → turn → straight(1
     * cell) === */
    {
      uint8_t seen_l45 =
          (sensor_fusion.ir_sensors[1] > TR_DETECT_THRESH) ? 1 : 0;
      uint8_t seen_r45 =
          (sensor_fusion.ir_sensors[4] > TR_DETECT_THRESH) ? 1 : 0;
      uint16_t max_l45 = 0, max_r45 = 0;
      uint8_t corrected = 0;
      float cooldown_dist = -100.0f;
      uint32_t phase_start, led_off_time = 0;

      SensorFusion_ResetOdometry(&sensor_fusion);
      Motion_StartPID();

      /* ========== PHASE 1: Straight with transition detection ========== */
      sprintf(buf, "Phase1: Straight + transition detect\r\n");
      UART_SendString(buf);

      Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);
      phase_start = millis();

      while (!Motion_IsComplete(&motion_ctrl)) {
        SSD1306_Process_DMA();
        if (Check_Button_Exit()) {
          Motion_Stop(&motion_ctrl);
          Motion_StopPID();
          UART_SendString("Aborted!\r\n");
          goto turn_done;
        }
        if (millis() - phase_start > 10000) {
          Motion_Stop(&motion_ctrl);
          UART_SendString("Phase1 TIMEOUT!\r\n");
          goto turn_done;
        }
        if (led_off_time && millis() >= led_off_time) {
          LED_OFF();
          led_off_time = 0;
        }

        /* Transition detection (same logic) */
        if (!corrected) {
          uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
          uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
          float dist = motion_ctrl.traveled_distance;
          uint8_t disappear = 0, appear = 0;

          if (l45 > TR_DETECT_THRESH && !seen_l45)
            appear = 1;
          if (l45 > TR_DETECT_THRESH) {
            seen_l45 = 1;
            if (l45 > max_l45)
              max_l45 = l45;
          }
          if (l45 < TR_FADEOFF_THRESH && seen_l45) {
            seen_l45 = 0;
            disappear = 1;
          }

          if (r45 > TR_DETECT_THRESH && !seen_r45)
            appear = 1;
          if (r45 > TR_DETECT_THRESH) {
            seen_r45 = 1;
            if (r45 > max_r45)
              max_r45 = r45;
          }
          if (r45 < TR_FADEOFF_THRESH && seen_r45) {
            seen_r45 = 0;
            disappear = 1;
          }

          if (disappear && dist > TR_MIN_DIST_MM &&
              (dist - cooldown_dist > TR_COOLDOWN_MM)) {
            uint16_t peak = (max_l45 > max_r45) ? max_l45 : max_r45;
            float remaining = CELL_SIZE_MM - TR_TRAILING_POS_MM;

            Hardware_ResetEncoder(0);
            Hardware_ResetEncoder(1);
            motion_ctrl.traveled_distance = 0.0f;
            motion_ctrl.target_distance = remaining;
            motion_ctrl.start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
            corrected = 1;
            cooldown_dist = 0.0f;
            sprintf(buf, "TRANS:%s d=%.0f rem=%.0f pk=%d\r\n",
                    (peak > TR_WALL_LEVEL) ? "W" : "P", dist, remaining, peak);
            UART_SendString(buf);
            LED_ON();
            led_off_time = millis() + 80;
          } else if (appear && !disappear &&
                     (dist - cooldown_dist > TR_COOLDOWN_MM) &&
                     dist > TR_MIN_DIST_MM) {
            float remaining = CELL_SIZE_MM - TR_LEADING_POS_MM;

            Hardware_ResetEncoder(0);
            Hardware_ResetEncoder(1);
            motion_ctrl.traveled_distance = 0.0f;
            motion_ctrl.target_distance = remaining;
            motion_ctrl.start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
            corrected = 1;
            cooldown_dist = 0.0f;
            sprintf(buf, "TRANS:A d=%.0f rem=%.0f\r\n", dist, remaining);
            UART_SendString(buf);
            LED_ON();
            led_off_time = millis() + 80;
          }
        }
      }

      sprintf(buf, "Phase1 done: dist=%.0f yaw=%.1f\r\n",
              motion_ctrl.traveled_distance,
              MPU6050_GetYaw(&sensor_fusion.mpu_handle));
      UART_SendString(buf);

      delay_ms_blocking(100);

      /* ========== PHASE 2: Pivot turn ========== */
      sprintf(buf, "Phase2: Turn %s 90\r\n", turn_dir);
      UART_SendString(buf);

      Motion_SmoothTurn(&motion_ctrl, turn_angle);
      phase_start = millis();

      while (!Motion_IsComplete(&motion_ctrl)) {
        SSD1306_Process_DMA();
        if (Check_Button_Exit()) {
          Motion_Stop(&motion_ctrl);
          UART_SendString("Aborted!\r\n");
          goto turn_done;
        }
        if (millis() - phase_start > 5000) {
          Motion_Stop(&motion_ctrl);
          UART_SendString("Phase2 TIMEOUT!\r\n");
          goto turn_done;
        }
      }

      sprintf(buf, "Phase2 done: yaw=%.1f\r\n",
              MPU6050_GetYaw(&sensor_fusion.mpu_handle));
      UART_SendString(buf);

      delay_ms_blocking(100);

      /* ========== PHASE 3: Straight 1 cell (stop at center) ========== */
      UART_SendString("Phase3: Straight 1 cell\r\n");

      /* Reset transition state for new direction */
      seen_l45 = 0;
      seen_r45 = 0;
      max_l45 = 0;
      max_r45 = 0;
      corrected = 0;
      cooldown_dist = -100.0f;

      Motion_Straight(&motion_ctrl, CELL_SIZE_MM);
      phase_start = millis();

      while (!Motion_IsComplete(&motion_ctrl)) {
        SSD1306_Process_DMA();
        if (Check_Button_Exit()) {
          Motion_Stop(&motion_ctrl);
          UART_SendString("Aborted!\r\n");
          goto turn_done;
        }
        if (millis() - phase_start > 5000) {
          Motion_Stop(&motion_ctrl);
          UART_SendString("Phase3 TIMEOUT!\r\n");
          goto turn_done;
        }
      }

      sprintf(buf, "Phase3 done: dist=%.0f yaw=%.1f\r\n",
              motion_ctrl.traveled_distance,
              MPU6050_GetYaw(&sensor_fusion.mpu_handle));
      UART_SendString(buf);
    }

  turn_done:
    Motion_StopPID();
    Hardware_StopMotors();

    {
      float final_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
      sprintf(buf, "RESULT: %s turn Yaw=%.1f\r\n", turn_dir, final_yaw);
      UART_SendString(buf);

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("TRANS TURN", &Font_7x10, SSD1306_WHITE);
      sprintf(buf, "%s Yaw:%.1f", turn_dir, final_yaw);
      SSD1306_SetCursor(0, 18);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
      SSD1306_SetCursor(0, 42);
      SSD1306_WriteString("Short:Again", &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(0, 54);
      SSD1306_WriteString("Long: Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();
    }

    delay_ms_blocking(500);
  }
}

/* ================= TEST SPHT (Sensors Presence Hysteresis Test) ======= */

/**
 * @brief SPHT Mode: Drive straight and detect wall appear/disappear events
 *        using L0/R0 sensors with hysteresis thresholds.
 *        Logs all events to UART (OLED mirror since hardware OLED is broken).
 */
static void Test_SPHT(void) {
  char buf[100];
  uint8_t run_count = 0;
  uint8_t btn;
  uint32_t log_timer;

  /* Thresholds from calibration */
  uint16_t l0_th, r0_th;
  uint16_t l0_off_th, r0_off_th; /* 40% of threshold = disappear level */
  uint16_t front_th;             /* 50% of front_fsum_target */

  Perform_Countdown();
  Force_MPU_Calibration();

  Show_Header("Init Motion...");
  UART_SendString("\r\n[SPHT] Init Motion...\r\n");
  Motion_Init(&motion_ctrl);
  delay_ms_blocking(500);

  /* Copy IR calibration into sensor_fusion for wall steering */
  sensor_fusion.ir_calib = ir_calib;

  UART_SendString("\r\n=== SPHT: Sensors Presence Hysteresis Test ===\r\n");
  UART_SendString("Drive straight, detect wall appear/disappear events\r\n");

  /* Setup thresholds from calibration */
  if (ir_calib.is_calibrated) {
    /* Fallback to old coefficient thresholds */
    l0_th = ir_calib.wall_threshold_front_left;
    r0_th = ir_calib.wall_threshold_front_right;
    sprintf(buf, "[SPHT] Using coeff thresholds: L0=%d R0=%d\r\n", l0_th,
            r0_th);
  } else {
    l0_th = 150;
    r0_th = 150;
    sprintf(buf, "[SPHT] WARNING: Not calibrated! Using default: %d\r\n", 150);
  }
  UART_SendString(buf);

  /* Disappear threshold = 40% of appear threshold */
  l0_off_th = (uint16_t)(l0_th * 0.40f);
  r0_off_th = (uint16_t)(r0_th * 0.40f);
  sprintf(buf, "[SPHT] Off-thresholds: L0=%d R0=%d\r\n", l0_off_th, r0_off_th);
  UART_SendString(buf);

  /* Front wall threshold = 50% of front_fsum_target */
  if (ir_calib.is_calibrated && ir_calib.front_fsum_target > 0) {
    front_th = (uint16_t)(ir_calib.front_fsum_target * 0.50f);
  } else {
    front_th = 400; /* Safe default */
  }
  sprintf(buf, "[SPHT] Front threshold (Fsum): %d (target=%lu)\r\n", front_th,
          (unsigned long)ir_calib.front_fsum_target);
  UART_SendString(buf);

  /* ===== Outer loop: repeated runs ===== */
  while (1) {
    run_count++;

    /* --- Display menu (OLED + UART) --- */
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("11. SPHT", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("Short:Start", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Long :Exit", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    /* UART mirror of OLED */
    sprintf(buf, "\r\n[SPHT] === Run #%d === Short:Start | Long:Exit\r\n",
            run_count);
    UART_SendString(buf);

    /* Wait for button */
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG) {
        UART_SendString("[SPHT] Button LONG pressed\r\n");
        UART_SendString("[SPHT] OLED Displaying: EXIT\r\n");
        UART_SendString("[SPHT] Exiting SPHT mode\r\n");
        Show_Header("EXIT");
        delay_ms_blocking(1000);
        return;
      }
      if (btn == BTN_SHORT) {
        UART_SendString("[SPHT] Button SHORT pressed\r\n");
        break;
      }
      delay_ms_blocking(10);
    }

    delay_ms_blocking(300);

    /* --- Setup for this run --- */
    SensorFusion_ResetOdometry(&sensor_fusion);
    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);

    UART_SendString("[SPHT] Starting run...\r\n");

    /* Display running state (OLED + UART) */
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("SPHT RUNNING", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "Run #%d", run_count);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 50);
    SSD1306_WriteString("Press=STOP", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    UART_SendString("[SPHT] OLED Displaying: SPHT RUNNING | Press=STOP\r\n");
    UART_SendString("[SPHT] RUNNING | Press button to stop\r\n");

    Motion_StartPID();
    Motion_StraightConstant(&motion_ctrl, 1000.0f); /* Max 1000mm */

    log_timer = millis();

    /* --- Detection loop state --- */
    {
      uint8_t seen_l0 = 0, seen_r0 = 0;
      uint8_t front_detected = 0;

      /* CSV header for logging */
      UART_SendString("[SPHT] T_ms,Dist,L0,R0,L90,R90,L45,R45,sL,sR\r\n");

      while (1) {
        /* Get sensor values from sensor fusion (filtered by TIM11 ISR) */
        uint16_t l0 = (uint16_t)sensor_fusion.ir_sensors[2];
        uint16_t r0 = (uint16_t)sensor_fusion.ir_sensors[3];
        uint16_t l90 = (uint16_t)sensor_fusion.ir_sensors[0];
        uint16_t r90 = (uint16_t)sensor_fusion.ir_sensors[5];
        uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
        uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
        uint16_t fsum = l90 + r90;
        float dist = motion_ctrl.traveled_distance;

        /* === LEFT WALL detection === */
        if (l0 > l0_th && !seen_l0) {
          seen_l0 = 1;
          LED_ON();
          delay_ms_blocking(30);
          LED_OFF();
          sprintf(buf, "[SPHT] LEFT APPEARED  @ %.0fmm L0=%d\r\n", dist, l0);
          UART_SendString(buf);
        } else if (l0 < l0_off_th && seen_l0) {
          seen_l0 = 0;
          LED_ON();
          delay_ms_blocking(30);
          LED_OFF();
          sprintf(buf, "[SPHT] LEFT DISAPPEARED @ %.0fmm L0=%d\r\n", dist, l0);
          UART_SendString(buf);
        }

        /* === RIGHT WALL detection === */
        if (r0 > r0_th && !seen_r0) {
          seen_r0 = 1;
          LED_ON();
          delay_ms_blocking(30);
          LED_OFF();
          sprintf(buf, "[SPHT] RIGHT APPEARED @ %.0fmm R0=%d\r\n", dist, r0);
          UART_SendString(buf);
        } else if (r0 < r0_off_th && seen_r0) {
          seen_r0 = 0;
          LED_ON();
          delay_ms_blocking(30);
          LED_OFF();
          sprintf(buf, "[SPHT] RIGHT DISAPPEARED @ %.0fmm R0=%d\r\n", dist, r0);
          UART_SendString(buf);
        }

        /* === FRONT WALL detection === */
        if (fsum > front_th && !front_detected) {
          front_detected = 1;
          /* Double blink */
          LED_ON();
          delay_ms_blocking(50);
          LED_OFF();
          delay_ms_blocking(50);
          LED_ON();
          delay_ms_blocking(50);
          LED_OFF();
          sprintf(buf, "[SPHT] FRONT DETECTED @ %.0fmm Fsum=%d (th=%d)\r\n",
                  dist, fsum, front_th);
          UART_SendString(buf);
          Motion_Stop(&motion_ctrl);
          break;
        }

        /* === Button abort === */
        if (Check_Button_Exit()) {
          UART_SendString("[SPHT] Manual STOP!\r\n");
          Motion_Stop(&motion_ctrl);
          break;
        }

        /* === Max distance safety === */
        if (Motion_IsComplete(&motion_ctrl)) {
          UART_SendString("[SPHT] Max distance reached (1000mm)\r\n");
          break;
        }

        /* === Periodic logging (every 100ms) === */
        if (millis() - log_timer > 100) {
          sprintf(buf, "[SPHT] %lu,%.0f,%d,%d,%d,%d,%d,%d,%d,%d\r\n", millis(),
                  dist, l0, r0, l90, r90, l45, r45, seen_l0, seen_r0);
          UART_SendString(buf);
          log_timer = millis();
        }
      }
    }

    /* --- Stop and show results --- */
    Motion_Stop(&motion_ctrl);
    Motion_StopPID();

    {
      float final_dist = motion_ctrl.traveled_distance;
      float final_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

      sprintf(buf, "\r\n[SPHT] === RESULT ===\r\n");
      UART_SendString(buf);
      sprintf(buf, "[SPHT] Distance: %.1f mm\r\n", final_dist);
      UART_SendString(buf);
      sprintf(buf, "[SPHT] Yaw:      %.2f deg\r\n", final_yaw);
      UART_SendString(buf);

      /* OLED result display */
      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("SPHT DONE", &Font_7x10, SSD1306_WHITE);
      sprintf(buf, "D:%.0fmm", final_dist);
      SSD1306_SetCursor(0, 18);
      SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
      sprintf(buf, "Yaw:%.1f", final_yaw);
      SSD1306_SetCursor(0, 40);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(0, 52);
      SSD1306_WriteString("Short:Again Long:Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();

      /* UART mirror of OLED result */
      UART_SendString("[SPHT] OLED Displaying: SPHT DONE\r\n");
      sprintf(buf, "[SPHT] OLED Displaying: D:%.0fmm | Yaw:%.1f\r\n",
              final_dist, final_yaw);
      UART_SendString(buf);
      UART_SendString("[SPHT] Short:Again | Long:Exit\r\n");
    }

    delay_ms_blocking(500); /* Debounce */
  }
}

/* ================= TEST 1-WHEEL TURN (T1WT) ================= */

static void Test_T1WT(void) {
  char buf[80];
  uint8_t run_count = 0;
  uint8_t btn;
  uint16_t l0_th = 150, r0_th = 150;
  uint16_t l0_off_th, r0_off_th;
  uint8_t turn_mode = 0; /* 0=L-turn, 1=U-turn, 2=S-curve */
  static const char *const mode_labels[] = {"L-turn", "U-turn", "S-curve"};

  Perform_Countdown();
  Force_MPU_Calibration();

  Show_Header("Init Motion...");
  UART_SendString("\r\n[T1WT] Init Motion...\r\n");
  Motion_Init(&motion_ctrl);
  Motion_SetSpeedProfile_Explore(&motion_ctrl);
  delay_ms_blocking(100);

  sensor_fusion.ir_calib = ir_calib;

  if (ir_calib.is_calibrated) {
    l0_th = ir_calib.wall_threshold_front_left;
    r0_th = ir_calib.wall_threshold_front_right;
  }
  l0_off_th = (uint16_t)(l0_th * 0.40f);
  r0_off_th = (uint16_t)(r0_th * 0.40f);

  motion_ctrl.wall_steer_enable = 1;

  while (1) {
    run_count++;
    uint8_t turns_remaining; /* How many pivots to do */
    uint8_t current_turn;    /* Which pivot we're on (0-based) */
    float first_turn_angle;  /* Angle of first detected turn */

    /* === MODE + READY SCREEN === */
    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("12. T1WT", &Font_7x10, SSD1306_WHITE);
    sprintf(buf, "#%d %s", run_count, mode_labels[turn_mode]);
    SSD1306_SetCursor(0, 18);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("Short:Mode Long:GO", &Font_7x10, SSD1306_WHITE);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("L=1x U=2x S=3x", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    sprintf(buf, "\r\n--- T1WT #%d mode=%s ---\r\n", run_count,
            mode_labels[turn_mode]);
    UART_SendString(buf);

    /* Button: short=cycle mode, long=start run */
    while (1) {
      btn = Check_Button_Action();
      if (btn == BTN_LONG)
        break; /* Start */
      if (btn == BTN_SHORT) {
        turn_mode = (turn_mode + 1) % 3;
        /* Update display */
        SSD1306_Fill(SSD1306_BLACK);
        SSD1306_SetCursor(0, 0);
        SSD1306_WriteString("12. T1WT", &Font_7x10, SSD1306_WHITE);
        sprintf(buf, "#%d %s", run_count, mode_labels[turn_mode]);
        SSD1306_SetCursor(0, 18);
        SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);
        SSD1306_SetCursor(0, 40);
        SSD1306_WriteString("Short:Mode Long:GO", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0, 52);
        SSD1306_WriteString("L=1x U=2x S=3x", &Font_7x10, SSD1306_WHITE);
        SSD1306_UpdateScreen();
        sprintf(buf, "Mode: %s\r\n", mode_labels[turn_mode]);
        UART_SendString(buf);
      }
      delay_ms_blocking(10);
    }
    while (Hardware_ReadButton(0))
      ; /* Wait for release */
    delay_ms_blocking(100);

    turns_remaining = (turn_mode == 0) ? 1 : 2; /* L=1 turn, U/S=2 turns */
    current_turn = 0;
    first_turn_angle = 0.0f;

    /* === TURN LOOP (1 or 2 iterations) === */
    for (current_turn = 0; current_turn < turns_remaining; current_turn++) {
      SensorFusion_ResetOdometry(&sensor_fusion);
      Hardware_ResetEncoder(0);
      Hardware_ResetEncoder(1);

      motion_ctrl.current_speed = 35.0f;
      motion_ctrl.max_speed = 45.0f;

      Motion_StartPID();
      Motion_StraightConstant(&motion_ctrl, 1000.0f);

      uint8_t seen_l0 = 0, seen_r0 = 0;
      uint8_t seen_l45 = 0, seen_r45 = 0;
      float turn_angle = 0.0f;
      uint8_t turn_detected = 0;
      uint8_t loss_detected = 0;
      float l45_loss_dist = 0.0f;
      static float estimated_k = 60.0f;

      sprintf(buf, "[T1WT] Turn %d/%d running...\r\n", current_turn + 1,
              turns_remaining);
      UART_SendString(buf);

      while (1) {
        if (Check_Button_Exit() || Motion_IsComplete(&motion_ctrl)) {
          Motion_Stop(&motion_ctrl);
          break;
        }

        static uint32_t last_log_time = 0;
        if (millis() - last_log_time >= 50) {
          last_log_time = millis();
          sprintf(buf,
                  "T1WT IR: L45=%d L0=%d R0=%d R45=%d dist=%.1f\r\n",
                  (int)sensor_fusion.ir_sensors[1],
                  (int)sensor_fusion.ir_sensors[2],
                  (int)sensor_fusion.ir_sensors[3],
                  (int)sensor_fusion.ir_sensors[4],
                  motion_ctrl.traveled_distance);
          UART_SendString(buf);
        }

        uint16_t l0 = (uint16_t)sensor_fusion.ir_sensors[2];
        uint16_t r0 = (uint16_t)sensor_fusion.ir_sensors[3];
        uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
        uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];

        if (l0 > l0_th)
          seen_l0 = 1;
        if (r0 > r0_th)
          seen_r0 = 1;
        if (l45 > 80)
          seen_l45 = 1;
        if (r45 > 80)
          seen_r45 = 1;

        if (!loss_detected) {
          if ((seen_l0 && seen_l45 && l45 < 40 && r0 < r0_th + 150) ||
              (seen_r0 && seen_r45 && r45 < 40 && l0 < l0_th + 150)) {
            loss_detected = 1;
            l45_loss_dist = motion_ctrl.traveled_distance;

            if (seen_l0 && seen_l45 && l45 < 40 && r0 < r0_th + 150) {
              turn_angle = 90.0f;
            } else {
              turn_angle = -90.0f;
            }

            /* For U-turn: second turn same dir. For S-curve: flip. */
            if (current_turn == 0)
              first_turn_angle = turn_angle;
            if (current_turn == 1) {
              if (turn_mode == 1)
                turn_angle = first_turn_angle; /* U: same dir */
              else
                turn_angle = -first_turn_angle; /* S: opposite */
            }

            sprintf(buf, "[T1WT] Wall lost! d=%.1f k=%.1f angle=%.0f\r\n",
                    l45_loss_dist, estimated_k, turn_angle);
            UART_SendString(buf);
          }
        }

        if (loss_detected) {
          float k_adjusted =
              estimated_k + (estimated_k * T1WT_EXTRA_NUM / T1WT_EXTRA_DEN);
          if (k_adjusted < 10.0f)
            k_adjusted = 10.0f;

          if (motion_ctrl.traveled_distance - l45_loss_dist >= k_adjusted) {
            UART_SendString("[T1WT] Target reached! Pivoting.\r\n");
            turn_detected = 1;
            break;
          }

          if (turn_angle > 0 && seen_l0 && l0 < l0_off_th) {
            float k_measured = motion_ctrl.traveled_distance - l45_loss_dist;
            if (k_measured > 15.0f && k_measured < 150.0f) {
              estimated_k = k_measured;
              seen_l0 = 0;
            }
          } else if (turn_angle < 0 && seen_r0 && r0 < r0_off_th) {
            float k_measured = motion_ctrl.traveled_distance - l45_loss_dist;
            if (k_measured > 15.0f && k_measured < 150.0f) {
              estimated_k = k_measured;
              seen_r0 = 0;
            }
          }
        }
      }

      if (turn_detected) {
        SensorFusion_ResetOdometry(&sensor_fusion);
        Hardware_ResetEncoder(0);
        Hardware_ResetEncoder(1);

        sprintf(buf, "[T1WT] Pivoting %.1f deg (turn %d)\r\n", turn_angle,
                current_turn + 1);
        UART_SendString(buf);

        motion_ctrl.pivot_turn_speed = 100.0f;
        Motion_SmoothTurn(&motion_ctrl, turn_angle);
        while (!Motion_IsComplete(&motion_ctrl)) {
          if (Check_Button_Exit())
            break;
        }

        float final_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
        sprintf(buf, "[T1WT] Pivot done. Yaw: %.1f deg\r\n", final_yaw);
        UART_SendString(buf);

        /* If more turns coming, drive into next cell */
        if (current_turn + 1 < turns_remaining) {
          Motion_Stop(&motion_ctrl);
          motion_ctrl.current_speed = 35.0f;
          motion_ctrl.max_speed = 45.0f;
          /* Continue to next iteration — StraightConstant will start there */
          sprintf(buf, "[T1WT] Chaining turn %d...\r\n", current_turn + 2);
          UART_SendString(buf);
          continue;
        }

        /* Last turn: advance to cell center */
        float post_turn_dist = CELL_SIZE_MM / 2.0f;
        sprintf(buf, "[T1WT] Advancing %.1f mm\r\n", post_turn_dist);
        UART_SendString(buf);

        Motion_Stop(&motion_ctrl);
        motion_ctrl.current_speed = 35.0f;
        motion_ctrl.max_speed = 45.0f;
        Motion_Straight(&motion_ctrl, post_turn_dist);
        while (!Motion_IsComplete(&motion_ctrl)) {
          if (Check_Button_Exit())
            break;
        }
        motion_ctrl.pivot_turn_speed = 50.0f;
      } else {
        /* T1WT didn't trigger — abort chain */
        break;
      }
    }

    Motion_Stop(&motion_ctrl);
    Motion_StopPID();

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(0, 0);
    sprintf(buf, "T1WT %s DONE", mode_labels[turn_mode]);
    SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    delay_ms_blocking(500);
  }
}

/* ================= PUBLIC API ================= */

void SystemTest_Run(MPU6050_Handle_t *mpu) {
  uint8_t mode;
  uint8_t btn_action;
  const char *mode_name;
  char buf[32];
  const uint8_t MAX_MODE = 7;

  static const char * const names[] = {
    "1.IR", "2.Align", "3.Pivot", "4.TrStr",
    "5.TrTn", "6.SPHT", "7.T1WT", "8.EXIT"
  };

  mode = 0;
  btn_action = BTN_NONE;
  mode_name = "";

  UART_SendString("\r\n=== SYSTEM TEST MENU ===\r\n");

  while (1) {
    mode_name = names[mode];

    SSD1306_Fill(SSD1306_BLACK);
    SSD1306_SetCursor(25, 0);
    SSD1306_WriteString("TEST MENU", &Font_7x10, SSD1306_WHITE);
    SSD1306_DrawLine(0, 12, 128, 12, SSD1306_WHITE);

    sprintf(buf, "< %s >", mode_name);
    SSD1306_SetCursor(0, 25);
    SSD1306_WriteString(buf, &Font_11x18, SSD1306_WHITE);

    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Short:Next Long:OK", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    btn_action = Check_Button_Action();

    if (btn_action == BTN_SHORT) {
      mode++;
      if (mode > MAX_MODE)
        mode = 0;

      /* Log current mode to UART (since OLED is not used) */
      sprintf(buf, "[Mode %d] %s\r\n", mode, names[mode]);
      UART_SendString(buf);
    } else if (btn_action == BTN_LONG) {
      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(30, 25);
      SSD1306_WriteString("SELECTED!", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();

      UART_SendString("Selected: ");
      UART_SendString(mode_name);
      UART_SendString("\r\n");

      while (Hardware_ReadButton(0))
        ;
      delay_ms_blocking(500);

      switch (mode) {
      case 0:
        Test_IR_Sensors();
        break;
      case 1:
        Test_FrontAlign();
        break;
      case 2:
        Test_Smooth_Turn();
        break;
      case 3:
        Test_TransitionStraight();
        break;
      case 4:
        Test_TransitionTurn();
        break;
      case 5:
        Test_SPHT();
        break;
      case 6:
        Test_T1WT();
        break;
      case 7:
        UART_SendString("Exiting test menu...\r\n");
        return;
      }
    }

    delay_ms_blocking(10);
  }
}
