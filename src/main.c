/**
 * @file main.c - FIXED for new Motion API
 * @brief Micromouse System with Menu Selection
 */

#include "back_align.h"
#include "bt_debug.h"
#include "flash_storage.h"
#include "hardware.h"
#include "i2c.h"
#include "ir_sensor.h"
#include "ir_simple_calib.h"
#include "maze_solver.h"
#include "motion_controller.h"
#include "mpu6050.h"
#include "sensor_fusion.h"
#include "ssd1306.h"
#include "stm32f4xx.h"
#include "systemTest.h"
#include "system_timer.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

/* Global Objects */
IR_Simple_Calib_t ir_calib;
Motion_Controller_t motion_ctrl;
Sensor_Fusion_t sensor_fusion;
Maze_t maze;
Maze_Persistent_t persistent_map;
Maze_AStarResult_t astar_result;
MPU_Calib_Flash_t mpu_calib_flash;

/* [FIX] Anti-hang timeout constants */
#define EXPLORE_TIMEOUT_MS 600000 /* 10 minutes max exploration */
#define MOTION_TIMEOUT_MS                                                      \
  2000 /* 4 seconds max per motion (reduced from 8s)                           \
        */
#define ASTAR_MOTION_TIMEOUT_MS 5000 /* A* can need longer on first segments   \
                                      */
#define ASTAR_STALL_TIMEOUT_MS 1500  /* No encoder progress watchdog */
#define ASTAR_HARD_TIMEOUT_MS 15000  /* Last-resort safety timeout */
#define IR_WAIT_TIMEOUT_MS 3000      /* 3 seconds max waiting for IR */
#define MAX_SAME_ACTION_COUNT 20     /* Max consecutive identical actions */
#define ALIGN_STABILITY_DELAY_MS                                               \
  100 /* ms delay after alignment before turning */
#define MIDPOINT_EARLY_MM                                                      \
  0.0f /* Read sensors BEFORE wheel axle reaches cell center */
/* Both explore modes are always compiled; selected at runtime via menu */
#define MIDPOINT_SAMPLES 15 /* Number of samples to average at midpoint */

/* Debug toggles: set to 0 to disable, 1 to enable */
#define EXPLORE_UART_DEBUG 1 /* UART debug prints during exploration */
#define EXPLORE_OLED_DEBUG 0 /* OLED display updates during exploration */

#if EXPLORE_UART_DEBUG
#define EXPLORE_DBG(s) UART_SendString(s)
#else
#define EXPLORE_DBG(s) ((void)0)
#endif

/* 2-threshold hysteresis wall/post transition detection (continuous mode)
 * Tuned from real sensor data: noise=0-9, post_peak=160-430, wall=200-1600+
 *
 * Geometry: when transition triggers at encoder distance d, the robot's TRUE
 * position is at a KNOWN offset from cell start. Remaining = CELL - KNOWN_POS.
 * target = d + remaining  →  never exceeds d + CELL_SIZE. */
#define SIDE_WALL_DETECT_THRESH 80  /* L45/R45 ADC > this → see wall/post  */
#define SIDE_WALL_FADEOFF_THRESH 20 /* L45/R45 ADC < this → transition done */
#define SIDE_WALL_LEVEL_THRESH 350  /* peak > this → full wall, else post   */
#define SIDE_TRAILING_POS_MM 137.0f /* True pos when trailing edge triggers */
#define SIDE_LEADING_POS_MM 163.0f  /* True pos when leading edge triggers  */
#define SIDE_COOLDOWN_MM 50.0f      /* Min distance between events          */
#define SIDE_MIN_DIST_MM 40.0f      /* Ignore events before this distance   */

/* Pivot turn post detection tuning */
#define PIVOT_POST_THRESHOLD                                                   \
  10 /* ADC threshold for diagonal IR post detect (lower = more sensitive) */
#define PIVOT_MIN_TRAVEL_MM 5.0f /* Min travel before post detection triggers  \
                                  */
#define PIVOT_FORWARD_OFFSET_MM                                                \
  0.0f /* Extra mm added to CELL/2 after pivot (tune on hardware) */
#define PIVOT_POST_TURN_ADVANCE_MM                                             \
  40.0f /* A* advance after pivot; shorter than CELL/2 to avoid overshoot */
#define PIVOT_APPROACH_OFFSET_MM                                               \
  0.0f /* Extra mm to drive AFTER post detection BEFORE pivot.                 \
           Compensates sensor-to-axle distance. Tune with 9.Post test. */
#define ASTAR_T1W_PRESENT_TH 80
#define ASTAR_T1W_LOST_TH 40
#define ASTAR_T1W_LOST_RATIO_PCT 45
#define ASTAR_T1W_EXTRA_MIN_MM 50.0f
#define ASTAR_PEAK_CORRECT_MIN_DIST_MM 60.0f

typedef struct {
  uint8_t seen_l45;
  uint8_t seen_r45;
  uint16_t max_l45;
  uint16_t max_r45;
  float cooldown_dist;
  uint16_t on_l45;
  uint16_t off_l45;
  uint16_t on_r45;
  uint16_t off_r45;
  uint16_t lvl_l45;
  uint16_t lvl_r45;
} SideTransitionTracker_t;

static void __attribute__((unused))
SideTransition_Init(SideTransitionTracker_t *trk,
                    const IR_Simple_Calib_t *calib) {
  memset(trk, 0, sizeof(*trk));
  trk->cooldown_dist = -100.0f;

  if (calib->has_open_calib) {
    trk->on_l45 = calib->wall_on_l45;
    trk->off_l45 = calib->wall_off_l45;
    trk->on_r45 = calib->wall_on_r45;
    trk->off_r45 = calib->wall_off_r45;
    trk->lvl_l45 = calib->side_level_l45;
    trk->lvl_r45 = calib->side_level_r45;
  } else {
    trk->on_l45 = SIDE_WALL_DETECT_THRESH;
    trk->off_l45 = SIDE_WALL_FADEOFF_THRESH;
    trk->on_r45 = SIDE_WALL_DETECT_THRESH;
    trk->off_r45 = SIDE_WALL_FADEOFF_THRESH;
    trk->lvl_l45 = SIDE_WALL_LEVEL_THRESH;
    trk->lvl_r45 = SIDE_WALL_LEVEL_THRESH;
  }
}

static void __attribute__((unused))
SideTransition_ArmCurrent(SideTransitionTracker_t *trk, uint16_t l45,
                          uint16_t r45) {
  trk->seen_l45 = (l45 > trk->on_l45) ? 1 : 0;
  trk->seen_r45 = (r45 > trk->on_r45) ? 1 : 0;
  trk->max_l45 = trk->seen_l45 ? l45 : 0;
  trk->max_r45 = trk->seen_r45 ? r45 : 0;
  trk->cooldown_dist = -100.0f;
}

static uint8_t __attribute__((unused))
SideTransition_ApplyCorrection(SideTransitionTracker_t *trk,
                               Motion_Controller_t *mc, char *buf,
                               const char *tag) {
  uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
  uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
  float dist = mc->traveled_distance;
  uint8_t disappear = 0;
  uint8_t appear = 0;

  if (l45 > trk->on_l45 && !trk->seen_l45) {
    appear = 1;
  }
  if (l45 > trk->on_l45) {
    trk->seen_l45 = 1;
    if (l45 > trk->max_l45)
      trk->max_l45 = l45;
  }
  if (l45 < trk->off_l45 && trk->seen_l45) {
    trk->seen_l45 = 0;
    disappear = 1;
  }

  if (r45 > trk->on_r45 && !trk->seen_r45) {
    appear = 1;
  }
  if (r45 > trk->on_r45) {
    trk->seen_r45 = 1;
    if (r45 > trk->max_r45)
      trk->max_r45 = r45;
  }
  if (r45 < trk->off_r45 && trk->seen_r45) {
    trk->seen_r45 = 0;
    disappear = 1;
  }

  if (disappear && dist > SIDE_MIN_DIST_MM &&
      (dist - trk->cooldown_dist > SIDE_COOLDOWN_MM)) {
    uint16_t peak = (trk->max_l45 > trk->max_r45) ? trk->max_l45 : trk->max_r45;
    float remaining = CELL_SIZE_MM - SIDE_TRAILING_POS_MM;

    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);
    mc->traveled_distance = 0.0f;
    mc->target_distance = remaining;
    mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

    trk->cooldown_dist = 0.0f;
    trk->max_l45 = 0;
    trk->max_r45 = 0;

    sprintf(
        buf, "%s:%s d=%.0f rem=%.0f pk=%d\r\n", tag,
        (peak > ((trk->lvl_l45 > trk->lvl_r45) ? trk->lvl_l45 : trk->lvl_r45))
            ? "W"
            : "P",
        dist, remaining, peak);
    UART_SendString(buf);
    return 1;
  }

  if (appear && !disappear && dist > SIDE_MIN_DIST_MM &&
      (dist - trk->cooldown_dist > SIDE_COOLDOWN_MM)) {
    float remaining = CELL_SIZE_MM - SIDE_LEADING_POS_MM;

    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);
    mc->traveled_distance = 0.0f;
    mc->target_distance = remaining;
    mc->start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);

    trk->cooldown_dist = 0.0f;
    trk->max_l45 = 0;
    trk->max_r45 = 0;

    sprintf(buf, "%s:A d=%.0f rem=%.0f\r\n", tag, dist, remaining);
    UART_SendString(buf);
    return 1;
  }

  return 0;
}

/* ========================================================================== */
/* Forward declarations needed by WaitForHandStart                            */
/* ========================================================================== */
typedef enum { STATE_MENU, STATE_RUNNING } System_State_t;
static System_State_t system_state;

/* ========================================================================== */
/* HAND START HELPER — Place hand on L90 then remove → 2s countdown → GO     */
/* ========================================================================== */
static void WaitForHandStart(void) {
  uint16_t l90_thresh;
  uint32_t countdown_start = 0;
  uint8_t phase = 0; /* 0=waiting hand, 1=hand detected, 2=countdown */
  char buf[32];
  uint32_t hand_wait_start;
  uint16_t l90_val;

  /* Use 20% of calibrated L90 threshold as hand-detection trigger */
  if (ir_calib.is_calibrated == IR_CALIB_MAGIC) {
    l90_thresh = (uint16_t)(ir_calib.wall_threshold_left * 0.2f);
  } else {
    l90_thresh = 400; /* Fallback */
  }

  /* [CRITICAL FIX] Force IR sensor reset on every hand-start entry.
   * After explore exit, TIM11 stops → sensor_fusion.ir_sensors[] frozen.
   * Old values > 10 would bypass the direct-read fallback below,
   * making hand detection permanently unresponsive until reset. */
  IR_Sensor_Reset();
  {
    uint8_t i;
    for (i = 0; i < 6; i++)
      sensor_fusion.ir_sensors[i] = 0.0f;
  }
  IR_Sensor_StartScan();

  SSD1306_Fill(SSD1306_BLACK);
  SSD1306_SetCursor(0, 0);
  SSD1306_WriteString("READY", &Font_7x10, SSD1306_WHITE);
  SSD1306_SetCursor(0, 20);
  SSD1306_WriteString("Place hand on", &Font_7x10, SSD1306_WHITE);
  SSD1306_SetCursor(0, 35);
  SSD1306_WriteString("L90 to start", &Font_7x10, SSD1306_WHITE);
  SSD1306_UpdateScreen();

  UART_SendString("Hand start: waiting for L90...\r\n");

  hand_wait_start = millis();

  while (1) {
    /* Timeout: return to menu after 5 seconds with no hand */
    if (phase == 0 && (millis() - hand_wait_start) > 5000) {
      UART_SendString("TIMEOUT: No hand detected in 5s\r\n");
      system_state = STATE_MENU;
      return;
    }

    /* Read L90 from sensor fusion (updated by ISR) or direct scan */
    l90_val = (uint16_t)sensor_fusion.ir_sensors[0];
    if (l90_val < 10) {
      /* Sensor fusion not running yet, do direct read */
      if (IR_Sensor_IsReady()) {
        uint16_t raw[6];
        IR_Sensor_GetResults(raw);
        l90_val = raw[0];
        IR_Sensor_StartScan();
      }
    }

    switch (phase) {
    case 0: /* Waiting for hand to be placed */
      if (l90_val > l90_thresh) {
        phase = 1;
        UART_SendString("Hand detected! Remove to start...\r\n");
        SSD1306_Fill(SSD1306_BLACK);
        SSD1306_SetCursor(0, 0);
        SSD1306_WriteString("HAND OK", &Font_7x10, SSD1306_WHITE);
        SSD1306_SetCursor(0, 25);
        SSD1306_WriteString("Remove hand", &Font_7x10, SSD1306_WHITE);
        SSD1306_UpdateScreen();
      }
      break;

    case 1: /* Hand detected, waiting for removal */
      if (l90_val < l90_thresh) {
        phase = 2;
        countdown_start = millis();
        UART_SendString("Hand removed! Countdown...\r\n");
      }
      break;

    case 2: /* Countdown after hand removed */ {
      uint32_t elapsed = millis() - countdown_start;
      uint8_t remaining =
          (elapsed < 2000) ? (2 - (uint8_t)(elapsed / 1000)) : 0;

      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("STARTING", &Font_7x10, SSD1306_WHITE);
      SSD1306_SetCursor(50, 25);
      sprintf(buf, "%d", remaining);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen();

      if (elapsed >= 2000) {
        /* Countdown complete — GO! */
        SSD1306_Fill(SSD1306_BLACK);
        SSD1306_SetCursor(30, 20);
        SSD1306_WriteString("GO!", &Font_7x10, SSD1306_WHITE);
        SSD1306_UpdateScreen();
        UART_SendString("GO!\r\n");
        delay_ms_blocking(300);
        return;
      }
      break;
    }
    }

    /* Allow exit via button */
    if (Hardware_ReadButton(0)) {
      system_state = STATE_MENU;
      return;
    }

    delay_ms_blocking(10);
  }
}
/* ========================================================================== */
/* MENU SYSTEM                                                                */
/* ========================================================================== */

typedef enum {
  MENU_CALIBRATION = 0,
  MENU_EXPLORE_CBC,
  MENU_EXPLORE_CONT,
  MENU_ASTAR_RUN,
  MENU_SPEEDRUN,
  MENU_RESET_MAP,
  MENU_SYSTEM_TEST,
  MENU_COUNT
} Menu_Item_t;

static Menu_Item_t selected_mode = MENU_CALIBRATION;
static uint8_t menu_selection = 0;

const char *menu_items[] = {"1.Calibration", "2.Run Slow", "3.Run Faster",
                            "4.A* Run",      "5.Speedrun", "6.Reset Map",
                            "7.System Test"};

/* ========================================================================== */
/* DISPLAY FUNCTIONS                                                          */
/* ========================================================================== */

void Display_Menu(void) {
  uint8_t i;
  uint8_t y_pos;
  uint8_t items_per_page = 4;
  uint8_t start_index = 0;

  if (menu_selection >= items_per_page) {
    start_index = menu_selection - (items_per_page - 1);
  }

  SSD1306_Fill(SSD1306_BLACK);

  SSD1306_SetCursor(10, 0);
  SSD1306_WriteString("SELECT MODE", &Font_7x10, SSD1306_WHITE);
  SSD1306_DrawLine(0, 12, 127, 12, SSD1306_WHITE);

  for (i = 0; i < items_per_page; i++) {
    uint8_t current_item_idx = start_index + i;

    if (current_item_idx >= MENU_COUNT)
      break;

    y_pos = 15 + i * 11;

    if (current_item_idx == menu_selection) {
      SSD1306_SetCursor(0, y_pos);
      SSD1306_WriteString(">", &Font_7x10, SSD1306_WHITE);
    }

    SSD1306_SetCursor(10, y_pos);
    SSD1306_WriteString((char *)menu_items[current_item_idx], &Font_7x10,
                        SSD1306_WHITE);
  }

  if (start_index > 0) {
    SSD1306_DrawPixel(127, 15, SSD1306_WHITE);
  }
  if (start_index + items_per_page < MENU_COUNT) {
    SSD1306_DrawPixel(127, 63, SSD1306_WHITE);
  }

  SSD1306_UpdateScreen();
}

void Display_Running(const char *mode_name) {
  SSD1306_Fill(SSD1306_BLACK);

  SSD1306_SetCursor(0, 0);
  SSD1306_WriteString(mode_name, &Font_7x10, SSD1306_WHITE);

  SSD1306_SetCursor(0, 20);
  SSD1306_WriteString("Running...", &Font_7x10, SSD1306_WHITE);

  SSD1306_SetCursor(0, 50);
  SSD1306_WriteString("Press=Exit", &Font_7x10, SSD1306_WHITE);

  SSD1306_UpdateScreen();
}

void Display_Status(const char *title, const char *message) {
  SSD1306_Fill(SSD1306_BLACK);

  SSD1306_SetCursor(10, 15);
  SSD1306_WriteString(title, &Font_7x10, SSD1306_WHITE);

  SSD1306_SetCursor(10, 35);
  SSD1306_WriteString(message, &Font_7x10, SSD1306_WHITE);

  SSD1306_UpdateScreen();
}

void Execute_Calibration(void) {
  uint16_t ir_samples[6];
  uint8_t i;
  uint8_t sub_mode = 0; /* 0=ALL, 1-6=individual steps */
  char buf[96];

  static const char *const sub_names[] = {"0.ALL",    "1.Center", "2.Open",
                                          "3.FrDiff", "4.Diag45", "5.L-Wall",
                                          "6.R-Wall"};

  UART_SendString("\r\n=== CALIBRATION MODE ===\r\n");

  while (Hardware_ReadButton(0)) {
    delay_ms_blocking(10);
  }

  /* ---- Sub-mode selection: short=next, long=confirm ---- */
  UART_SendString("Select step: Short=Next, Long=Confirm\r\n");
  UART_SendString("  0=ALL 1=Ctr 2=Open 3=FrDiff 4=Diag\r\n");
  UART_SendString("  5=L-Wall 6=R-Wall\r\n");

  {
    uint32_t press_start = 0;
    uint8_t prev_btn = 0, confirmed = 0, is_long = 0;

    sprintf(buf, "< %s >", sub_names[sub_mode]);
    Display_Status("CALIB STEP", buf);
    SSD1306_SetCursor(0, 52);
    SSD1306_WriteString("Hold to confirm", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();

    while (!confirmed) {
      uint8_t btn = Hardware_ReadButton(0);

      if (btn && !prev_btn) {
        delay_ms_blocking(20);
        if (Hardware_ReadButton(0)) {
          press_start = millis();
          is_long = 0;
        }
      }
      if (btn && press_start > 0 && (millis() - press_start > 800)) {
        is_long = 1;
        confirmed = 1;
      }
      if (!btn && prev_btn) {
        if (!is_long && press_start > 0) {
          sub_mode = (sub_mode + 1) % 7;
          sprintf(buf, "< %s >", sub_names[sub_mode]);
          Display_Status("CALIB STEP", buf);
          SSD1306_SetCursor(0, 52);
          SSD1306_WriteString("Hold to confirm", &Font_7x10, SSD1306_WHITE);
          SSD1306_UpdateScreen();
          sprintf(buf, "-> Step: %s\r\n", sub_names[sub_mode]);
          UART_SendString(buf);
        }
        press_start = 0;
      }
      prev_btn = btn;
      delay_ms_blocking(10);
    }
  }

  sprintf(buf, "\r\n+++ Running: %s +++\r\n", sub_names[sub_mode]);
  UART_SendString(buf);

  while (Hardware_ReadButton(0)) {
    delay_ms_blocking(10);
  }

  /* ---- Init hardware ---- */
  Motion_Init(&motion_ctrl);
  Motion_SetSpeedProfile_Explore(&motion_ctrl);
  Motion_StartPID();
  delay_ms_blocking(100);

  Display_Status("CALIBRATING", "MPU6050...");
  UART_SendString("Calibrating MPU6050...\r\n");
  UART_SendString("Keep robot STILL!\r\n");
  delay_ms_blocking(100);
  SensorFusion_Init(&sensor_fusion);
  UART_SendString("MPU6050 calibration complete!\r\n");

  /* ---- Load existing calib from flash (for partial updates) ---- */
  if (sub_mode != 0) {
    Flash_LoadIRCalib(&ir_calib);
    if (ir_calib.is_calibrated != 0xCA) {
      UART_SendString("WARN: No saved calib, init defaults\r\n");
      IR_Simple_Init(&ir_calib);
    } else {
      UART_SendString("Loaded existing calib from flash\r\n");
    }
  } else {
    IR_Simple_Init(&ir_calib);
  }
  ir_calib.calib_mode = 2;

  /* ---- IR test scans ---- */
  Display_Status("CALIBRATING", "IR Sensors...");
  UART_SendString("\r\n=== IR SENSOR CALIBRATION ===\r\n");
  UART_SendString("Testing IR sensors (3 scans)...\r\n");
  for (i = 0; i < 3; i++) {
    if (!IR_Sensor_IsReady())
      IR_Sensor_StartScan();
    while (!IR_Sensor_IsReady()) {
    }
    IR_Sensor_GetResults(ir_samples);
    sprintf(buf, "  Scan %d: L90=%d L45=%d L0=%d R0=%d R45=%d R90=%d\r\n",
            i + 1, ir_samples[0], ir_samples[1], ir_samples[2], ir_samples[3],
            ir_samples[4], ir_samples[5]);
    UART_SendString(buf);
    delay_ms_blocking(100);
  }

  /* Step 1: Center (90/0) */
  if (sub_mode == 0 || sub_mode == 1) {
    UART_SendString("\r\n[Step 1] L+F+R walls, robot moves ~20mm\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 1", "L+F+R walls");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(500);
    ir_calib.calib_mode = 1;
    IR_Simple_Calib90and0(&ir_calib, &motion_ctrl);
    ir_calib.calib_mode = 2;
  }

  /* Step 2: Open-space */
  if (sub_mode == 0 || sub_mode == 2) {
    UART_SendString("\r\n[Step 2] Open space (no walls)\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 2", "OPEN space");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(500);
    IR_Simple_CalibOpenSpace(&ir_calib);
  }

  /* Step 3: Front-diff */
  if (sub_mode == 0 || sub_mode == 3) {
    UART_SendString("\r\n[Step 3] Face FRONT wall (perpendicular)\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 3", "FRONT DIFF");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(500);
    IR_Simple_CalibFrontDiff(&ir_calib);
  }

  /* Step 4: 45-degree diagonal */
  if (sub_mode == 0 || sub_mode == 4) {
    UART_SendString("\r\n[Step 4] Corridor L+R walls, moves ~50mm\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 4", "L+R walls");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(500);
    ir_calib.calib_mode = 1;
    IR_Simple_Calib45(&ir_calib, &motion_ctrl);
    ir_calib.calib_mode = 2;
  }

  /* Step 5: Left wall proximity */
  if (sub_mode == 0 || sub_mode == 5) {
    UART_SendString("\r\n[Step 5] Push against LEFT wall\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 5", "LEFT wall");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(300);
    IR_Simple_CalibLeftWall(&ir_calib);
  }

  /* Step 6: Right wall proximity */
  if (sub_mode == 0 || sub_mode == 6) {
    UART_SendString("\r\n[Step 6] Push against RIGHT wall\r\n");
    UART_SendString("Press button when ready...\r\n");
    Display_Status("Step 6", "RIGHT wall");
    SSD1306_SetCursor(0, 45);
    SSD1306_WriteString("Press button", &Font_7x10, SSD1306_WHITE);
    SSD1306_UpdateScreen();
    while (!Hardware_ReadButton(0))
      ;
    delay_ms_blocking(300);
    IR_Simple_CalibRightWall(&ir_calib);
  }

  /* Recompute thresholds if wall proximity was updated */
  if (sub_mode == 0 || sub_mode == 5 || sub_mode == 6) {
    IR_Simple_RefineThresholds(&ir_calib);
  }

  /* ---- Finalize ---- */
  ir_calib.calib_mode = 2;
  ir_calib.is_calibrated = 0xCA;
  IR_Simple_ComputeHysteresis(&ir_calib);
  IR_Simple_PrintCalib(&ir_calib);

  Display_Status("CALIBRATING", "Encoders...");
  UART_SendString("Resetting encoders...\r\n");
  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);
  delay_ms_blocking(50);

  Display_Status("CALIBRATION", "COMPLETE!");
  UART_SendString("\r\n=== CALIBRATION COMPLETE ===\r\n");

  Flash_SaveIRCalib(&ir_calib);
  UART_SendString("IR calibration saved to Flash.\r\n");

  mpu_calib_flash.gyro_z_offset = sensor_fusion.mpu_handle.gyro_z_offset;
  Flash_SaveMPUCalib(&mpu_calib_flash);
  UART_SendString("MPU calibration saved to Flash.\r\n");

  LED_ON();
  delay_ms_blocking(500);
  LED_OFF();

  Motion_Stop(&motion_ctrl);
  Motion_StopPID();
  system_state = STATE_MENU;
}

/* ========================================================================== */
/* Helper: scan IR and update maze walls at current position */
/* ========================================================================== */
static void Explore_ScanAndUpdateMaze(Maze_t *m, IR_Simple_Calib_t *cal,
                                      const char *tag) {
  uint16_t scan_ir[6];
  uint8_t sl, sf, sr;
  char dbg[64];

  IR_Sensor_StartScan();
  {
    uint32_t iw = millis();
    while (!IR_Sensor_IsReady() && (millis() - iw < 20))
      ;
  }
  if (IR_Sensor_IsReady()) {
    IR_Sensor_GetResults(scan_ir);
    sl = IR_Simple_DetectLeftWall(cal, scan_ir);
    sf = IR_Simple_DetectFrontWall(cal, scan_ir);
    sr = IR_Simple_DetectRightWall(cal, scan_ir);
    Maze_AddWalls(m, sl, sf, sr);
    Maze_SendCellUpdate(m);
    sprintf(dbg, "%s:(%d,%d) D:%d W:L%dF%dR%d\r\n", tag, m->robot_x, m->robot_y,
            m->robot_dir, sl, sf, sr);
    UART_SendString(dbg);
  }
}

/* ========================================================================== */
/* CELL-BY-CELL EXPLORE — stop at each cell center, blocking IR scan          */
/* ========================================================================== */

/* Peak-based post detection constants (cell-by-cell mode) */
#define PEAK_SLOPE_THRESH 10 /* Min ADC delta per sample to count as slope */
#define PEAK_CONFIRM 3       /* Consecutive opposite-slope samples to confirm */
#define PEAK_MIN_VALUE 40    /* Ignore peaks below this ADC value */
#define TREND_FLAT 0
#define TREND_RISING 1
#define TREND_FALLING 2
#define POST_EXTRA_MM 60.0f /* Drive further after post edge to cell center */

#define ADVANCED_CELL_BY_CELL 0 /* 1: Front align & peak snap. 0: disabled */
#define ENABLE_BACK_ALIGN                                                      \
  1 /* 1: Back-align before L/R turns. Independent of above. */

/* Start-cell (0,0) can be slightly off-center after hand start. Relax side
 * detection once and use diagonal backup to avoid missing obvious walls. */
#define START_SIDE_RELAX_RATIO 0.90f

static uint8_t DetectStartLeftWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw) {
  uint16_t l0_th = calib->wall_threshold_front_left;
  uint16_t l45_th = calib->wall_threshold_l45;
  uint16_t l0_relaxed;
  uint16_t l45_relaxed;

  if (l0_th < 10)
    l0_th = 10;
  if (l45_th < 10)
    l45_th = 10;

  l0_relaxed = (uint16_t)(l0_th * START_SIDE_RELAX_RATIO);
  l45_relaxed = (uint16_t)(l45_th * START_SIDE_RELAX_RATIO);

  return (ir_raw[2] >= l0_relaxed || ir_raw[1] >= l45_relaxed) ? 1 : 0;
}

static uint8_t DetectStartRightWall(IR_Simple_Calib_t *calib,
                                    uint16_t *ir_raw) {
  uint16_t r0_th = calib->wall_threshold_front_right;
  uint16_t r45_th = calib->wall_threshold_r45;
  uint16_t r0_relaxed;
  uint16_t r45_relaxed;

  if (r0_th < 10)
    r0_th = 10;
  if (r45_th < 10)
    r45_th = 10;

  r0_relaxed = (uint16_t)(r0_th * START_SIDE_RELAX_RATIO);
  r45_relaxed = (uint16_t)(r45_th * START_SIDE_RELAX_RATIO);

  return (ir_raw[3] >= r0_relaxed || ir_raw[4] >= r45_relaxed) ? 1 : 0;
}

void Execute_MazeExplore(uint8_t fast_mode) {
  typedef enum {
    EXPLORE_START_CELL,
    EXPLORE_MOVING,
    EXPLORE_DECIDE_TURN,
    EXPLORE_WAIT_MANEUVER,
    EXPLORE_WAIT_BTURN_STOP,     /* Non-blocking B-turn: wait for stop */
    EXPLORE_WAIT_BTURN_TURN,     /* Non-blocking B-turn: wait for 180 turn */
    EXPLORE_WAIT_INPLACE_STOP,   /* In-place turn: wait for stop */
    EXPLORE_WAIT_INPLACE_TURN,   /* In-place turn: wait for 90 turn */
    EXPLORE_FRONT_ALIGN,         /* Front wall alignment before turning */
    EXPLORE_POST_TURN_ALIGN,     /* Front wall alignment AFTER turn completes */
    EXPLORE_WAIT_FINISH_CELL,    /* Wait for remaining cell distance before L/R
                                    turn */
    EXPLORE_STABILITY_WAIT,      /* Delay after alignment before turning */
    EXPLORE_BACK_ALIGN_PRIMITIVE /* Executes the standalone back_align
                                    macro-maneuver */
  } Explore_State_t;

  Explore_State_t explore_state;
  char buf[128];
  /* Direction deltas: North(0), East(1), South(2), West(3) */
  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {1, 0, -1, 0};
  uint16_t ir_raw[6];
  uint8_t left_wall, front_wall, right_wall;
  uint8_t next_move;
  float inplace_turn_angle; /* Track angle for in-place turn states */
#if EXPLORE_OLED_DEBUG
  uint32_t last_display_time = 0;
#endif
#if ADVANCED_CELL_BY_CELL
  uint16_t post_ir[6]; /* IR snapshot after turn for post-turn front align */
#endif
  Explore_State_t stability_next_state; /* Where to go after stability wait */
  uint32_t stability_start_time;

  /* [FIX] Anti-hang: timeouts and stuck detection */
  uint32_t explore_start_time;
  uint32_t motion_start_time;
  uint8_t last_action = 0;
  uint8_t same_action_count = 0;

  /* [NEW] Canonical back alignment */
  BackAlign_t back_align;
#if ENABLE_BACK_ALIGN || ADVANCED_CELL_BY_CELL
  TurnContext_t turn_ctx;
  uint8_t back_align_cooldown = 0; /* 0=allow, >0=skip (do-skip-skip pattern) */
  uint8_t back_align_skip_after_use = fast_mode ? 1 : 2;
#endif

  /* [NEW] Peak-based post detection */
#if ADVANCED_CELL_BY_CELL
  uint16_t post_prev_l45 = 0, post_prev_r45 = 0;
  uint16_t post_peak_l45 = 0, post_peak_r45 = 0;
  uint8_t post_trend_l45 = TREND_FLAT, post_trend_r45 = TREND_FLAT;
  uint8_t post_fall_l45 = 0, post_fall_r45 = 0;
  uint8_t post_rise_l45 = 0, post_rise_r45 = 0;
  uint8_t post_corrected = 0;
  uint8_t post_armed = 0;
#endif

  /* [NEW] Hysteresis wall detection (from SPHT) for logging */
  uint16_t l0_th = 0, r0_th = 0;
  uint16_t l0_off_th = 0, r0_off_th = 0;
  uint8_t seen_l0 = 0, seen_r0 = 0;

  next_move = 'F';
  inplace_turn_angle = 0.0f;
  stability_next_state = EXPLORE_START_CELL;
  stability_start_time = 0;
  explore_start_time = millis();
  motion_start_time = millis();

  if (!ir_calib.is_calibrated) {
    UART_SendString(
        "WARNING: IR not calibrated! Using default thresholds.\r\n");
  }

  UART_SendString("\r\n=== MAZE EXPLORATION MODE ===\r\n");
  if (ir_calib.is_calibrated) {
    sprintf(buf, "Thresholds: L90=%d L45=%d R45=%d R90=%d\r\n",
            ir_calib.wall_threshold_left, ir_calib.wall_threshold_l45,
            ir_calib.wall_threshold_r45, ir_calib.wall_threshold_right);
    UART_SendString(buf);

    /* Setup thresholds for SPHT wall detection logging using coeff */
    l0_th = ir_calib.wall_threshold_front_left;
    r0_th = ir_calib.wall_threshold_front_right;
  } else {
    sprintf(buf, "Thresholds: DEFAULT (500)\r\n");
    UART_SendString(buf);
    l0_th = 150;
    r0_th = 150;
  }
  l0_off_th = (uint16_t)(l0_th * 0.40f);
  r0_off_th = (uint16_t)(r0_th * 0.40f);

  UART_SendString("Press button to exit\r\n");

  UART_SendString("[DBG] Before Display_Running\r\n");
  Display_Running("EXPLORING");
  UART_SendString("[DBG] After Display_Running\r\n");

  /* ===== MPU6050: Use flash offset or fallback calibrate ===== */
  if (mpu_calib_flash.is_valid == MPU_CALIB_MAGIC) {
    sensor_fusion.mpu_handle.gyro_z_offset = mpu_calib_flash.gyro_z_offset;
    EXPLORE_DBG("MPU: using flash offset\r\n");
  } else {
    Display_Status("MPU CALIB", "Keep still!");
    EXPLORE_DBG("No flash MPU calib, running live...\r\n");
    delay_ms_blocking(500);
    MPU6050_Calibrate(&sensor_fusion.mpu_handle);
  }

  SensorFusion_ResetOdometry(&sensor_fusion);
  sensor_fusion.gyro_z = 0.0f;
  MPU6050_ResetYaw(&sensor_fusion.mpu_handle);
  UART_SendString("[DBG] Fusion reset OK\r\n");

  /* Hand start: wait for L90 hand detection + 2s countdown */
  WaitForHandStart();
  if (system_state == STATE_MENU)
    return;

  /* Motion: Only reset state, do NOT re-run Motion_Init() which
   * reconfigures TIM11 registers and NVIC from scratch. */
  Motion_Stop(&motion_ctrl);
  motion_ctrl.current_speed = 0.0f;
  if (fast_mode) {
    Motion_SetSpeedProfile_ExploreFast(&motion_ctrl);
  } else {
    Motion_SetSpeedProfile_Explore(&motion_ctrl);
  }
  motion_ctrl.stabilize_duration = 50; /* Cell-by-cell: fast stabilize */
  sensor_fusion.ir_calib = ir_calib;
  UART_SendString("[DBG] Motion reset OK\r\n");

  Maze_Init(&maze);
  /* Load persistent knowledge into per-run maze */
  Maze_Persistent_LoadInto(&persistent_map, &maze);
  Maze_Persistent_IncrementVisit(&persistent_map, 0, 0); /* Start cell */
  UART_SendString("[DBG] Maze init + persistent load OK\r\n");

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  Motion_StartPID(); /* Ensure PID interrupt starts running */
  UART_SendString("[DBG] PID started, entering loop\r\n");

  LED_ON();

  /* === Initial multi-scan at (0,0) — average before first move === */
  UART_SendString("[DBG] Scanning (0,0) multi-sample...\r\n");
  {
    uint8_t s, i_init;
    uint8_t valid_samples = 0;
    uint32_t ir_sum[6] = {0, 0, 0, 0, 0, 0};
    uint16_t scan_buf[6];

    for (s = 0; s < MIDPOINT_SAMPLES; s++) {
      IR_Sensor_StartScan();
      {
        uint32_t iw = millis();
        while (!IR_Sensor_IsReady() && (millis() - iw < 20))
          ;
      }
      if (IR_Sensor_IsReady()) {
        IR_Sensor_GetResults(scan_buf);
        for (i_init = 0; i_init < 6; i_init++) {
          ir_sum[i_init] += scan_buf[i_init];
        }
        valid_samples++;
      }
    }

    if (valid_samples > 0) {
      for (i_init = 0; i_init < 6; i_init++) {
        ir_raw[i_init] = (uint16_t)(ir_sum[i_init] / valid_samples);
      }

      left_wall = DetectStartLeftWall(&ir_calib, ir_raw);
      front_wall = IR_Simple_DetectFrontWall(&ir_calib, ir_raw);
      right_wall = DetectStartRightWall(&ir_calib, ir_raw);

      sprintf(buf, "InitSamp:%d/%d\r\n", valid_samples, MIDPOINT_SAMPLES);
      EXPLORE_DBG(buf);
      sprintf(buf, "Init:(%d,%d) D:%d W:L%dF%dR%d\r\n", maze.robot_x,
              maze.robot_y, maze.robot_dir, left_wall, front_wall, right_wall);
      EXPLORE_DBG(buf);
      sprintf(buf, "  IR:%d %d %d %d %d %d\r\n", ir_raw[0], ir_raw[1],
              ir_raw[2], ir_raw[3], ir_raw[4], ir_raw[5]);
      EXPLORE_DBG(buf);

      Maze_UpdateWalls(&maze, left_wall, front_wall, right_wall);
      Maze_SendCellUpdate(&maze);
      Maze_Persistent_MarkObserved(&persistent_map, maze.robot_x, maze.robot_y,
                                   maze.robot_dir);

      Maze_FloodFillCenter(&maze);
      next_move = Maze_GetNextMove_Smart(&maze, &persistent_map);

      /* If front wall at (0,0) — align first, then decide turn */
      if (front_wall && next_move != 'F') {
        Motion_FrontAlign(&motion_ctrl, &ir_calib);
        motion_start_time = millis();
        EXPLORE_DBG("ALIGN: start (0,0)\r\n");
        explore_state = EXPLORE_FRONT_ALIGN;
      } else {
        explore_state = EXPLORE_DECIDE_TURN;
      }
    } else {
      EXPLORE_DBG("WARN: IR not ready at (0,0), driving blind\r\n");
      explore_state = EXPLORE_START_CELL;
    }
  }

  while (system_state == STATE_RUNNING) {

    /* [CRITICAL] Process DMA display transfers to avoid I2C deadlock
     * with TIM11 ISR. Never use blocking SSD1306_UpdateScreen() while
     * PID is running! */
    SSD1306_Process_DMA();

    /* [FIX] Global exploration timeout */
    if ((millis() - explore_start_time) > EXPLORE_TIMEOUT_MS) {
      EXPLORE_DBG("TIMEOUT: Exploration exceeded 2min limit!\r\n");
      Motion_Stop(&motion_ctrl);
      Hardware_StopMotors();
      Maze_Persistent_SyncFrom(&persistent_map, &maze);
      persistent_map.total_runs++;
      Flash_SaveMaze(&persistent_map);
      system_state = STATE_MENU;
      break;
    }

    /* [FIX] Per-motion timeout: force-complete stuck motions */
    /* Skip during back-align — it manages its own timing internally */
    if (explore_state != EXPLORE_BACK_ALIGN_PRIMITIVE &&
        !Motion_IsComplete(&motion_ctrl) &&
        (millis() - motion_start_time) > MOTION_TIMEOUT_MS) {
      sprintf(buf, "WARN: Motion timeout! State=%d\r\n", motion_ctrl.state);
      EXPLORE_DBG(buf);
      Motion_Stop(&motion_ctrl);
      /* Return to safe state */
      explore_state = EXPLORE_START_CELL;
    }

    /* [CLEANUP] Sensors updated by TIM11 @ 1kHz */

    switch (explore_state) {
    case EXPLORE_START_CELL:
      EXPLORE_DBG("[DBG] START_CELL\r\n");
      /* Cell-by-cell: stop at end of each cell for reliable sensing */
      Motion_Straight(&motion_ctrl, CELL_SIZE_MM);
#if ADVANCED_CELL_BY_CELL
      post_corrected = 0;
      post_prev_l45 = (uint16_t)sensor_fusion.ir_sensors[1];
      post_prev_r45 = (uint16_t)sensor_fusion.ir_sensors[4];
      post_peak_l45 = 0;
      post_peak_r45 = 0;
      post_trend_l45 = TREND_FLAT;
      post_trend_r45 = TREND_FLAT;
      post_fall_l45 = 0;
      post_fall_r45 = 0;
      post_rise_l45 = 0;
      post_rise_r45 = 0;
      post_armed = 0;
#endif

      seen_l0 = (sensor_fusion.ir_sensors[2] > l0_th) ? 1 : 0;
      seen_r0 = (sensor_fusion.ir_sensors[3] > r0_th) ? 1 : 0;

      motion_start_time = millis();
#if EXPLORE_OLED_DEBUG
      last_display_time = 0; /* Allow OLED redraw when this cell completes */
#endif
      explore_state = EXPLORE_MOVING;
      break;

    case EXPLORE_MOVING:
      /* Safety: If position went out of bounds, abort */
      if (maze.robot_x >= MAZE_SIZE || maze.robot_y >= MAZE_SIZE) {
        sprintf(buf, "ERR: OOB pos (%d,%d)!\r\n", maze.robot_x, maze.robot_y);
        EXPLORE_DBG(buf);
        Motion_Stop(&motion_ctrl);
        system_state = STATE_MENU;
        break;
      }

      /* === Peak-based post detection (while moving) === */
#if ADVANCED_CELL_BY_CELL
      if (!post_corrected) {
        uint16_t l45 = (uint16_t)sensor_fusion.ir_sensors[1];
        uint16_t r45 = (uint16_t)sensor_fusion.ir_sensors[4];
        int16_t dl = (int16_t)l45 - (int16_t)post_prev_l45;
        int16_t dr = (int16_t)r45 - (int16_t)post_prev_r45;
        uint8_t pk_found = 0;

        if (motion_ctrl.traveled_distance < 40.0f) {
          post_prev_l45 = l45;
          post_prev_r45 = r45;
        } else {
          post_armed = 1;

          /* L45 slope */
          if (dl > PEAK_SLOPE_THRESH) {
            post_trend_l45 = TREND_RISING;
            post_rise_l45++;
            post_fall_l45 = 0;
            if (l45 > post_peak_l45)
              post_peak_l45 = l45;
          } else if (dl < -PEAK_SLOPE_THRESH) {
            post_fall_l45++;
            post_rise_l45 = 0;
            if (post_trend_l45 == TREND_RISING &&
                post_peak_l45 > PEAK_MIN_VALUE && post_fall_l45 >= PEAK_CONFIRM)
              pk_found = 1;
            post_trend_l45 = TREND_FALLING;
          }
          /* R45 slope */
          if (dr > PEAK_SLOPE_THRESH) {
            post_trend_r45 = TREND_RISING;
            post_rise_r45++;
            post_fall_r45 = 0;
            if (r45 > post_peak_r45)
              post_peak_r45 = r45;
          } else if (dr < -PEAK_SLOPE_THRESH) {
            post_fall_r45++;
            post_rise_r45 = 0;
            if (post_trend_r45 == TREND_RISING &&
                post_peak_r45 > PEAK_MIN_VALUE && post_fall_r45 >= PEAK_CONFIRM)
              pk_found = 1;
            post_trend_r45 = TREND_FALLING;
          }
          /* Sustained drop (wall disappearing) */
          if (!pk_found && post_armed) {
            if (post_trend_l45 == TREND_FALLING &&
                post_fall_l45 >= PEAK_CONFIRM && post_peak_l45 > PEAK_MIN_VALUE)
              pk_found = 1;
            if (post_trend_r45 == TREND_FALLING &&
                post_fall_r45 >= PEAK_CONFIRM && post_peak_r45 > PEAK_MIN_VALUE)
              pk_found = 1;
          }

          if (pk_found && motion_ctrl.traveled_distance > 80.0f) {
            pk_found = 0; /* Too far — likely next cell boundary post */
          }

          if (pk_found) {
            /* Cell-by-cell: snap traveled_distance to CELL_SIZE_MM/2
             * (known position at cell boundary). Keep target_distance = 180mm
             * so remaining distance = 180 - 90 = 90mm exactly. */
            float old_dist = motion_ctrl.traveled_distance;
            motion_ctrl.traveled_distance = CELL_SIZE_MM / 2.0f;
            sprintf(buf, "PEAK: %.0f->%.0f\r\n", old_dist, CELL_SIZE_MM / 2.0f);
            EXPLORE_DBG(buf);
            post_corrected = 1;
          }
          post_prev_l45 = l45;
          post_prev_r45 = r45;
        }
      }
#endif

      /* === Hysteresis wall detection logging (from SPHT) === */
      {
        uint16_t l0 = (uint16_t)sensor_fusion.ir_sensors[2];
        uint16_t r0 = (uint16_t)sensor_fusion.ir_sensors[3];
        float dist = motion_ctrl.traveled_distance;

/* ANSI escape codes for colors */
#define C_GREEN "\033[32m"
#define C_RED "\033[31m"
#define C_RESET "\033[0m"

        /* Left wall detection */
        if (l0 > l0_th && !seen_l0) {
          seen_l0 = 1;
          sprintf(buf, "%s[WALL] LEFT APPEARED  @ %.0fmm L0=%d%s\r\n", C_GREEN,
                  dist, l0, C_RESET);
          EXPLORE_DBG(buf);
        } else if (l0 < l0_off_th && seen_l0) {
          seen_l0 = 0;
          sprintf(buf, "%s[WALL] LEFT DISAPPEARED @ %.0fmm L0=%d%s\r\n", C_RED,
                  dist, l0, C_RESET);
          EXPLORE_DBG(buf);
        }

        /* Right wall detection */
        if (r0 > r0_th && !seen_r0) {
          seen_r0 = 1;
          sprintf(buf, "%s[WALL] RIGHT APPEARED @ %.0fmm R0=%d%s\r\n", C_GREEN,
                  dist, r0, C_RESET);
          EXPLORE_DBG(buf);
        } else if (r0 < r0_off_th && seen_r0) {
          seen_r0 = 0;
          sprintf(buf, "%s[WALL] RIGHT DISAPPEARED @ %.0fmm R0=%d%s\r\n", C_RED,
                  dist, r0, C_RESET);
          EXPLORE_DBG(buf);
        }
      }

      /* Cell-by-cell: wait for full stop, then blocking IR scan */
      if (Motion_IsComplete(&motion_ctrl)) {
        /* Read sensors at cell center (robot is stopped) */
        IR_Sensor_StartScan();
        {
          uint32_t iw = millis();
          while (!IR_Sensor_IsReady() && (millis() - iw < 20))
            ;
        }
        if (!IR_Sensor_IsReady()) {
          EXPLORE_DBG("WARN: IR not ready at cell center\r\n");
          explore_state = EXPLORE_START_CELL;
          break;
        }
        IR_Sensor_GetResults(ir_raw);

        left_wall = IR_Simple_DetectLeftWall(&ir_calib, ir_raw);
        front_wall = IR_Simple_DetectFrontWall(&ir_calib, ir_raw);
        right_wall = IR_Simple_DetectRightWall(&ir_calib, ir_raw);

        /* [FIX] Do NOT override left/right wall with front sensors.
         * Side sensors (L90/R90) are reliable for side walls even when
         * a front wall exists. Overriding caused known walls to be erased. */

        sprintf(buf, "Peek:(%d,%d) D:%d W:L%dF%dR%d\r\n", maze.robot_x,
                maze.robot_y, maze.robot_dir, left_wall, front_wall,
                right_wall);
        EXPLORE_DBG(buf);
        sprintf(buf, "  IR:%d %d %d %d %d %d\r\n", ir_raw[0], ir_raw[1],
                ir_raw[2], ir_raw[3], ir_raw[4], ir_raw[5]);
        EXPLORE_DBG(buf);

        Maze_UpdateWalls(&maze, left_wall, front_wall, right_wall);
        Maze_SendCellUpdate(&maze);
        Maze_Persistent_MarkObserved(&persistent_map, maze.robot_x,
                                     maze.robot_y, maze.robot_dir);

        if (Maze_IsAtCenter(&maze)) {
          EXPLORE_DBG("*** CENTER REACHED! ***\r\n");
          persistent_map.center_reached_count++;
          LED_OFF();
          Motion_Stop(&motion_ctrl);
          Maze_Persistent_SyncFrom(&persistent_map, &maze);
          persistent_map.total_runs++;
          Flash_SaveMaze(&persistent_map);
          Maze_Persistent_PrintStatus(&persistent_map);
          system_state = STATE_MENU;
          break;
        }

        Maze_FloodFillCenter(&maze);
        next_move = Maze_GetNextMove_Smart(&maze, &persistent_map);

#if ENABLE_BACK_ALIGN || ADVANCED_CELL_BY_CELL
        turn_ctx.left_wall_before_turn = left_wall;
        turn_ctx.front_wall_before_turn = front_wall;
        turn_ctx.right_wall_before_turn = right_wall;
#endif

#if ADVANCED_CELL_BY_CELL
        if (front_wall && next_move != 'F') {
          Motion_FrontAlign(&motion_ctrl, &ir_calib);
          motion_start_time = millis();
          EXPLORE_DBG("ALIGN: start\r\n");
          explore_state = EXPLORE_FRONT_ALIGN;
        } else {
          explore_state = EXPLORE_DECIDE_TURN;
        }
#else
        explore_state = EXPLORE_DECIDE_TURN;
#endif
      }

      break;

    case EXPLORE_DECIDE_TURN:
      sprintf(buf, "Action: %c\r\n", next_move);
      EXPLORE_DBG(buf);

      /* [FIX] Stuck detection: same action repeated too many times */
      if (next_move == last_action) {
        same_action_count++;
        if (same_action_count >= MAX_SAME_ACTION_COUNT) {
          sprintf(buf, "STUCK: Action '%c' repeated %d times!\r\n", next_move,
                  same_action_count);
          EXPLORE_DBG(buf);
          Motion_Stop(&motion_ctrl);
          Hardware_StopMotors();
          Maze_Persistent_SyncFrom(&persistent_map, &maze);
          persistent_map.total_runs++;
          system_state = STATE_MENU;
          break;
        }
      } else {
        same_action_count = 1;
        last_action = next_move;
      }

      motion_start_time = millis(); /* Reset per-motion timeout */

      if (next_move == 'F') {
        /* Cell-by-cell: pre-advance position before driving */
        maze.robot_x += dx[maze.robot_dir];
        maze.robot_y += dy[maze.robot_dir];
        Maze_Persistent_IncrementVisit(&persistent_map, maze.robot_x,
                                       maze.robot_y);
        explore_state = EXPLORE_START_CELL;
      } else if (next_move == 'L' || next_move == 'R') {
        uint8_t old_dir, new_dir;
        inplace_turn_angle = (next_move == 'L') ? 90.0f : -90.0f;

        old_dir = maze.robot_dir;
        new_dir = (next_move == 'L') ? (maze.robot_dir + 3) % 4
                                     : (maze.robot_dir + 1) % 4;

        /* [NEW] Check for back-align primitive eligibility */
#if ENABLE_BACK_ALIGN
        if (BackAlign_IsEligible(next_move, &turn_ctx)) {
          if (back_align_cooldown == 0) {
            EXPLORE_DBG("ELIGIBLE for back align!\r\n");
            BackAlign_Start(&back_align, next_move, new_dir, &motion_ctrl);

            /* Only update direction here. Position (robot_x/y) will be
             * updated later in EXPLORE_DECIDE_TURN when next_move is
             * processed after back-align completes. */
            maze.robot_dir = new_dir;

            motion_start_time = millis(); /* Reset timeout for back-align */
            explore_state = EXPLORE_BACK_ALIGN_PRIMITIVE;
            back_align_cooldown =
                back_align_skip_after_use; /* Mode-dependent cooldown */
            break;
          } else {
            back_align_cooldown--;
            EXPLORE_DBG("SKIP back align (cooldown)\r\n");
          }
        }
#endif

        /* Robot is already stopped (cell-by-cell mode) */
        {
          maze.robot_dir = new_dir;
          maze.robot_x += dx[new_dir];
          maze.robot_y += dy[new_dir];
          Maze_Persistent_IncrementVisit(&persistent_map, maze.robot_x,
                                         maze.robot_y);
          sprintf(buf, "Pivot %c: dir=%d->%d pos=(%d,%d)\r\n", next_move,
                  old_dir, new_dir, maze.robot_x, maze.robot_y);
          EXPLORE_DBG(buf);
          explore_state = EXPLORE_WAIT_INPLACE_STOP;
        }
      } else if (next_move == 'B') {
        uint8_t new_dir;
        new_dir = (maze.robot_dir + 2) % 4;

#if ADVANCED_CELL_BY_CELL
        if (BackAlign_IsEligible(next_move, &turn_ctx)) {
          if (back_align_cooldown == 0) {
            EXPLORE_DBG("ELIGIBLE for back align (B)!\r\n");
            BackAlign_Start(&back_align, next_move, new_dir, &motion_ctrl);

            maze.robot_dir = new_dir;
            motion_start_time = millis(); /* Reset timeout for back-align */
            explore_state = EXPLORE_BACK_ALIGN_PRIMITIVE;
            back_align_cooldown =
                back_align_skip_after_use; /* Mode-dependent cooldown */
            break;
          } else {
            back_align_cooldown--;
            EXPLORE_DBG("SKIP back align B (cooldown)\r\n");
          }
        }
#endif

        /* Non-blocking B-turn: split into 2 phases */
        Motion_Stop(&motion_ctrl);
        maze.robot_dir = new_dir;
        /* B means turn back and move to the previous cell. Pre-advance map
         * position exactly like F/L/R so the next START_CELL scan is written
         * to the correct cell. */
        maze.robot_x += dx[new_dir];
        maze.robot_y += dy[new_dir];
        Maze_Persistent_IncrementVisit(&persistent_map, maze.robot_x,
                                       maze.robot_y);
        explore_state = EXPLORE_WAIT_BTURN_STOP;
      } else {
        /* Unknown command - safety fallback */
        EXPLORE_DBG("WARN: Unknown move command!\r\n");
        explore_state = EXPLORE_START_CELL;
      }
      break;

    case EXPLORE_WAIT_MANEUVER:
      if (Motion_IsComplete(&motion_ctrl)) {
        /* Forward done — scan and update maze before starting next cell */
        Explore_ScanAndUpdateMaze(&maze, &ir_calib, "FWD_DONE");
        explore_state = EXPLORE_START_CELL;
      }
      break;

    /* === Wait for remaining cell distance before L/R pivot turn === */
    case EXPLORE_WAIT_FINISH_CELL:
      if (Motion_IsComplete(&motion_ctrl)) {
        uint8_t new_dir = (inplace_turn_angle > 0) ? (maze.robot_dir + 3) % 4
                                                   : (maze.robot_dir + 1) % 4;
        Motion_Stop(&motion_ctrl);
        maze.robot_dir = new_dir;
        maze.robot_x += dx[new_dir];
        maze.robot_y += dy[new_dir];
        Maze_Persistent_IncrementVisit(&persistent_map, maze.robot_x,
                                       maze.robot_y);
        motion_start_time = millis();
        explore_state = EXPLORE_WAIT_INPLACE_STOP;
      }
      break;

    /* [FIX] Non-blocking B-turn states */
    case EXPLORE_WAIT_BTURN_STOP:
      if (Motion_IsComplete(&motion_ctrl)) {
        Motion_Turn(&motion_ctrl, 180.0f);
        motion_start_time = millis();
        explore_state = EXPLORE_WAIT_BTURN_TURN;
      }
      break;

    case EXPLORE_WAIT_BTURN_TURN:
      if (Motion_IsComplete(&motion_ctrl)) {
        motion_ctrl.current_speed = motion_ctrl.base_speed;
        /* Scan + update maze FIRST after 180-turn */
        Explore_ScanAndUpdateMaze(&maze, &ir_calib, "BTURN");

        /* Then check if front wall visible for square-up */
#if ADVANCED_CELL_BY_CELL
        if (IR_Sensor_IsReady() && ir_calib.is_calibrated &&
            ir_calib.front_fsum_target > 0) {
          IR_Sensor_GetResults(post_ir);
          if ((post_ir[0] + post_ir[5]) >
              (uint16_t)(ir_calib.front_fsum_target * 0.3f)) {
            EXPLORE_DBG("POST_ALIGN: start (B)\r\n");
            Motion_FrontAlign(&motion_ctrl, &ir_calib);
            motion_start_time = millis();
            explore_state = EXPLORE_POST_TURN_ALIGN;
            break;
          }
        }
#endif
        Motion_SnapHeading();
        explore_state = EXPLORE_START_CELL;
      }
      break;

    /* === In-place 90° turn states === */
    case EXPLORE_WAIT_INPLACE_STOP:
      if (Motion_IsComplete(&motion_ctrl)) {
        /* Explore modes keep classic in-place turn behavior. */
        Motion_Turn(&motion_ctrl, inplace_turn_angle);
        motion_start_time = millis();
        explore_state = EXPLORE_WAIT_INPLACE_TURN;
      }
      break;

    case EXPLORE_WAIT_INPLACE_TURN:
      if (Motion_IsComplete(&motion_ctrl)) {
        motion_ctrl.current_speed = motion_ctrl.base_speed;
        /* Read sensors after turn to update maze with new orientation */
        {
          uint16_t turn_ir[6];
          uint8_t tl, tf, tr;
          IR_Sensor_StartScan();
          {
            uint32_t iw = millis();
            while (!IR_Sensor_IsReady() && (millis() - iw < 20))
              ;
          }
          if (IR_Sensor_IsReady()) {
            IR_Sensor_GetResults(turn_ir);
            tl = IR_Simple_DetectLeftWall(&ir_calib, turn_ir);
            tf = IR_Simple_DetectFrontWall(&ir_calib, turn_ir);
            tr = IR_Simple_DetectRightWall(&ir_calib, turn_ir);
            /* maze.robot_dir already updated to new direction */
            Maze_AddWalls(&maze, tl, tf, tr);
            Maze_SendCellUpdate(&maze);

            /* Use same scan data for front wall alignment check */
#if ADVANCED_CELL_BY_CELL
            if (tf && ir_calib.is_calibrated &&
                ir_calib.front_fsum_target > 0 &&
                (turn_ir[0] + turn_ir[5]) >
                    (uint16_t)(ir_calib.front_fsum_target * 0.3f)) {
              EXPLORE_DBG("POST_ALIGN: start\r\n");
              Motion_FrontAlign(&motion_ctrl, &ir_calib);
              motion_start_time = millis();
              explore_state = EXPLORE_POST_TURN_ALIGN;
              break;
            }
#endif
          }
        }

        /* No front wall: just snap heading and start next cell */
        Motion_SnapHeading();
        explore_state = EXPLORE_START_CELL;
      }
      break;

    case EXPLORE_FRONT_ALIGN:
      /* Motion_Update in TIM11 ISR handles the PID loop */
      if (Motion_IsComplete(&motion_ctrl)) {
        if (motion_ctrl.align.result == 1) {
          sprintf(buf, "ALIGN: OK\r\n");
        } else if (motion_ctrl.align.result == 2) {
          sprintf(buf, "ALIGN: timeout\r\n");
        } else {
          sprintf(buf, "ALIGN: lost wall\r\n");
        }
        EXPLORE_DBG(buf);
        /* Snap heading to nearest 90-degree grid line */
        if (motion_ctrl.align.result == 1) {
          Motion_SnapHeading();
        }

        /* Read sensors after align to update maze (robot is stationary) */
        {
          uint16_t align_ir[6];
          uint8_t al, af, ar;
          IR_Sensor_StartScan();
          {
            uint32_t iw = millis();
            while (!IR_Sensor_IsReady() && (millis() - iw < 20))
              ;
          }
          if (IR_Sensor_IsReady()) {
            IR_Sensor_GetResults(align_ir);
            al = (align_ir[2] > ir_calib.wall_threshold_front_left) ? 1 : 0;
            af = IR_Simple_DetectFrontWall(&ir_calib, align_ir);
            ar = (align_ir[3] > ir_calib.wall_threshold_front_right) ? 1 : 0;
            Maze_AddWalls(&maze, al, af, ar);
            Maze_SendCellUpdate(&maze);
            sprintf(buf, "POST_ALIGN_SENSE:(%d,%d) D:%d W:L%dF%dR%d\r\n",
                    maze.robot_x, maze.robot_y, maze.robot_dir, al, af, ar);
            EXPLORE_DBG(buf);

            /* Re-evaluate next_move with updated wall data */
            Maze_FloodFillCenter(&maze);
            next_move = Maze_GetNextMove_Smart(&maze, &persistent_map);
            sprintf(buf, "RE-DECIDE: %c\r\n", next_move);
            EXPLORE_DBG(buf);
          }
        }

        /* Front-align done → go to DECIDE_TURN where back-align
         * eligibility will be evaluated */
        stability_next_state = EXPLORE_DECIDE_TURN;
        stability_start_time = millis();
        explore_state = EXPLORE_STABILITY_WAIT;
      }
      break;

    /* === Front wall alignment state (POST-TURN) === */
    case EXPLORE_POST_TURN_ALIGN:
      if (Motion_IsComplete(&motion_ctrl)) {
        if (motion_ctrl.align.result == 1) {
          EXPLORE_DBG("POST_ALIGN: OK\r\n");
          Motion_SnapHeading();
        } else if (motion_ctrl.align.result == 2) {
          EXPLORE_DBG("POST_ALIGN: timeout\r\n");
          Motion_SnapHeading();
        } else {
          EXPLORE_DBG("POST_ALIGN: lost wall\r\n");
        }

        /* Scan after post-turn align */
        Explore_ScanAndUpdateMaze(&maze, &ir_calib, "PTA");

        /* Post-turn align done → start next cell */
        stability_next_state = EXPLORE_START_CELL;
        stability_start_time = millis();
        explore_state = EXPLORE_STABILITY_WAIT;
      }
      break;

    /* === Stability delay after alignment === */
    case EXPLORE_STABILITY_WAIT:
      if (millis() - stability_start_time >= ALIGN_STABILITY_DELAY_MS) {
        explore_state = stability_next_state;
      }
      break;

    case EXPLORE_BACK_ALIGN_PRIMITIVE:
      BackAlign_Update(&back_align, &motion_ctrl);
      if (BackAlign_IsDone(&back_align)) {
        EXPLORE_DBG("Back-align primitive complete.\r\n");

        /* Read sensors after back-align to update maze with new orientation */
        {
          uint16_t ba_ir[6];
          uint8_t bl, bf, br;
          IR_Sensor_StartScan();
          {
            uint32_t iw = millis();
            while (!IR_Sensor_IsReady() && (millis() - iw < 20))
              ;
          }
          if (IR_Sensor_IsReady()) {
            IR_Sensor_GetResults(ba_ir);
            bl = IR_Simple_DetectLeftWall(&ir_calib, ba_ir);
            bf = IR_Simple_DetectFrontWall(&ir_calib, ba_ir);
            br = IR_Simple_DetectRightWall(&ir_calib, ba_ir);
            Maze_AddWalls(&maze, bl, bf, br);
            Maze_SendCellUpdate(&maze);
            Maze_FloodFillCenter(&maze);
            next_move = Maze_GetNextMove_Smart(&maze, &persistent_map);
            sprintf(buf, "POST_BA:(%d,%d) D:%d W:L%dF%dR%d NM:%c\r\n",
                    maze.robot_x, maze.robot_y, maze.robot_dir, bl, bf, br,
                    next_move);
            EXPLORE_DBG(buf);

            /* If front wall detected after back-align, align first */
            if (bf && ir_calib.is_calibrated &&
                ir_calib.front_fsum_target > 0 &&
                (ba_ir[0] + ba_ir[5]) >
                    (uint16_t)(ir_calib.front_fsum_target * 0.3f)) {
              EXPLORE_DBG("POST_BA_ALIGN: start\r\n");
              Motion_FrontAlign(&motion_ctrl, &ir_calib);
              motion_start_time = millis();
              explore_state = EXPLORE_FRONT_ALIGN;
              break;
            }
          }
        }
        /* [CRITICAL FIX] Advance position when going forward after back-align.
         * Without this, robot physically moves to next cell but maze position
         * stays at the old cell, causing all subsequent navigation to be wrong.
         */
        if (next_move == 'F') {
          maze.robot_x += dx[maze.robot_dir];
          maze.robot_y += dy[maze.robot_dir];
          Maze_Persistent_IncrementVisit(&persistent_map, maze.robot_x,
                                         maze.robot_y);
        }
        explore_state = EXPLORE_START_CELL;
      }
      break;
    }

#if EXPLORE_OLED_DEBUG
    if (Motion_IsComplete(&motion_ctrl) && last_display_time != 0xFFFFFFFF) {
      last_display_time = 0xFFFFFFFF; /* Mark as drawn, reset in START_CELL */
      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("EXPLORING", &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 20);
      sprintf(buf, "Pos:(%d,%d)", maze.robot_x, maze.robot_y);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 35);
      sprintf(buf, "Move:%c", next_move);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 50);
      SSD1306_WriteString("Press=Exit", &Font_7x10, SSD1306_WHITE);

      /* [CRITICAL FIX] Use DMA (non-blocking) instead of blocking
       * SSD1306_UpdateScreen() which causes I2C deadlock with TIM11 ISR */
      SSD1306_UpdateScreen_DMA();
    }
#endif

    if (Hardware_ReadButton(0)) {
      Motion_Stop(&motion_ctrl);
      Motion_StopPID(); /* Stop the 1ms Interrupt */
      Hardware_StopMotors();
      /* Sync walls back even on manual exit */
      Maze_Persistent_SyncFrom(&persistent_map, &maze);
      persistent_map.total_runs++;
      Flash_SaveMaze(&persistent_map);
      Maze_Persistent_PrintStatus(&persistent_map);
      system_state = STATE_MENU;
      /* Small spin delay just to debounce button on exit */
      delay_ms_blocking(500);
      break;
    }
  }

  LED_OFF();
  Motion_StopPID(); /* Ensure it is stopped on exit */
  UART_SendString("Exploration ended\r\n");
  Maze_PrintCompact(&maze);
}

/* ========================================================================== */
/* RUN FASTER — Cell-by-cell with higher speed                                */
/* ========================================================================== */

void Execute_MazeExploreContinuous(void) { Execute_MazeExplore(1); }

/* ========================================================================== */
/* A* OPTIMAL RUN MODE                                                        */
/* ========================================================================== */

void Execute_AStarRun(void) {
  typedef enum {
    ASTAR_INIT,
    ASTAR_MOVING,
    ASTAR_DECIDE,
    ASTAR_WAIT_MANEUVER,
    ASTAR_WAIT_BTURN_STOP,
    ASTAR_WAIT_BTURN_TURN,
    ASTAR_WAIT_INPLACE_STOP,
    ASTAR_WAIT_INPLACE_TURN,
    ASTAR_SMOOTH_ENTERING, /* Entering intermediate cell before smooth turn */
    ASTAR_APPROACH_PIVOT,  /* Driving extra offset after post detect, before
                              pivot */
    ASTAR_WAIT_SMOOTH_TURN /* Waiting for smooth turn arc to complete */
  } AStar_State_t;

  AStar_State_t astar_state;
  char buf[128];
  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {1, 0, -1, 0};
  uint8_t next_move;
  float astar_inplace_angle;
  uint32_t last_display_time = 0;
  uint32_t motion_start_time;
  uint32_t astar_last_progress_time;
  float astar_last_progress_dist;

  /* Smooth turn tracking variables for A* mode */
  float astar_smooth_angle = 0.0f;
  int8_t astar_smooth_final_x = 0, astar_smooth_final_y = 0;
  uint8_t astar_smooth_new_dir = 0;

  /* Chain turn variables (U-turn / S-curve) */
  uint8_t astar_chain_pending = 0;  /* 1 if second smooth turn queued */
  float astar_chain_angle = 0.0f;   /* Angle for second turn */
  uint8_t astar_chain_new_dir = 0;  /* Direction after second turn */
  int8_t astar_chain_final_x = 0, astar_chain_final_y = 0;

  /* [ITEM 5] Peak-based post detection for A* straights */
  uint16_t astar_prev_l45 = 0, astar_prev_r45 = 0;
  uint16_t astar_peak_l45 = 0, astar_peak_r45 = 0;
  uint8_t astar_trend_l45 = TREND_FLAT, astar_trend_r45 = TREND_FLAT;
  uint8_t astar_fall_l45 = 0, astar_fall_r45 = 0;
  uint8_t astar_rise_l45 = 0, astar_rise_r45 = 0;
  uint8_t astar_peak_armed = 0;
  uint8_t astar_peak_skip = 0;
  uint8_t astar_is_straight = 0;
  uint8_t astar_diag_seen = 0;
  uint8_t astar_side0_seen = 0;
  uint8_t astar_l45_lost = 0;
  uint8_t astar_l0_lost = 0;
  uint16_t astar_diag_peak = 0;
  float astar_l45_loss_dist = 0.0f;
  float astar_turn_target_dist = 0.0f;
  uint8_t astar_t1w_turn_used = 0;
  static float astar_estimated_k = 60.0f;
  uint16_t astar_l0_th = 150, astar_r0_th = 150;
  uint16_t astar_l0_off_th = 60, astar_r0_off_th = 60;
  uint32_t astar_t1w_dbg_ms = 0;

  UART_SendString("\r\n=== A* OPTIMAL RUN MODE ===\r\n");

  /* Check if persistent map is ready */
  if (!Maze_Persistent_IsReadyForAStar(&persistent_map)) {
    UART_SendString("ERROR: Need at least 1 successful explore first!\r\n");
    Display_Status("A* RUN", "Need explore!");
    delay_ms_blocking(500);
    system_state = STATE_MENU;
    return;
  }

  Maze_Persistent_PrintStatus(&persistent_map);

  Display_Running("A* RUN");

  /* ===== MPU6050: Use flash offset or fallback calibrate ===== */
  if (mpu_calib_flash.is_valid == MPU_CALIB_MAGIC) {
    sensor_fusion.mpu_handle.gyro_z_offset = mpu_calib_flash.gyro_z_offset;
    UART_SendString("MPU: using flash offset (A*)\r\n");
  } else {
    Display_Status("MPU CALIB", "Keep still!");
    UART_SendString("No flash MPU calib, running live...\r\n");
    delay_ms_blocking(1000);
    MPU6050_Calibrate(&sensor_fusion.mpu_handle);
  }

  SensorFusion_ResetOdometry(&sensor_fusion);
  sensor_fusion.gyro_z = 0.0f;
  MPU6050_ResetYaw(&sensor_fusion.mpu_handle);
  Motion_Stop(&motion_ctrl);
  motion_ctrl.current_speed = 0.0f;

  /* Hand start: wait for L90 hand detection + 2s countdown */
  WaitForHandStart();
  if (system_state == STATE_MENU)
    return;

  /* [ITEM 1] Use optimal speed profile for A* (faster than default) */
  Motion_SetSpeedProfile_Optimal(&motion_ctrl);
  motion_ctrl.stabilize_duration = 250; /* A*: keep longer stabilize */
  motion_ctrl.max_speed = 70.0f;        /* A*: 70% PWM cruise */
  motion_ctrl.speed_pid_enable = 1;     /* A*: use closed-loop speed PID */
  motion_ctrl.accel_mmps2 = 2500.0f;    /* A*: fast ramp (was 800) */
  motion_ctrl.speed_ff_gain =
      0.18f; /* A*: keep original (matches encoder noise) */
  motion_ctrl.max_speed_mmps =
      300.0f; /* A*: limit cruise speed to avoid overshoot */
  motion_ctrl.desired_speed = 0.0f; /* First segment ramps from standstill */
  /* [ITEM 3] Copy IR calib so wall-following PID has setpoints */
  sensor_fusion.ir_calib = ir_calib;

  if (ir_calib.is_calibrated) {
    astar_l0_th = ir_calib.wall_threshold_front_left;
    astar_r0_th = ir_calib.wall_threshold_front_right;
  }
  astar_l0_off_th = (uint16_t)(astar_l0_th * 0.40f);
  astar_r0_off_th = (uint16_t)(astar_r0_th * 0.40f);

  /* Load persistent map into working maze */
  Maze_Init(&maze);
  Maze_Persistent_LoadInto(&persistent_map, &maze);

  /* Calculate A* optimal path */
  if (!Maze_CalculateOptimalPath(&maze, &astar_result)) {
    UART_SendString("A* path calculation FAILED!\r\n");
    Display_Status("A* RUN", "Path failed!");
    delay_ms_blocking(1000);
    system_state = STATE_MENU;
    return;
  }

  sprintf(buf, "A* path ready: %d actions\r\n",
          astar_result.optimal_path_length);
  UART_SendString(buf);

  Hardware_ResetEncoder(0);
  Hardware_ResetEncoder(1);

  Motion_StartPID();
  LED_ON();

  astar_state = ASTAR_INIT;
  next_move = 0;
  motion_start_time = millis();
  astar_last_progress_time = motion_start_time;
  astar_last_progress_dist = motion_ctrl.traveled_distance;

  while (system_state == STATE_RUNNING) {
    SSD1306_Process_DMA();

    /* A* watchdog:
     * - Straight segments: timeout by LACK OF ENCODER PROGRESS (stall),
     *   not by absolute segment time.
     * - Keep a wide hard timeout only as last-resort safety. */
    if (!Motion_IsComplete(&motion_ctrl)) {
      float dist_delta =
          motion_ctrl.traveled_distance - astar_last_progress_dist;
      if (dist_delta < 0.0f)
        dist_delta = -dist_delta;

      if (dist_delta > 3.0f) {
        astar_last_progress_dist = motion_ctrl.traveled_distance;
        astar_last_progress_time = millis();
      }

      if ((motion_ctrl.state == MOTION_STRAIGHT ||
           motion_ctrl.state == MOTION_STRAIGHT_CONSTANT) &&
          (millis() - astar_last_progress_time) > ASTAR_STALL_TIMEOUT_MS) {
        sprintf(buf, "WARN: A* stall timeout st=%d d=%ld/%ld\r\n",
                (int)astar_state, (long)motion_ctrl.traveled_distance,
                (long)motion_ctrl.target_distance);
        UART_SendString(buf);
        Motion_Stop(&motion_ctrl);
        system_state = STATE_MENU;
        break;
      }

      if ((millis() - motion_start_time) > ASTAR_HARD_TIMEOUT_MS) {
        sprintf(buf, "WARN: A* hard timeout st=%d d=%ld/%ld\r\n",
                (int)astar_state, (long)motion_ctrl.traveled_distance,
                (long)motion_ctrl.target_distance);
        UART_SendString(buf);
        Motion_Stop(&motion_ctrl);
        system_state = STATE_MENU;
        break;
      }
    }

    switch (astar_state) {
    case ASTAR_INIT:
      next_move = Maze_GetNextMove_Optimal(&maze, &astar_result);
      if (next_move == 0) {
        /* Path complete at start? Should not happen */
        UART_SendString("A* path empty!\r\n");
        system_state = STATE_MENU;
        break;
      }
      astar_state = ASTAR_DECIDE;
      break;

    case ASTAR_DECIDE:
      sprintf(buf, "A* Action[%d/%d]: %c at (%d,%d)\r\n",
              astar_result.optimal_path_index, astar_result.optimal_path_length,
              next_move, maze.robot_x, maze.robot_y);
      UART_SendString(buf);

      if (next_move == 'F') {
        /* === SMOOTH TURN DETECTION ===
         * Look ahead for F + L/R + F pattern in A* path.
         * Also detect chain patterns (U-turn / S-curve):
         *   F + L/R + F + L/R + F  (5 commands)
         * Current next_move='F' is the first F (already consumed). */
        uint8_t idx = astar_result.optimal_path_index;
        uint8_t peek1 = 0, peek2 = 0, peek3 = 0, peek4 = 0;
        uint8_t smooth_ok = 0;

        if (idx < astar_result.optimal_path_length)
          peek1 = astar_result.optimal_path[idx].command;
        if (idx + 1 < astar_result.optimal_path_length)
          peek2 = astar_result.optimal_path[idx + 1].command;
        if (idx + 2 < astar_result.optimal_path_length)
          peek3 = astar_result.optimal_path[idx + 2].command;
        if (idx + 3 < astar_result.optimal_path_length)
          peek4 = astar_result.optimal_path[idx + 3].command;

        if ((peek1 == 'L' || peek1 == 'R') && peek2 == 'F') {
          uint8_t old_dir = maze.robot_dir;
          uint8_t new_dir;
          int8_t inter_x, inter_y, final_x, final_y;

          if (peek1 == 'L') {
            new_dir = (old_dir + 3) % 4;
          } else {
            new_dir = (old_dir + 1) % 4;
          }

          /* Intermediate cell = 1 step forward in old direction */
          inter_x = maze.robot_x + dx[old_dir];
          inter_y = maze.robot_y + dy[old_dir];
          /* Final cell = 1 step in new direction from intermediate */
          final_x = inter_x + dx[new_dir];
          final_y = inter_y + dy[new_dir];

          /* Check feasibility:
           * 1) No wall ahead from current cell (can enter intermediate)
           * 2) Intermediate cell in bounds
           * 3) No wall from intermediate cell in new direction
           * 4) Final cell in bounds */
          if (inter_x >= 0 && inter_x < MAZE_SIZE && inter_y >= 0 &&
              inter_y < MAZE_SIZE && final_x >= 0 && final_x < MAZE_SIZE &&
              final_y >= 0 && final_y < MAZE_SIZE &&
              !(maze.walls[maze.robot_y][maze.robot_x] & (1 << old_dir)) &&
              !(maze.walls[inter_y][inter_x] & (1 << new_dir))) {
            smooth_ok = 1;
          }

          if (smooth_ok) {
            /* Consume L/R + F from A* path (2 commands) */
            astar_result.optimal_path_index += 2;

            /* Save smooth turn parameters */
            astar_smooth_angle = (peek1 == 'L') ? 90.0f : -90.0f;
            astar_smooth_new_dir = new_dir;
            astar_smooth_final_x = final_x;
            astar_smooth_final_y = final_y;

            /* === CHAIN DETECTION (U-turn / S-curve) ===
             * Check if peek3+peek4 form another L/R+F after the first turn.
             * U-turn: peek3 == peek1 (same direction)
             * S-curve: peek3 != peek1 (opposite direction) */
            astar_chain_pending = 0;
            if ((peek3 == 'L' || peek3 == 'R') && peek4 == 'F') {
              uint8_t chain_dir;
              int8_t chain_inter_x, chain_inter_y;
              int8_t chain_fx, chain_fy;

              if (peek3 == 'L')
                chain_dir = (new_dir + 3) % 4;
              else
                chain_dir = (new_dir + 1) % 4;

              /* Chain intermediate = final of first turn */
              chain_inter_x = final_x;
              chain_inter_y = final_y;
              /* Chain final = 1 step from chain_inter in chain_dir */
              chain_fx = chain_inter_x + dx[chain_dir];
              chain_fy = chain_inter_y + dy[chain_dir];

              /* Feasibility: no wall + in bounds */
              if (chain_fx >= 0 && chain_fx < MAZE_SIZE &&
                  chain_fy >= 0 && chain_fy < MAZE_SIZE &&
                  !(maze.walls[final_y][final_x] & (1 << new_dir)) &&
                  !(maze.walls[chain_inter_y][chain_inter_x] &
                    (1 << chain_dir))) {
                /* Consume 2 more commands (L/R + F) */
                astar_result.optimal_path_index += 2;
                astar_chain_pending = 1;
                astar_chain_angle = (peek3 == 'L') ? 90.0f : -90.0f;
                astar_chain_new_dir = chain_dir;
                astar_chain_final_x = chain_fx;
                astar_chain_final_y = chain_fy;

                sprintf(buf, "A* chain %c: ->(%d,%d)->(%d,%d)\r\n",
                        (peek3 == peek1) ? 'U' : 'S', chain_inter_x,
                        chain_inter_y, chain_fx, chain_fy);
                UART_SendString(buf);
              }
            }

            /* Start forward motion into intermediate cell.
             * Pivot turn will be triggered by peak detection (IR diagonal). */
            Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);

            /* Initialize peak detection for pivot post sensing */
            {
              uint16_t init_val;
              uint16_t init_side0;
              if (astar_smooth_angle > 0)
                init_val = (uint16_t)sensor_fusion.ir_sensors[1]; /* L45 */
              else
                init_val = (uint16_t)sensor_fusion.ir_sensors[4]; /* R45 */
              if (astar_smooth_angle > 0)
                init_side0 = (uint16_t)sensor_fusion.ir_sensors[2]; /* L0 */
              else
                init_side0 = (uint16_t)sensor_fusion.ir_sensors[3]; /* R0 */
              astar_prev_l45 = init_val;
              astar_peak_l45 = init_val;
              astar_trend_l45 = TREND_FLAT;
              astar_fall_l45 = 0;
              astar_rise_l45 = 0;
              astar_peak_armed = 0;
              astar_diag_seen = (init_val > ASTAR_T1W_PRESENT_TH) ? 1 : 0;
              astar_diag_peak = init_val;
              astar_side0_seen = 0;
              astar_l45_lost = 0;
              astar_l0_lost = 0;
              astar_l45_loss_dist = 0.0f;
              astar_turn_target_dist = 0.0f;
              astar_t1w_turn_used = 0;
              astar_t1w_dbg_ms = 0;
              sprintf(buf, "A*T1W init d=%ld D=%d S=%d th=%d/%d\r\n",
                      (long)motion_ctrl.traveled_distance, init_val, init_side0,
                      ASTAR_T1W_PRESENT_TH,
                      (astar_smooth_angle > 0) ? astar_r0_th : astar_l0_th);
              UART_SendString(buf);
            }

            /* Update position to intermediate cell (entering it) */
            maze.robot_x = inter_x;
            maze.robot_y = inter_y;

            sprintf(buf, "Pivot %c: (%d,%d)->(%d,%d)->(%d,%d)\r\n", peek1,
                    (int)(inter_x - dx[old_dir]), (int)(inter_y - dy[old_dir]),
                    inter_x, inter_y, final_x, final_y);
            UART_SendString(buf);

            motion_start_time = millis();
            astar_state = ASTAR_SMOOTH_ENTERING;
            break;
          }
        }

        /* [ITEM 2] Multi-cell straight merge: count consecutive F commands
         * and merge them into one long Motion_Straight() with decel ramp.
         * Stop merging when:
         *   - next command is not F (L/R/B)
         *   - next F leads to smooth turn (F + L/R + F pattern)
         *   - end of path */
        {
          uint8_t merge_count = 1; /* Current F already counts as 1 */
          uint8_t midx = astar_result.optimal_path_index;

          while (midx < astar_result.optimal_path_length) {
            uint8_t mcmd = astar_result.optimal_path[midx].command;
            if (mcmd != 'F')
              break; /* Not F, stop merging */

            /* Check if this F leads to a smooth turn entry */
            {
              uint8_t mn1 = 0, mn2 = 0;
              if (midx + 1 < astar_result.optimal_path_length)
                mn1 = astar_result.optimal_path[midx + 1].command;
              if (midx + 2 < astar_result.optimal_path_length)
                mn2 = astar_result.optimal_path[midx + 2].command;
              if ((mn1 == 'L' || mn1 == 'R') && mn2 == 'F')
                break; /* Leave this F for smooth turn processing */
            }

            merge_count++;
            midx++;
          }

          /* Consume extra F commands from path */
          astar_result.optimal_path_index += (merge_count - 1);

          /* Update robot position for all merged cells */
          {
            uint8_t mi;
            for (mi = 0; mi < merge_count; mi++) {
              maze.robot_x += dx[maze.robot_dir];
              maze.robot_y += dy[maze.robot_dir];
            }
          }

          if (merge_count > 1) {
            /* Multi-cell: use Motion_Straight (with decel ramp) */
            Motion_Straight(&motion_ctrl, merge_count * CELL_SIZE_MM);
            sprintf(buf, "MultiCell: %d cells (%.0fmm)\r\n", merge_count,
                    merge_count * CELL_SIZE_MM);
            UART_SendString(buf);
          } else {
            /* Single cell: use StraightConstant (no decel, keep momentum) */
            Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);
          }
          motion_start_time = millis();
          astar_state = ASTAR_WAIT_MANEUVER;
          /* [ITEM 5] Mark as straight and init peak detector */
          astar_is_straight = 1;
          astar_prev_l45 = (uint16_t)sensor_fusion.ir_sensors[1];
          astar_prev_r45 = (uint16_t)sensor_fusion.ir_sensors[4];
          astar_peak_l45 = 0;
          astar_peak_r45 = 0;
          astar_trend_l45 = TREND_FLAT;
          astar_trend_r45 = TREND_FLAT;
          astar_fall_l45 = 0;
          astar_fall_r45 = 0;
          astar_rise_l45 = 0;
          astar_rise_r45 = 0;
          astar_peak_armed = 0;
          astar_peak_skip = 0;
        }
      } else if (next_move == 'L') {
        /* In-place turn left (fallback when smooth not possible) */
        Motion_Stop(&motion_ctrl);
        maze.robot_dir = (maze.robot_dir + 3) % 4;
        astar_inplace_angle = 90.0f;
        astar_state = ASTAR_WAIT_INPLACE_STOP;
        break;
      } else if (next_move == 'R') {
        /* In-place turn right (fallback when smooth not possible) */
        Motion_Stop(&motion_ctrl);
        maze.robot_dir = (maze.robot_dir + 1) % 4;
        astar_inplace_angle = -90.0f;
        astar_state = ASTAR_WAIT_INPLACE_STOP;
        break;
      } else if (next_move == 'B') {
        /* Non-blocking B-turn */
        Motion_Stop(&motion_ctrl);
        maze.robot_dir = (maze.robot_dir + 2) % 4;
        astar_state = ASTAR_WAIT_BTURN_STOP;
        break;
      }

      break;

    case ASTAR_WAIT_MANEUVER:
      /* [ITEM 5] Peak-based distance correction during straight motion */
      if (astar_is_straight && ir_calib.is_calibrated) {
        uint16_t al45 = (uint16_t)sensor_fusion.ir_sensors[1];
        uint16_t ar45 = (uint16_t)sensor_fusion.ir_sensors[4];
        int16_t adl = (int16_t)al45 - (int16_t)astar_prev_l45;
        int16_t adr = (int16_t)ar45 - (int16_t)astar_prev_r45;
        uint8_t apk = 0;

        astar_peak_skip++;
        if (astar_peak_skip < 3) {
          astar_prev_l45 = al45;
          astar_prev_r45 = ar45;
        } else {
          astar_peak_armed = 1;

          /* L45 slope */
          if (adl > PEAK_SLOPE_THRESH) {
            astar_trend_l45 = TREND_RISING;
            astar_rise_l45++;
            astar_fall_l45 = 0;
            if (al45 > astar_peak_l45)
              astar_peak_l45 = al45;
          } else if (adl < -PEAK_SLOPE_THRESH) {
            astar_fall_l45++;
            astar_rise_l45 = 0;
            if (astar_trend_l45 == TREND_RISING &&
                astar_peak_l45 > PEAK_MIN_VALUE &&
                astar_fall_l45 >= PEAK_CONFIRM)
              apk = 1;
            astar_trend_l45 = TREND_FALLING;
          }
          /* R45 slope */
          if (adr > PEAK_SLOPE_THRESH) {
            astar_trend_r45 = TREND_RISING;
            astar_rise_r45++;
            astar_fall_r45 = 0;
            if (ar45 > astar_peak_r45)
              astar_peak_r45 = ar45;
          } else if (adr < -PEAK_SLOPE_THRESH) {
            astar_fall_r45++;
            astar_rise_r45 = 0;
            if (astar_trend_r45 == TREND_RISING &&
                astar_peak_r45 > PEAK_MIN_VALUE &&
                astar_fall_r45 >= PEAK_CONFIRM)
              apk = 1;
            astar_trend_r45 = TREND_FALLING;
          }
          /* Sustained drop */
          if (!apk && astar_peak_armed) {
            if (astar_trend_l45 == TREND_FALLING &&
                astar_fall_l45 >= PEAK_CONFIRM &&
                astar_peak_l45 > PEAK_MIN_VALUE)
              apk = 1;
            if (astar_trend_r45 == TREND_FALLING &&
                astar_fall_r45 >= PEAK_CONFIRM &&
                astar_peak_r45 > PEAK_MIN_VALUE)
              apk = 1;
          }

          if (apk &&
              motion_ctrl.traveled_distance > ASTAR_PEAK_CORRECT_MIN_DIST_MM) {
            /* Snap distance to nearest cell boundary */
            float dist = motion_ctrl.traveled_distance;
            float nearest_boundary =
                ((int)(dist / CELL_SIZE_MM + 0.5f)) * CELL_SIZE_MM;
            float corrected_remaining =
                motion_ctrl.target_distance - nearest_boundary;

            if (corrected_remaining > 10.0f) {
              sprintf(buf, "A*PEAK: %ld->%ld rem=%ld\r\n", (long)dist,
                      (long)nearest_boundary, (long)corrected_remaining);
              UART_SendString(buf);
              Hardware_ResetEncoder(0);
              Hardware_ResetEncoder(1);
              motion_ctrl.traveled_distance = 0.0f;
              motion_ctrl.prev_traveled_distance = 0.0f;
              motion_ctrl.actual_speed = 0.0f;
              motion_ctrl.target_distance = corrected_remaining;
              motion_ctrl.start_yaw = MPU6050_GetYaw(&sensor_fusion.mpu_handle);
            }
            /* Reset peak tracker for next edge */
            astar_peak_l45 = 0;
            astar_peak_r45 = 0;
            astar_trend_l45 = TREND_FLAT;
            astar_trend_r45 = TREND_FLAT;
            astar_fall_l45 = 0;
            astar_fall_r45 = 0;
            astar_rise_l45 = 0;
            astar_rise_r45 = 0;
          }
          astar_prev_l45 = al45;
          astar_prev_r45 = ar45;
        }
      }

      if (Motion_IsComplete(&motion_ctrl)) {
        astar_is_straight = 0; /* No longer in straight segment */
        /* Check if we reached the target */
        if (Maze_IsAtCenter(&maze)) {
          UART_SendString("*** A* RUN: CENTER REACHED! ***\r\n");
          Motion_Stop(&motion_ctrl);
          LED_OFF();
          system_state = STATE_MENU;
          break;
        }

        /* Get next action from A* path */
        next_move = Maze_GetNextMove_Optimal(&maze, &astar_result);
        if (next_move == 0) {
          UART_SendString("A* path exhausted\r\n");
          Motion_Stop(&motion_ctrl);
          system_state = STATE_MENU;
          break;
        }
        astar_state = ASTAR_DECIDE;
      }
      break;

    case ASTAR_MOVING:
      /* Reserved for future speed-profile straight runs */
      break;

    /* [FIX] Non-blocking B-turn states */
    case ASTAR_WAIT_BTURN_STOP:
      if (Motion_IsComplete(&motion_ctrl)) {
        Motion_Turn(&motion_ctrl, 180.0f);
        motion_start_time = millis();
        astar_state = ASTAR_WAIT_BTURN_TURN;
      }
      break;

    case ASTAR_WAIT_BTURN_TURN:
      if (Motion_IsComplete(&motion_ctrl)) {
        motion_ctrl.current_speed = motion_ctrl.base_speed;
        if (motion_ctrl.speed_ff_gain > 0.001f) {
          motion_ctrl.desired_speed =
              motion_ctrl.base_speed / motion_ctrl.speed_ff_gain;
        }
        /* 180 turn done, get next action */
        next_move = Maze_GetNextMove_Optimal(&maze, &astar_result);
        if (next_move == 0) {
          UART_SendString("A* path exhausted after B-turn\r\n");
          Motion_Stop(&motion_ctrl);
          system_state = STATE_MENU;
          break;
        }
        astar_state = ASTAR_DECIDE;
      }
      break;

    /* === In-place 90° turn states (A* L/R) === */
    case ASTAR_WAIT_INPLACE_STOP:
      if (Motion_IsComplete(&motion_ctrl)) {
        Motion_Turn(&motion_ctrl, astar_inplace_angle);
        motion_start_time = millis();
        astar_state = ASTAR_WAIT_INPLACE_TURN;
      }
      break;

    case ASTAR_WAIT_INPLACE_TURN:
      if (Motion_IsComplete(&motion_ctrl)) {
        motion_ctrl.current_speed = motion_ctrl.base_speed;
        if (motion_ctrl.speed_ff_gain > 0.001f) {
          motion_ctrl.desired_speed =
              motion_ctrl.base_speed / motion_ctrl.speed_ff_gain;
        }
        /* In-place turn done, get next A* action (should be F) */
        next_move = Maze_GetNextMove_Optimal(&maze, &astar_result);
        if (next_move == 0) {
          UART_SendString("A* path exhausted after turn\r\n");
          Motion_Stop(&motion_ctrl);
          system_state = STATE_MENU;
          break;
        }
        astar_state = ASTAR_DECIDE;
      }
      break;

    /* === Pivot turn states (A* mode only) === */
    case ASTAR_SMOOTH_ENTERING:
      /* T1WT logic in A*:
       * 1) Detect diagonal wall loss (L45/R45)
       * 2) Verify front 0-deg sensor was seen, then wait for its loss
       * 3) At 0-deg loss (cell boundary), advance extra k/3 then turn */
      {
        uint16_t l45_val = (uint16_t)sensor_fusion.ir_sensors[1];
        uint16_t r45_val = (uint16_t)sensor_fusion.ir_sensors[4];
        uint16_t l0_val = (uint16_t)sensor_fusion.ir_sensors[2];
        uint16_t r0_val = (uint16_t)sensor_fusion.ir_sensors[3];
        uint16_t diag_val;
        uint16_t side0_val;
        uint16_t side0_off_th;
        uint16_t side0_th;
        float k_now;
        uint16_t diag_loss_dyn_th;

        if (astar_smooth_angle > 0) {
          diag_val = l45_val; /* L45 */
          side0_val = l0_val; /* L0 */
          side0_off_th = astar_l0_off_th;
          side0_th = astar_l0_th;
        } else {
          diag_val = r45_val; /* R45 */
          side0_val = r0_val; /* R0 */
          side0_off_th = astar_r0_off_th;
          side0_th = astar_r0_th;
        }

        if (!astar_diag_seen && diag_val > ASTAR_T1W_PRESENT_TH) {
          astar_diag_seen = 1;
          sprintf(buf, "A*T1W diag_seen d=%ld v=%d\r\n",
                  (long)motion_ctrl.traveled_distance, diag_val);
          UART_SendString(buf);
        }

        if (diag_val > astar_diag_peak) {
          astar_diag_peak = diag_val;
        }

        diag_loss_dyn_th =
            (uint16_t)((astar_diag_peak * ASTAR_T1W_LOST_RATIO_PCT) / 100U);
        if (diag_loss_dyn_th < ASTAR_T1W_LOST_TH)
          diag_loss_dyn_th = ASTAR_T1W_LOST_TH;

        if (!astar_side0_seen && side0_val > side0_th) {
          astar_side0_seen = 1;
          sprintf(buf, "A*T1W side0_seen d=%ld v=%d th=%d\r\n",
                  (long)motion_ctrl.traveled_distance, side0_val, side0_th);
          UART_SendString(buf);
        }

        if (!astar_l45_lost && astar_diag_seen && astar_side0_seen &&
            diag_val < diag_loss_dyn_th) {
          astar_l45_lost = 1;
          astar_l45_loss_dist = motion_ctrl.traveled_distance;
          sprintf(buf, "A*T1W loss@%ld v=%d th=%d pk=%d\r\n",
                  (long)astar_l45_loss_dist, diag_val, diag_loss_dyn_th,
                  astar_diag_peak);
          UART_SendString(buf);
        }

        if (astar_l45_lost && !astar_l0_lost && side0_val < side0_off_th) {
          k_now = motion_ctrl.traveled_distance - astar_l45_loss_dist;
          astar_l0_lost = 1;
          if (k_now > 15.0f && k_now < 150.0f) {
            astar_estimated_k = k_now;
          } else {
            k_now = astar_estimated_k;
          }
          astar_turn_target_dist = (k_now / 3.0f);
          if (astar_turn_target_dist < ASTAR_T1W_EXTRA_MIN_MM)
            astar_turn_target_dist = ASTAR_T1W_EXTRA_MIN_MM;

          Motion_StraightConstant(&motion_ctrl, astar_turn_target_dist);

          sprintf(buf, "A*T1W k=%ld tgt=%ld th=%d\r\n", (long)k_now,
                  (long)astar_turn_target_dist, side0_th);
          UART_SendString(buf);

          motion_start_time = millis();
          astar_state = ASTAR_APPROACH_PIVOT;
          break;
        }

        if ((millis() - astar_t1w_dbg_ms) > 120) {
          astar_t1w_dbg_ms = millis();
          sprintf(buf,
                  "A*T1W IR d=%ld L45=%d R45=%d L0=%d R0=%d D=%d S=%d th=%d "
                  "pk=%d st=%d%d%d%d\r\n",
                  (long)motion_ctrl.traveled_distance, l45_val, r45_val, l0_val,
                  r0_val, diag_val, side0_val, diag_loss_dyn_th,
                  astar_diag_peak, astar_diag_seen, astar_side0_seen,
                  astar_l45_lost, astar_l0_lost);
          UART_SendString(buf);
        }

        if (Motion_IsComplete(&motion_ctrl)) {
          sprintf(buf, "A*T1W fallback d=%ld D=%d S=%d st=%d%d%d%d\r\n",
                  (long)motion_ctrl.traveled_distance, diag_val, side0_val,
                  astar_diag_seen, astar_side0_seen, astar_l45_lost,
                  astar_l0_lost);
          UART_SendString(buf);
          /* Fallback by encoder: complete first cell, then advance extra
           * distance before pivoting. This preserves 180mm + extra behavior
           * even when wall-loss conditions are not observed. */
          astar_turn_target_dist = astar_estimated_k / 3.0f;
          if (astar_turn_target_dist < ASTAR_T1W_EXTRA_MIN_MM)
            astar_turn_target_dist = ASTAR_T1W_EXTRA_MIN_MM;

          Motion_StraightConstant(&motion_ctrl, astar_turn_target_dist);

          sprintf(buf, "A*T1W fallback_extra=%ldmm\r\n",
                  (long)astar_turn_target_dist);
          UART_SendString(buf);

          motion_start_time = millis();
          astar_state = ASTAR_APPROACH_PIVOT;
        }
      }
      break;

    case ASTAR_APPROACH_PIVOT:
      /* Driving the approach offset after post detection.
       * Once complete, execute the pivot turn immediately. */
      if ((millis() - astar_t1w_dbg_ms) > 120) {
        astar_t1w_dbg_ms = millis();
        sprintf(buf, "A*T1W app d=%ld/%ld\r\n",
                (long)motion_ctrl.traveled_distance,
                (long)motion_ctrl.target_distance);
        UART_SendString(buf);
      }
      if (Motion_IsComplete(&motion_ctrl)) {
        sprintf(buf, "A*T1W app_done d=%ld/%ld\r\n",
                (long)motion_ctrl.traveled_distance,
                (long)motion_ctrl.target_distance);
        UART_SendString(buf);
        motion_ctrl.pivot_turn_speed = 100.0f;
        Motion_SmoothTurn(&motion_ctrl, astar_smooth_angle);
        astar_t1w_turn_used = 1;

        maze.robot_dir = astar_smooth_new_dir;
        maze.robot_x = astar_smooth_final_x;
        maze.robot_y = astar_smooth_final_y;

        sprintf(buf, "Pivot: angle=%ld -> (%d,%d) d=%d\r\n",
                (long)astar_smooth_angle, maze.robot_x, maze.robot_y,
                maze.robot_dir);
        UART_SendString(buf);

        motion_start_time = millis();
        astar_state = ASTAR_WAIT_SMOOTH_TURN;
      }
      break;

    case ASTAR_WAIT_SMOOTH_TURN:
      if (Motion_IsComplete(&motion_ctrl)) {
        motion_ctrl.pivot_turn_speed = PIVOT_TURN_SPEED;
        motion_ctrl.current_speed = motion_ctrl.base_speed;
        if (motion_ctrl.speed_ff_gain > 0.001f) {
          motion_ctrl.desired_speed =
              motion_ctrl.base_speed / motion_ctrl.speed_ff_gain;
        }
        /* Pivot turn completed. Robot is at intermediate cell facing new dir.
         * Drive 1 more cell forward immediately to reach final cell.
         */
        if (Maze_IsAtCenter(&maze)) {
          UART_SendString("*** A* RUN: CENTER REACHED (pivot)! ***\r\n");
          Motion_Stop(&motion_ctrl);
          LED_OFF();
          system_state = STATE_MENU;
          break;
        }

        /* === CHAIN TURN: start second smooth turn immediately === */
        if (astar_chain_pending) {
          astar_chain_pending = 0;

          /* Set second turn parameters */
          astar_smooth_angle = astar_chain_angle;
          astar_smooth_new_dir = astar_chain_new_dir;
          astar_smooth_final_x = astar_chain_final_x;
          astar_smooth_final_y = astar_chain_final_y;

          /* Drive into next intermediate cell (= final of first turn) */
          Motion_StraightConstant(&motion_ctrl, CELL_SIZE_MM);

          /* Re-init T1WT detection for second turn */
          {
            uint16_t init_val;
            uint16_t init_side0;
            if (astar_smooth_angle > 0)
              init_val = (uint16_t)sensor_fusion.ir_sensors[1];
            else
              init_val = (uint16_t)sensor_fusion.ir_sensors[4];
            if (astar_smooth_angle > 0)
              init_side0 = (uint16_t)sensor_fusion.ir_sensors[2];
            else
              init_side0 = (uint16_t)sensor_fusion.ir_sensors[3];
            astar_prev_l45 = init_val;
            astar_peak_l45 = init_val;
            astar_trend_l45 = TREND_FLAT;
            astar_fall_l45 = 0;
            astar_rise_l45 = 0;
            astar_peak_armed = 0;
            astar_diag_seen = (init_val > ASTAR_T1W_PRESENT_TH) ? 1 : 0;
            astar_diag_peak = init_val;
            astar_side0_seen = 0;
            astar_l45_lost = 0;
            astar_l0_lost = 0;
            astar_l45_loss_dist = 0.0f;
            astar_turn_target_dist = 0.0f;
            astar_t1w_turn_used = 0;
            astar_t1w_dbg_ms = 0;
            sprintf(buf, "A*T1W chain init D=%d S=%d\r\n", init_val,
                    init_side0);
            UART_SendString(buf);
          }

          /* Position is already at first turn's final cell.
           * That cell becomes the intermediate cell for second turn. */
          sprintf(buf, "Chain pivot: (%d,%d)->(%d,%d)\r\n", maze.robot_x,
                  maze.robot_y, astar_chain_final_x, astar_chain_final_y);
          UART_SendString(buf);

          motion_start_time = millis();
          astar_state = ASTAR_SMOOTH_ENTERING;
        } else {
          /* Normal (non-chain): drive to final cell center */
          if (astar_t1w_turn_used) {
            Motion_Straight(&motion_ctrl, PIVOT_POST_TURN_ADVANCE_MM);
            sprintf(buf, "PivotFwd(T1W): %ldmm -> (%d,%d)\r\n",
                    (long)PIVOT_POST_TURN_ADVANCE_MM, maze.robot_x,
                    maze.robot_y);
          } else {
            Motion_Straight(&motion_ctrl, CELL_SIZE_MM);
            sprintf(buf, "PivotFwd(FB): %ldmm -> (%d,%d)\r\n",
                    (long)CELL_SIZE_MM, maze.robot_x, maze.robot_y);
          }
          motion_start_time = millis();
          astar_state = ASTAR_WAIT_MANEUVER;
          UART_SendString(buf);
        }
      }
      break;
    }

    /* Display update */
    if (millis() - last_display_time >= 250) {
      last_display_time = millis();
      SSD1306_Fill(SSD1306_BLACK);
      SSD1306_SetCursor(0, 0);
      SSD1306_WriteString("A* RUN", &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 15);
      sprintf(buf, "Step:%d/%d", astar_result.optimal_path_index,
              astar_result.optimal_path_length);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 30);
      sprintf(buf, "Pos:(%d,%d)", maze.robot_x, maze.robot_y);
      SSD1306_WriteString(buf, &Font_7x10, SSD1306_WHITE);

      SSD1306_SetCursor(0, 50);
      SSD1306_WriteString("Press=Exit", &Font_7x10, SSD1306_WHITE);
      SSD1306_UpdateScreen_DMA();
    }

    if (Hardware_ReadButton(0)) {
      Motion_Stop(&motion_ctrl);
      Motion_StopPID();
      Hardware_StopMotors();
      system_state = STATE_MENU;
      delay_ms_blocking(500);
      break;
    }
  }

  LED_OFF();
  Motion_StopPID();
  UART_SendString("A* run ended\r\n");
}

/* ========================================================================== */
/* RESET MAP MODE                                                             */
/* ========================================================================== */

void Execute_ResetMap(void) {
  UART_SendString("\r\n=== RESET MAP ===\r\n");

  Maze_Persistent_PrintStatus(&persistent_map);

  Display_Status("RESET MAP?", "Long press=Yes");
  UART_SendString(
      "Long press button to confirm RESET, short press to cancel\r\n");

  delay_ms_blocking(500); /* Debounce from menu selection */

  /* Wait for button press */
  {
    uint32_t wait_start = millis();
    while (millis() - wait_start < 5000) { /* 5s timeout */
      SSD1306_Process_DMA();

      if (Hardware_ReadButton(0)) {
        uint32_t press_start = millis();

        while (Hardware_ReadButton(0)) {
          SSD1306_Process_DMA();
          if (millis() - press_start > 1000) {
            /* Long press confirmed */
            Maze_Persistent_Reset(&persistent_map);
            Flash_SaveMaze(&persistent_map);
            UART_SendString("MAZE_RESET\r\n");

            Display_Status("MAP RESET", "Done!");
            UART_SendString("Persistent map ERASED!\r\n");
            delay_ms_blocking(500);
            system_state = STATE_MENU;
            return;
          }
        }

        /* Short press = cancel */
        UART_SendString("Reset cancelled\r\n");
        Display_Status("RESET MAP", "Cancelled");
        delay_ms_blocking(1000);
        system_state = STATE_MENU;
        return;
      }
    }
  }

  UART_SendString("Reset timed out\r\n");
  system_state = STATE_MENU;
}

/* ========================================================================== */
/* SPEEDRUN MODE                                                              */
/* ========================================================================== */

void Execute_Speedrun(void) {
  /* TODO: Implement speed run with optimized path */
  UART_SendString("\r\n=== SPEEDRUN MODE (not yet implemented) ===\r\n");
  Display_Status("SPEEDRUN", "Not ready");
  delay_ms_blocking(1000);
  system_state = STATE_MENU;
}

void Execute_SystemTest(void) {
  UART_SendString("\r\n=== ENTERING SYSTEM TEST SUB-MENU ===\r\n");

  SystemTest_Run(&sensor_fusion.mpu_handle);

  UART_SendString("Exiting System Test...\r\n");
  system_state = STATE_MENU;
}

/* ========================================================================== */
/* MAIN FUNCTION                                                              */
/* ========================================================================== */

int main(void) {
  /* Variables for main loop */
  static uint32_t last_main_tick = 0;

  /* ===== HARDWARE INITIALIZATION ===== */
  Hardware_Init();
  UART_Init();
  BT_Init();
  I2C_Init(I2C_1, 1);
  delay_ms_blocking(10);

  BT_LOG("[BT] Micromouse v2.1 Bluetooth debug ready\r\n");

  /* ===== DISPLAY INITIALIZATION ===== */
  SSD1306_Init_DMA();
  SSD1306_Fill(SSD1306_BLACK);

  SSD1306_SetCursor(10, 20);
  SSD1306_WriteString("MICROMOUSE", &Font_7x10, SSD1306_WHITE);
  SSD1306_SetCursor(35, 35);
  SSD1306_WriteString("v2.1", &Font_7x10, SSD1306_WHITE);
  SSD1306_UpdateScreen();
  delay_ms_blocking(200);

  /* ===== SENSORS & MODULES INITIALIZATION ===== */
  Hardware_InitEncoders();
  Hardware_InitIRSensors();
  Hardware_InitIRSensorsDMA();
  Hardware_InitMotors();

  delay_ms_blocking(20);

  /* ===== APPLICATION LAYER INITIALIZATION ===== */
  /* CRITICAL: Init SensorFusion BEFORE Motion (Starts TIM11) */
  /* Init MPU6050 uses blocking I2C, do this before arming I2C DMA */
  SensorFusion_Init(&sensor_fusion);

  /* Arm I2C DMA now that calibration is complete */
  I2C_DMA_Init(I2C_1);

  Motion_Init(&motion_ctrl);
  Maze_Init(&maze);
  Maze_Persistent_Init(&persistent_map);
  Flash_LoadMaze(&persistent_map);
  Flash_LoadIRCalib(&ir_calib);
  Flash_LoadMPUCalib(&mpu_calib_flash);

  /* Print saved maze map at startup */
  if (persistent_map.is_valid == MAZE_PERSISTENT_MAGIC &&
      persistent_map.total_runs > 0) {
    UART_SendString("\r\n=== SAVED MAZE FROM FLASH ===\r\n");
    Maze_Persistent_LoadInto(&persistent_map, &maze);
    UART_SendString("\r\n");
    Maze_FloodFillCenter(&maze);
    UART_SendString("\r\n");
    Maze_PrintCompact(&maze);
    UART_SendString("\r\n");
    Maze_Persistent_PrintStatus(&persistent_map);
  } else {
    UART_SendString("\r\n=== NO SAVED MAZE - BLANK MAP ===\r\n");
    Maze_Init(&maze);
    Maze_FloodFillCenter(&maze);
    Maze_PrintCompact(&maze);
  }

  UART_SendString("Application modules initialized!\r\n");

  /* ===== STARTUP MESSAGE ===== */
  UART_SendString("\r\n");
  UART_SendString("================================================\r\n");
  UART_SendString("       MICROMOUSE ROBOT v2.0                   \r\n");
  UART_SendString("================================================\r\n");
  UART_SendString("System initialized successfully!\r\n");

  /* ===== SHOW MENU ===== */
  system_state = STATE_MENU;
  Display_Menu();

  LED_ON();
  delay_ms_blocking(200);
  LED_OFF();

  /* ===== MAIN LOOP ===== */
  while (1) {
    SSD1306_Process_DMA();

    if (system_state == STATE_MENU) {
      /* Simple state variables for button debounce in main */
      static uint32_t last_btn_press_time = 0;

      if (Hardware_ReadButton(0) && (millis() - last_btn_press_time > 250)) {
        uint32_t press_start = millis();
        uint32_t led_start;
        uint8_t long_press_detected = 0;
        last_btn_press_time = press_start;

        while (Hardware_ReadButton(0)) {
          SSD1306_Process_DMA();

          if ((millis() - press_start > 400) && !long_press_detected) {
            long_press_detected = 1;

            selected_mode = (Menu_Item_t)menu_selection;
            system_state = STATE_RUNNING;

            {
              char log_buf[64];
              sprintf(log_buf, "[MENU] RUNNING: %s\r\n",
                      menu_items[menu_selection]);
              UART_SendString(log_buf);
            }

            SSD1306_Fill(SSD1306_BLACK);
            SSD1306_SetCursor(10, 25);
            SSD1306_WriteString("SELECTED!", &Font_11x18, SSD1306_WHITE);
            SSD1306_UpdateScreen();

            LED_ON();
            /* Non-blocking spin delay for visual feedback */
            led_start = millis();
            while (millis() - led_start < 500) {
              SSD1306_Process_DMA();
            }
            LED_OFF();

            while (Hardware_ReadButton(0))
              ;
            break;
          }
        }

        if (!long_press_detected) {
          menu_selection++;
          if (menu_selection >= MENU_COUNT)
            menu_selection = 0;
          Display_Menu();

          {
            char log_buf[64];
            sprintf(log_buf, "[MENU] Focus: %s\r\n",
                    menu_items[menu_selection]);
            UART_SendString(log_buf);
          }
        }
      }
    }

    else if (system_state == STATE_RUNNING) {
      switch (selected_mode) {
      case MENU_CALIBRATION:
        Execute_Calibration();
        break;

      case MENU_EXPLORE_CBC:
        Execute_MazeExplore(0);
        break;

      case MENU_EXPLORE_CONT:
        Execute_MazeExploreContinuous();
        break;

      case MENU_ASTAR_RUN:
        Execute_AStarRun();
        break;

      case MENU_SPEEDRUN:
        Execute_Speedrun();
        break;

      case MENU_RESET_MAP:
        Execute_ResetMap();
        break;

      case MENU_SYSTEM_TEST:
        Execute_SystemTest();
        break;

      default:
        system_state = STATE_MENU;
        break;
      }

      if (system_state == STATE_MENU) {
        UART_SendString("\r\nReturning to menu...\r\n");
        Display_Menu();
      }
    }

    /* Non-blocking main loop throttling */
    if (millis() - last_main_tick < 10) {
      continue; /* Skip rest of loop but let CPU run other fast tasks */
    }
    last_main_tick = millis();
  }
}
