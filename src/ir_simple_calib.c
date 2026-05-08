#include "ir_simple_calib.h"
#include "ir_sensor.h"
#include "motion_controller.h"
#include "system_timer.h"

#include "uart.h"
#include <stdio.h>
#include <string.h>

#define FRONT_SENSOR_COEFF 0.9f /* L90/R90: stored as raw × coeff */
#define SIDE_SENSOR_COEFF 0.95f /* L0/R0/L45/R45: stored as raw × coeff */
#define DEFAULT_WALL_THRESHOLD 150
#define WALL_ON_NUM 2
#define WALL_ON_DEN 3
#define WALL_OFF_NUM 1
#define WALL_OFF_DEN 7

/* --- Median Filter Utility --- */
static void Sort_Array(uint16_t *arr, uint16_t n) {
  uint16_t i, j, temp;
  for (i = 0; i < n - 1; i++) {
    for (j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

static uint16_t Median_Filter(uint16_t *arr, uint16_t n) {
  if (n == 0)
    return 0;
  Sort_Array(arr, n);
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2;
  } else {
    return arr[n / 2];
  }
}

/* Helper: take N static samples and return median of a given channel */
static uint16_t Sample_Channel_Median(uint8_t channel_idx, uint8_t n_samples) {
  uint16_t arr[40];
  uint16_t samples[6];
  uint8_t i;
  if (n_samples > 40)
    n_samples = 40;

  for (i = 0; i < n_samples; i++) {
    if (!IR_Sensor_IsReady())
      IR_Sensor_StartScan();
    while (!IR_Sensor_IsReady()) {
    }
    IR_Sensor_GetResults(samples);
    arr[i] = samples[channel_idx];
    delay_ms_blocking(10);
  }
  return Median_Filter(arr, n_samples);
}

void IR_Simple_Init(IR_Simple_Calib_t *calib) {
  memset(calib, 0, sizeof(IR_Simple_Calib_t));

  calib->wall_threshold_left = DEFAULT_WALL_THRESHOLD;
  calib->wall_threshold_right = DEFAULT_WALL_THRESHOLD;
  calib->wall_threshold_front_left = DEFAULT_WALL_THRESHOLD;
  calib->wall_threshold_front_right = DEFAULT_WALL_THRESHOLD;
  calib->wall_threshold_l45 = DEFAULT_WALL_THRESHOLD;
  calib->wall_threshold_r45 = DEFAULT_WALL_THRESHOLD;

  /* Default hysteresis = same as single threshold (no dead-band) */
  calib->wall_on_l0 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_l0 = DEFAULT_WALL_THRESHOLD / 2;
  calib->wall_on_r0 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_r0 = DEFAULT_WALL_THRESHOLD / 2;
  calib->wall_on_l90 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_l90 = DEFAULT_WALL_THRESHOLD / 2;
  calib->wall_on_r90 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_r90 = DEFAULT_WALL_THRESHOLD / 2;
  calib->wall_on_l45 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_l45 = DEFAULT_WALL_THRESHOLD / 2;
  calib->wall_on_r45 = DEFAULT_WALL_THRESHOLD;
  calib->wall_off_r45 = DEFAULT_WALL_THRESHOLD / 2;
  calib->side_level_l45 = 350;
  calib->side_level_r45 = 350;

  /* Gain balance: 1.0 = no correction (pass-through) */
  calib->gain_l0 = 1.0f;
  calib->gain_r0 = 1.0f;
  calib->gain_l45 = 1.0f;
  calib->gain_r45 = 1.0f;
  calib->gain_l90 = 1.0f;
  calib->gain_r90 = 1.0f;

  /* Front alignment: 0 = assume symmetric */
  calib->front_diff_zero = 0;
  calib->has_front_diff = 0;

  /* Event derivative thresholds: conservative defaults.
   * Negative = drop (wall disappears), Positive = rise (wall appears).
   * Magnitude depends on sensor — 30 ADC is a safe starting point. */
  calib->l45_drop_th = -30;
  calib->l45_rise_th = 30;
  calib->r45_drop_th = -30;
  calib->r45_rise_th = 30;

  /* Front interference: disabled (0xFFFF = never triggers) until calibrated */
  calib->front_interference_th = 0xFFFF;

  calib->center_r90 = 0;
  calib->calib_mode = 1;
  calib->is_calibrated = 0;
  calib->has_open_calib = 0;
}

/* ==========================================================================
 * Step 1: 90/0 center calibration
 *   Mode 1: robot moves forward 20mm (spatial sampling)
 *   Mode 2: robot stands still (static sampling, 20 samples per channel)
 * ========================================================================== */
void IR_Simple_Calib90and0(IR_Simple_Calib_t *calib,
                           struct Motion_Controller_t *motion_ctrl) {
  char buf[80];

  if (calib->calib_mode >= 2) {
    /* --- MODE 2/3: STATIC SAMPLING (robot held still by hand) --- */
    uint16_t med_l90, med_r90, med_l0, med_r0;

    delay_ms_blocking(500);
    UART_SendString("Static sampling 90/0 (20 samples)...\r\n");

    med_l90 = Sample_Channel_Median(0, 20); /* L90 */
    med_r90 = Sample_Channel_Median(5, 20); /* R90 */
    med_l0 = Sample_Channel_Median(2, 20);  /* L0  */
    med_r0 = Sample_Channel_Median(3, 20);  /* R0  */

    sprintf(buf, "Static: L90=%d L0=%d R0=%d R90=%d\r\n", med_l90, med_l0,
            med_r0, med_r90);
    UART_SendString(buf);

    calib->center_l90 = med_l90;
    calib->center_r90 = med_r90;
    calib->front_fsum_target = (uint32_t)med_l90 + (uint32_t)med_r90;
    calib->center_l0 = med_l0;
    calib->center_r0 = med_r0;

    calib->wall_threshold_left = (uint16_t)(med_l90 * FRONT_SENSOR_COEFF);
    calib->wall_threshold_front_left = (uint16_t)(med_l0 * SIDE_SENSOR_COEFF);
    calib->wall_threshold_front_right = (uint16_t)(med_r0 * SIDE_SENSOR_COEFF);
    calib->wall_threshold_right = (uint16_t)(med_r90 * FRONT_SENSOR_COEFF);

    if (calib->wall_threshold_left < 10)
      calib->wall_threshold_left = 10;
    if (calib->wall_threshold_front_left < 10)
      calib->wall_threshold_front_left = 10;
    if (calib->wall_threshold_front_right < 10)
      calib->wall_threshold_front_right = 10;
    if (calib->wall_threshold_right < 10)
      calib->wall_threshold_right = 10;

    /* Gain equalization: normalize Right to match Left */
    calib->gain_l0 = 1.0f;
    calib->gain_l90 = 1.0f;
    if (med_r0 > 10) {
      calib->gain_r0 = (float)med_l0 / (float)med_r0;
      if (calib->gain_r0 < 0.5f)
        calib->gain_r0 = 0.5f;
      if (calib->gain_r0 > 2.0f)
        calib->gain_r0 = 2.0f;
    } else {
      calib->gain_r0 = 1.0f;
    }
    if (med_r90 > 10) {
      calib->gain_r90 = (float)med_l90 / (float)med_r90;
      if (calib->gain_r90 < 0.5f)
        calib->gain_r90 = 0.5f;
      if (calib->gain_r90 > 2.0f)
        calib->gain_r90 = 2.0f;
    } else {
      calib->gain_r90 = 1.0f;
    }

    sprintf(buf, "Gain: R0=%.3f R90=%.3f\r\n", calib->gain_r0, calib->gain_r90);
    UART_SendString(buf);

    UART_SendString("90/0 Static Calibration complete.\r\n");

  } else {
    /* --- MODE 1: SPATIAL SAMPLING (robot moves 20mm) --- */
    uint16_t l90_arr[100], r90_arr[100], l0_arr[100], r0_arr[100];
    uint16_t count = 0;
    uint16_t samples[6];

    delay_ms_blocking(1000);
    UART_SendString("Move 20mm to sample...\r\n");

    motion_ctrl->current_speed = 30.0f;
    motion_ctrl->max_speed = 30.0f;
    Motion_Straight(motion_ctrl, 20.0f);

    while (!Motion_IsComplete(motion_ctrl) && count < 100) {
      if (!IR_Sensor_IsReady())
        IR_Sensor_StartScan();
      while (!IR_Sensor_IsReady() && !Motion_IsComplete(motion_ctrl)) {
      }
      if (IR_Sensor_IsReady()) {
        IR_Sensor_GetResults(samples);
        l90_arr[count] = samples[0];
        l0_arr[count] = samples[2];
        r0_arr[count] = samples[3];
        r90_arr[count] = samples[5];
        count++;
      }
      delay_ms_blocking(2);
    }
    Motion_Stop(motion_ctrl);

    if (count == 0) {
      UART_SendString("Error: No samples collected!\r\n");
      return;
    }

    {
      uint16_t med_l90 = Median_Filter(l90_arr, count);
      uint16_t med_r90 = Median_Filter(r90_arr, count);
      uint16_t med_l0 = Median_Filter(l0_arr, count);
      uint16_t med_r0 = Median_Filter(r0_arr, count);

      sprintf(buf, "%d samples. Medians: L90=%d L0=%d R0=%d R90=%d\r\n", count,
              med_l90, med_l0, med_r0, med_r90);
      UART_SendString(buf);

      calib->center_l90 = med_l90;
      calib->center_r90 = med_r90;
      calib->front_fsum_target = (uint32_t)med_l90 + (uint32_t)med_r90;
      calib->center_l0 = med_l0;
      calib->center_r0 = med_r0;

      calib->wall_threshold_left = (uint16_t)(med_l90 * FRONT_SENSOR_COEFF);
      calib->wall_threshold_front_left = (uint16_t)(med_l0 * SIDE_SENSOR_COEFF);
      calib->wall_threshold_front_right =
          (uint16_t)(med_r0 * SIDE_SENSOR_COEFF);
      calib->wall_threshold_right = (uint16_t)(med_r90 * FRONT_SENSOR_COEFF);

      if (calib->wall_threshold_left < 10)
        calib->wall_threshold_left = 10;
      if (calib->wall_threshold_front_left < 10)
        calib->wall_threshold_front_left = 10;
      if (calib->wall_threshold_front_right < 10)
        calib->wall_threshold_front_right = 10;
      if (calib->wall_threshold_right < 10)
        calib->wall_threshold_right = 10;

      /* Gain equalization: normalize Right to match Left */
      calib->gain_l0 = 1.0f;
      calib->gain_l90 = 1.0f;
      if (med_r0 > 10) {
        calib->gain_r0 = (float)med_l0 / (float)med_r0;
        if (calib->gain_r0 < 0.5f)
          calib->gain_r0 = 0.5f;
        if (calib->gain_r0 > 2.0f)
          calib->gain_r0 = 2.0f;
      } else {
        calib->gain_r0 = 1.0f;
      }
      if (med_r90 > 10) {
        calib->gain_r90 = (float)med_l90 / (float)med_r90;
        if (calib->gain_r90 < 0.5f)
          calib->gain_r90 = 0.5f;
        if (calib->gain_r90 > 2.0f)
          calib->gain_r90 = 2.0f;
      } else {
        calib->gain_r90 = 1.0f;
      }

      sprintf(buf, "Gain: R0=%.3f R90=%.3f\r\n", calib->gain_r0,
              calib->gain_r90);
      UART_SendString(buf);
    }

    UART_SendString("90/0 Degree Calibration complete.\r\n");
  }
}

/* ==========================================================================
 * Step 2: 45-degree calibration
 *   Mode 1: robot moves forward 30mm (spatial sampling)
 *   Mode 2: robot stands still (static sampling)
 * ========================================================================== */
void IR_Simple_Calib45(IR_Simple_Calib_t *calib,
                       struct Motion_Controller_t *motion_ctrl) {
  char buf[80];

  if (calib->calib_mode >= 2) {
    /* --- MODE 2/3: STATIC SAMPLING --- */
    uint16_t med_l45, med_r45;

    delay_ms_blocking(500);
    UART_SendString("Static sampling 45 (20 samples)...\r\n");

    med_l45 = Sample_Channel_Median(1, 20); /* L45 */
    med_r45 = Sample_Channel_Median(4, 20); /* R45 */

    sprintf(buf, "Static: L45=%d R45=%d\r\n", med_l45, med_r45);
    UART_SendString(buf);

    calib->wall_threshold_l45 = (uint16_t)(med_l45 * SIDE_SENSOR_COEFF);
    calib->wall_threshold_r45 = (uint16_t)(med_r45 * SIDE_SENSOR_COEFF);

    if (calib->wall_threshold_l45 < 10)
      calib->wall_threshold_l45 = 10;
    if (calib->wall_threshold_r45 < 10)
      calib->wall_threshold_r45 = 10;

    /* Gain equalization for L45/R45 */
    calib->gain_l45 = 1.0f;
    if (med_r45 > 10) {
      calib->gain_r45 = (float)med_l45 / (float)med_r45;
      if (calib->gain_r45 < 0.5f)
        calib->gain_r45 = 0.5f;
      if (calib->gain_r45 > 2.0f)
        calib->gain_r45 = 2.0f;
    } else {
      calib->gain_r45 = 1.0f;
    }

    sprintf(buf, "Gain: R45=%.3f\r\n", calib->gain_r45);
    UART_SendString(buf);

    calib->is_calibrated = 0xCA;

    UART_SendString("45 Static Calibration complete.\r\n");

  } else {
    /* --- MODE 1: SPATIAL SAMPLING --- */
    uint16_t l45_arr[100], r45_arr[100];
    uint16_t count = 0;
    uint16_t samples[6];

    delay_ms_blocking(1000);
    UART_SendString("Move 50mm to sample...\r\n");

    motion_ctrl->current_speed = 30.0f;
    motion_ctrl->max_speed = 30.0f;
    Motion_Straight(motion_ctrl, 50.0f);

    while (!Motion_IsComplete(motion_ctrl) && count < 100) {
      if (!IR_Sensor_IsReady())
        IR_Sensor_StartScan();
      while (!IR_Sensor_IsReady() && !Motion_IsComplete(motion_ctrl)) {
      }
      if (IR_Sensor_IsReady()) {
        IR_Sensor_GetResults(samples);
        l45_arr[count] = samples[1];
        r45_arr[count] = samples[4];
        count++;
      }
      delay_ms_blocking(2);
    }
    Motion_Stop(motion_ctrl);

    if (count == 0) {
      UART_SendString("Error: No samples collected!\r\n");
      return;
    }

    {
      uint16_t med_l45 = Median_Filter(l45_arr, count);
      uint16_t med_r45 = Median_Filter(r45_arr, count);

      sprintf(buf, "%d samples. Medians (45deg): L45=%d R45=%d\r\n", count,
              med_l45, med_r45);
      UART_SendString(buf);

      calib->wall_threshold_l45 = (uint16_t)(med_l45 * SIDE_SENSOR_COEFF);
      calib->wall_threshold_r45 = (uint16_t)(med_r45 * SIDE_SENSOR_COEFF);

      if (calib->wall_threshold_l45 < 10)
        calib->wall_threshold_l45 = 10;
      if (calib->wall_threshold_r45 < 10)
        calib->wall_threshold_r45 = 10;

      /* Gain equalization for L45/R45 */
      calib->gain_l45 = 1.0f;
      if (med_r45 > 10) {
        calib->gain_r45 = (float)med_l45 / (float)med_r45;
        if (calib->gain_r45 < 0.5f)
          calib->gain_r45 = 0.5f;
        if (calib->gain_r45 > 2.0f)
          calib->gain_r45 = 2.0f;
      } else {
        calib->gain_r45 = 1.0f;
      }

      sprintf(buf, "Gain: R45=%.3f\r\n", calib->gain_r45);
      UART_SendString(buf);
    }

    calib->is_calibrated = 0xCA;
    UART_SendString("45 Degree Calibration complete.\r\n");
  }
}

/* ==========================================================================
 * Steps 3+4 (Mode 2+3): wall-proximity static sampling
 * ========================================================================== */

void IR_Simple_CalibLeftWall(IR_Simple_Calib_t *calib) {
  /* Robot is held against LEFT wall:
   * L0 (idx 2) and L45 (idx 1) → HIGH (max)
   * R0 (idx 3) and R45 (idx 4) → LOW  (min) */
  char buf[80];
  delay_ms_blocking(500);
  UART_SendString("Sampling LEFT wall proximity (20 samples)...\r\n");

  calib->l0_max = Sample_Channel_Median(2, 20);
  calib->l45_max = Sample_Channel_Median(1, 20);
  calib->r0_min = Sample_Channel_Median(3, 20);
  calib->r45_min = Sample_Channel_Median(4, 20);

  sprintf(buf, "Left wall: L0_max=%d L45_max=%d R0_min=%d R45_min=%d\r\n",
          calib->l0_max, calib->l45_max, calib->r0_min, calib->r45_min);
  UART_SendString(buf);
}

void IR_Simple_CalibRightWall(IR_Simple_Calib_t *calib) {
  /* Robot is held against RIGHT wall:
   * R0 (idx 3) and R45 (idx 4) → HIGH (max)
   * L0 (idx 2) and L45 (idx 1) → LOW  (min) */
  char buf[80];
  delay_ms_blocking(500);
  UART_SendString("Sampling RIGHT wall proximity (20 samples)...\r\n");

  calib->r0_max = Sample_Channel_Median(3, 20);
  calib->r45_max = Sample_Channel_Median(4, 20);
  calib->l0_min = Sample_Channel_Median(2, 20);
  calib->l45_min = Sample_Channel_Median(1, 20);

  sprintf(buf, "Right wall: R0_max=%d R45_max=%d L0_min=%d L45_min=%d\r\n",
          calib->r0_max, calib->r45_max, calib->l0_min, calib->l45_min);
  UART_SendString(buf);
}

/* ==========================================================================
 * RefineThresholds: ONLY store min/max data, do NOT override center-based
 * thresholds. The min/max data is used for normalized wall STEERING only.
 * Wall DETECTION threshold remains center * SIDE_SENSOR_COEFF (set in step1/2).
 * ========================================================================== */
void IR_Simple_RefineThresholds(IR_Simple_Calib_t *calib) {
  char buf[80];

  /* Just log the min/max for debugging — thresholds stay as-is from step 1+2 */
  sprintf(buf, "MinMax: L0[%d-%d] R0[%d-%d] L45[%d-%d] R45[%d-%d]\r\n",
          calib->l0_min, calib->l0_max, calib->r0_min, calib->r0_max,
          calib->l45_min, calib->l45_max, calib->r45_min, calib->r45_max);
  UART_SendString(buf);

  sprintf(buf, "Thresholds KEPT: L0=%d R0=%d L45=%d R45=%d\r\n",
          calib->wall_threshold_front_left, calib->wall_threshold_front_right,
          calib->wall_threshold_l45, calib->wall_threshold_r45);
  UART_SendString(buf);
}

/* ==========================================================================
 * Steps 5+6 (Mode 3 only): metal post calibration
 * Posts reflect STRONGER than walls → l90_post > center_l90
 * ========================================================================== */

void IR_Simple_CalibLeftPost(IR_Simple_Calib_t *calib) {
  char buf[80];
  delay_ms_blocking(500);
  UART_SendString("Sampling L90 near metal post (20 samples)...\r\n");

  calib->l90_post = Sample_Channel_Median(0, 20); /* L90 = index 0 */

  sprintf(buf, "L90 post value: %d (wall center: %d)\r\n", calib->l90_post,
          calib->center_l90);
  UART_SendString(buf);
}

void IR_Simple_CalibRightPost(IR_Simple_Calib_t *calib) {
  char buf[80];
  delay_ms_blocking(500);
  UART_SendString("Sampling R90 near metal post (20 samples)...\r\n");

  calib->r90_post = Sample_Channel_Median(5, 20); /* R90 = index 5 */

  sprintf(buf, "R90 post value: %d (wall center: %d)\r\n", calib->r90_post,
          calib->center_r90);
  UART_SendString(buf);
}

/* ==========================================================================
 * Open-space calibration: sample all 6 channels with NO walls nearby
 * → A values = max(NO_WALL) baseline for hysteresis
 * ========================================================================== */

void IR_Simple_CalibOpenSpace(IR_Simple_Calib_t *calib) {
  char buf[80];
  delay_ms_blocking(500);
  UART_SendString("Open-space sampling (20 samples, no walls)...\r\n");

  calib->open_l90 = Sample_Channel_Median(0, 20); /* L90 */
  calib->open_l45 = Sample_Channel_Median(1, 20); /* L45 */
  calib->open_l0 = Sample_Channel_Median(2, 20);  /* L0  */
  calib->open_r0 = Sample_Channel_Median(3, 20);  /* R0  */
  calib->open_r45 = Sample_Channel_Median(4, 20); /* R45 */
  calib->open_r90 = Sample_Channel_Median(5, 20); /* R90 */

  calib->has_open_calib = 1;

  sprintf(buf, "Open: L90=%d L45=%d L0=%d R0=%d R45=%d R90=%d\r\n",
          calib->open_l90, calib->open_l45, calib->open_l0, calib->open_r0,
          calib->open_r45, calib->open_r90);
  UART_SendString(buf);
}

/* ==========================================================================
 * Compute hysteresis thresholds from A (open) and B (center) values
 *   gap = B - A
 *   WALL_ON  = A + gap*WALL_ON_NUM/WALL_ON_DEN
 *   WALL_OFF = A + gap*WALL_OFF_NUM/WALL_OFF_DEN
 * ========================================================================== */

static void Compute_Pair(uint16_t A, uint16_t B, uint16_t *wall_on,
                         uint16_t *wall_off) {
  int16_t gap;
  if (B <= A) {
    /* Invalid: wall reading not higher than open → use B as fallback */
    *wall_on = B;
    *wall_off = B / 2;
    return;
  }
  gap = (int16_t)(B - A);
  *wall_on = A + (uint16_t)(gap * WALL_ON_NUM / WALL_ON_DEN);
  *wall_off = A + (uint16_t)(gap * WALL_OFF_NUM / WALL_OFF_DEN);
  if (*wall_on < 10)
    *wall_on = 10;
  if (*wall_off < 5)
    *wall_off = 5;
}

void IR_Simple_ComputeHysteresis(IR_Simple_Calib_t *calib) {
  char buf[80];

  if (!calib->has_open_calib) {
    UART_SendString("WARN: No open-space calib, using coeff fallback\r\n");
    return;
  }

  /* L0 (side-left wall) */
  Compute_Pair(calib->open_l0, calib->center_l0, &calib->wall_on_l0,
               &calib->wall_off_l0);
  /* R0 (side-right wall) */
  Compute_Pair(calib->open_r0, calib->center_r0, &calib->wall_on_r0,
               &calib->wall_off_r0);
  /* L90 (front-left) */
  Compute_Pair(calib->open_l90, calib->center_l90, &calib->wall_on_l90,
               &calib->wall_off_l90);
  /* R90 (front-right) */
  Compute_Pair(calib->open_r90, calib->center_r90, &calib->wall_on_r90,
               &calib->wall_off_r90);

  /* L45/R45: B = value from Calib45 step (wall_threshold_l45 was set as
   * Median * SIDE_SENSOR_COEFF; use the raw Median ≈ threshold/coeff) */
  {
    uint16_t b_l45 = (uint16_t)(calib->wall_threshold_l45 / SIDE_SENSOR_COEFF);
    uint16_t b_r45 = (uint16_t)(calib->wall_threshold_r45 / SIDE_SENSOR_COEFF);
    Compute_Pair(calib->open_l45, b_l45, &calib->wall_on_l45,
                 &calib->wall_off_l45);
    Compute_Pair(calib->open_r45, b_r45, &calib->wall_on_r45,
                 &calib->wall_off_r45);
    /* Side level: full-wall peak ≈ B value (Median when wall present) */
    calib->side_level_l45 = b_l45;
    calib->side_level_r45 = b_r45;
  }

  /* Update legacy single-threshold for backward compat */
  calib->wall_threshold_front_left = calib->wall_on_l0;
  calib->wall_threshold_front_right = calib->wall_on_r0;
  calib->wall_threshold_left = calib->wall_on_l90;
  calib->wall_threshold_right = calib->wall_on_r90;
  calib->wall_threshold_l45 = calib->wall_on_l45;
  calib->wall_threshold_r45 = calib->wall_on_r45;

  /* Also populate min/max from open/center for Mode 1 wall steering */
  if (calib->calib_mode == 1) {
    calib->l0_min = calib->open_l0;
    calib->l0_max = calib->center_l0;
    calib->r0_min = calib->open_r0;
    calib->r0_max = calib->center_r0;
    calib->l45_min = calib->open_l45;
    calib->r45_min = calib->open_r45;
  }

  /* ---- Task-based: front interference threshold ---- */
  /* When L0+R0 sum exceeds this, side sensors are affected by front wall
   * cross-reflection.  Use center L0+R0 (= both side walls present) plus
   * a 30% margin so normal corridor doesn't trigger it. */
  {
    uint32_t side_sum_center =
        (uint32_t)calib->center_l0 + (uint32_t)calib->center_r0;
    calib->front_interference_th = (uint16_t)(side_sum_center * 13 / 10);
    if (calib->front_interference_th < 200)
      calib->front_interference_th = 200;
  }

  /* ---- Task-based: event derivative thresholds for L45/R45 ---- */
  /* Use 1/4 of the gap between open and wall as the minimum delta
   * that qualifies as an edge event.  Sign: drop is negative. */
  {
    uint16_t b_l45 = (uint16_t)(calib->wall_threshold_l45 / SIDE_SENSOR_COEFF);
    uint16_t b_r45 = (uint16_t)(calib->wall_threshold_r45 / SIDE_SENSOR_COEFF);
    int16_t gap_l45, gap_r45;

    gap_l45 =
        (b_l45 > calib->open_l45) ? (int16_t)(b_l45 - calib->open_l45) : 30;
    gap_r45 =
        (b_r45 > calib->open_r45) ? (int16_t)(b_r45 - calib->open_r45) : 30;

    calib->l45_drop_th = -(gap_l45 / 4);
    calib->l45_rise_th = (gap_l45 / 4);
    calib->r45_drop_th = -(gap_r45 / 4);
    calib->r45_rise_th = (gap_r45 / 4);

    /* Clamp to sensible minimum magnitude */
    if (calib->l45_drop_th > -10)
      calib->l45_drop_th = -10;
    if (calib->l45_rise_th < 10)
      calib->l45_rise_th = 10;
    if (calib->r45_drop_th > -10)
      calib->r45_drop_th = -10;
    if (calib->r45_rise_th < 10)
      calib->r45_rise_th = 10;
  }

  sprintf(buf, "Hyst L0:  ON=%d OFF=%d\r\n", calib->wall_on_l0,
          calib->wall_off_l0);
  UART_SendString(buf);
  sprintf(buf, "Hyst R0:  ON=%d OFF=%d\r\n", calib->wall_on_r0,
          calib->wall_off_r0);
  UART_SendString(buf);
  sprintf(buf, "Hyst L90: ON=%d OFF=%d\r\n", calib->wall_on_l90,
          calib->wall_off_l90);
  UART_SendString(buf);
  sprintf(buf, "Hyst R90: ON=%d OFF=%d\r\n", calib->wall_on_r90,
          calib->wall_off_r90);
  UART_SendString(buf);
  sprintf(buf, "Hyst L45: ON=%d OFF=%d LVL=%d\r\n", calib->wall_on_l45,
          calib->wall_off_l45, calib->side_level_l45);
  UART_SendString(buf);
  sprintf(buf, "Hyst R45: ON=%d OFF=%d LVL=%d\r\n", calib->wall_on_r45,
          calib->wall_off_r45, calib->side_level_r45);
  UART_SendString(buf);
  sprintf(buf, "FrontInterf: %d\r\n", calib->front_interference_th);
  UART_SendString(buf);
  sprintf(buf, "EvtDeriv L45: drop=%d rise=%d\r\n", calib->l45_drop_th,
          calib->l45_rise_th);
  UART_SendString(buf);
  sprintf(buf, "EvtDeriv R45: drop=%d rise=%d\r\n", calib->r45_drop_th,
          calib->r45_rise_th);
  UART_SendString(buf);
}

/* ==========================================================================
 * Front-diff calibration: robot facing front wall perpendicularly.
 * Computes front_diff_zero = mean(L90_eq - R90_eq) for angular alignment.
 * Also refines front_fsum_target from this accurate perpendicular pose.
 * ========================================================================== */

void IR_Simple_CalibFrontDiff(IR_Simple_Calib_t *calib) {
  char buf[80];
  uint16_t med_l90, med_r90;
  int16_t l90_eq, r90_eq;

  delay_ms_blocking(500);
  UART_SendString("Front-diff sampling (20 samples, facing front wall)...\r\n");

  med_l90 = Sample_Channel_Median(0, 20); /* L90 */
  med_r90 = Sample_Channel_Median(5, 20); /* R90 */

  /* Apply gain equalization */
  l90_eq = (int16_t)(calib->gain_l90 * (float)med_l90);
  r90_eq = (int16_t)(calib->gain_r90 * (float)med_r90);

  calib->front_diff_zero = l90_eq - r90_eq;

  /* Also update front_fsum_target from this precise perpendicular pose */
  calib->front_fsum_target = (uint32_t)l90_eq + (uint32_t)r90_eq;

  calib->has_front_diff = 1;

  sprintf(buf, "FrontDiff: L90_eq=%d R90_eq=%d diff_zero=%d fsum=%lu\r\n",
          l90_eq, r90_eq, calib->front_diff_zero, calib->front_fsum_target);
  UART_SendString(buf);
}

/* ==========================================================================
 * Apply gain equalization to a raw sensor array in-place.
 * Input/output: values[6] = {L90, L45, L0, R0, R45, R90}
 * Left sensors (L90=idx0, L45=idx1, L0=idx2) use gain_l* (typically 1.0).
 * Right sensors (R0=idx3, R45=idx4, R90=idx5) use gain_r*.
 * ========================================================================== */

void IR_Simple_ApplyGain(IR_Simple_Calib_t *calib, uint16_t *values) {
  values[0] = (uint16_t)(calib->gain_l90 * (float)values[0]);
  values[1] = (uint16_t)(calib->gain_l45 * (float)values[1]);
  values[2] = (uint16_t)(calib->gain_l0 * (float)values[2]);
  values[3] = (uint16_t)(calib->gain_r0 * (float)values[3]);
  values[4] = (uint16_t)(calib->gain_r45 * (float)values[4]);
  values[5] = (uint16_t)(calib->gain_r90 * (float)values[5]);
}

/* ==========================================================================
 * Wall Detection (single-threshold, for stopped reads at cell center)
 * ========================================================================== */

uint8_t IR_Simple_DetectLeftWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw) {
  /* Use wall_on threshold (= legacy threshold when has_open_calib) */
  return (ir_raw[2] > calib->wall_threshold_front_left) ? 1 : 0;
}

uint8_t IR_Simple_DetectFrontWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw) {
  /* L90 (ir_raw[0]) = front-left, R90 (ir_raw[5]) = front-right */
  uint16_t l90_th = (uint16_t)(calib->wall_threshold_left * 0.20f);
  uint16_t r90_th = (uint16_t)(calib->wall_threshold_right * 0.20f);
  uint8_t l90_hit = 0, r90_hit = 0;
  if (l90_th < 10)
    l90_th = 10;
  if (r90_th < 10)
    r90_th = 10;

  l90_hit = (ir_raw[0] > l90_th) ? 1 : 0;
  r90_hit = (ir_raw[5] > r90_th) ? 1 : 0;

  /* Simple threshold: any front hit means front wall */
  return (l90_hit || r90_hit) ? 1 : 0;
}

uint8_t IR_Simple_DetectRightWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw) {
  return (ir_raw[3] > calib->wall_threshold_front_right) ? 1 : 0;
}

/* ==========================================================================
 * Hysteresis Wall Detection (for continuous reads while moving)
 *   state persists across calls: 0=no wall, 1=wall
 *   raw >= wall_on  → state = 1
 *   raw <= wall_off → state = 0
 *   between → hold previous state
 * ========================================================================== */

uint8_t IR_Simple_DetectLeftWall_Hyst(IR_Simple_Calib_t *calib, uint16_t raw_l0,
                                      uint8_t *state) {
  if (*state == 0 && raw_l0 >= calib->wall_on_l0)
    *state = 1;
  else if (*state == 1 && raw_l0 <= calib->wall_off_l0)
    *state = 0;
  return *state;
}

uint8_t IR_Simple_DetectFrontWall_Hyst(IR_Simple_Calib_t *calib,
                                       uint16_t raw_l90, uint16_t raw_r90,
                                       uint8_t *state) {
  uint8_t l_on = (raw_l90 >= calib->wall_on_l90) ? 1 : 0;
  uint8_t r_on = (raw_r90 >= calib->wall_on_r90) ? 1 : 0;
  uint8_t l_off = (raw_l90 <= calib->wall_off_l90) ? 1 : 0;
  uint8_t r_off = (raw_r90 <= calib->wall_off_r90) ? 1 : 0;

  if (*state == 0 && (l_on || r_on))
    *state = 1;
  else if (*state == 1 && l_off && r_off)
    *state = 0;
  return *state;
}

uint8_t IR_Simple_DetectRightWall_Hyst(IR_Simple_Calib_t *calib,
                                       uint16_t raw_r0, uint8_t *state) {
  if (*state == 0 && raw_r0 >= calib->wall_on_r0)
    *state = 1;
  else if (*state == 1 && raw_r0 <= calib->wall_off_r0)
    *state = 0;
  return *state;
}

void IR_Simple_PrintCalib(IR_Simple_Calib_t *calib) {
  char buf[80];

  sprintf(buf, "=== IR Calib (Mode %d) ===\r\n", calib->calib_mode);
  UART_SendString(buf);
  sprintf(buf, "Center L90:%d R90:%d  L0:%d R0:%d\r\n", calib->center_l90,
          calib->center_r90, calib->center_l0, calib->center_r0);
  UART_SendString(buf);
  if (calib->has_open_calib) {
    sprintf(buf, "Open   L90:%d R90:%d  L0:%d R0:%d\r\n", calib->open_l90,
            calib->open_r90, calib->open_l0, calib->open_r0);
    UART_SendString(buf);
    sprintf(buf, "Open   L45:%d R45:%d\r\n", calib->open_l45, calib->open_r45);
    UART_SendString(buf);
    sprintf(buf, "Hyst L0: ON=%d OFF=%d  R0: ON=%d OFF=%d\r\n",
            calib->wall_on_l0, calib->wall_off_l0, calib->wall_on_r0,
            calib->wall_off_r0);
    UART_SendString(buf);
    sprintf(buf, "Hyst L90:ON=%d OFF=%d  R90:ON=%d OFF=%d\r\n",
            calib->wall_on_l90, calib->wall_off_l90, calib->wall_on_r90,
            calib->wall_off_r90);
    UART_SendString(buf);
    sprintf(buf, "Hyst L45:ON=%d OFF=%d  R45:ON=%d OFF=%d\r\n",
            calib->wall_on_l45, calib->wall_off_l45, calib->wall_on_r45,
            calib->wall_off_r45);
    UART_SendString(buf);
    sprintf(buf, "Side LVL L45:%d R45:%d\r\n", calib->side_level_l45,
            calib->side_level_r45);
    UART_SendString(buf);
  } else {
    sprintf(buf, "Thresh L90: %d  R90: %d\r\n", calib->wall_threshold_left,
            calib->wall_threshold_right);
    UART_SendString(buf);
    sprintf(buf, "Thresh L0:  %d  R0: %d\r\n", calib->wall_threshold_front_left,
            calib->wall_threshold_front_right);
    UART_SendString(buf);
    sprintf(buf, "Thresh L45: %d  R45: %d\r\n", calib->wall_threshold_l45,
            calib->wall_threshold_r45);
    UART_SendString(buf);
  }
  if (calib->calib_mode == 3) {
    sprintf(buf, "Post L90: %d  R90: %d\r\n", calib->l90_post, calib->r90_post);
    UART_SendString(buf);
  }

  /* Task-based calibration parameters */
  sprintf(buf, "Gain: L0=%.3f R0=%.3f L45=%.3f R45=%.3f\r\n", calib->gain_l0,
          calib->gain_r0, calib->gain_l45, calib->gain_r45);
  UART_SendString(buf);
  sprintf(buf, "Gain: L90=%.3f R90=%.3f\r\n", calib->gain_l90, calib->gain_r90);
  UART_SendString(buf);
  if (calib->has_front_diff) {
    sprintf(buf, "FrontDiff zero=%d  Fsum=%lu\r\n", calib->front_diff_zero,
            calib->front_fsum_target);
    UART_SendString(buf);
  }
  sprintf(buf, "FrontInterf: %d\r\n", calib->front_interference_th);
  UART_SendString(buf);
  sprintf(buf, "EvtDeriv L45: drop=%d rise=%d  R45: drop=%d rise=%d\r\n",
          calib->l45_drop_th, calib->l45_rise_th, calib->r45_drop_th,
          calib->r45_rise_th);
  UART_SendString(buf);
}
