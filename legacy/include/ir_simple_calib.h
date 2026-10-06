#ifndef IR_SIMPLE_CALIB_H
#define IR_SIMPLE_CALIB_H

#include <stdint.h>

struct Motion_Controller_t;

/* IR calibration structure with hysteresis wall detection */
typedef struct {
  /* Legacy single-threshold (kept for backward compat, = wall_on_*) */
  uint16_t wall_threshold_left;        /* L90 threshold (front-left sensor) */
  uint16_t wall_threshold_right;       /* R90 threshold (front-right sensor) */
  uint16_t wall_threshold_front_left;  /* L0 threshold (side-left sensor) */
  uint16_t wall_threshold_front_right; /* R0 threshold (side-right sensor) */
  uint16_t wall_threshold_l45;         /* L45 threshold */
  uint16_t wall_threshold_r45;         /* R45 threshold */

  /* Front wall center reference (raw values when robot at cell center) */
  uint16_t center_l90;        /* L90 raw average at center position */
  uint16_t center_r90;        /* R90 raw average at center position */
  uint32_t front_fsum_target; /* L90+R90 at cell center (Fsum reference) */

  /* Side wall center reference (L0/R0 raw average when centered between walls) */
  uint16_t center_l0; /* L0 raw when centered (wall-follow setpoint) */
  uint16_t center_r0; /* R0 raw when centered (wall-follow setpoint) */

  /* Min/Max from wall-proximity calibration (Mode 2+) */
  uint16_t l0_min, l0_max;   /* L0 range */
  uint16_t l45_min, l45_max; /* L45 range */
  uint16_t r0_min, r0_max;   /* R0 range */
  uint16_t r45_min, r45_max; /* R45 range */

  /* Metal post calibration (Mode 3 only) */
  uint16_t l90_post;  /* L90 value when close to metal post (stronger than wall) */
  uint16_t r90_post;  /* R90 value when close to metal post (stronger than wall) */

  /* No-wall baseline: A = max(NO_WALL) for each sensor */
  uint16_t open_l90;  /* L90 ADC when no front wall */
  uint16_t open_r90;  /* R90 ADC when no front wall */
  uint16_t open_l0;   /* L0 ADC when no side wall */
  uint16_t open_r0;   /* R0 ADC when no side wall */
  uint16_t open_l45;  /* L45 ADC when no side wall */
  uint16_t open_r45;  /* R45 ADC when no side wall */

  /* Hysteresis thresholds: WALL_ON / WALL_OFF for each sensor */
  uint16_t wall_on_l0, wall_off_l0;     /* Left wall (L0) */
  uint16_t wall_on_r0, wall_off_r0;     /* Right wall (R0) */
  uint16_t wall_on_l90, wall_off_l90;   /* Front-left (L90) */
  uint16_t wall_on_r90, wall_off_r90;   /* Front-right (R90) */
  uint16_t wall_on_l45, wall_off_l45;   /* Side event L (L45) */
  uint16_t wall_on_r45, wall_off_r45;   /* Side event R (R45) */

  /* Side L45/R45 full-wall peak threshold (replaces SIDE_WALL_LEVEL_THRESH) */
  uint16_t side_level_l45;  /* Peak > this → full wall, else post */
  uint16_t side_level_r45;

  /* ====== Task-based calibration (Phase 1) ====== */

  /* Gain balance: equalize L/R sensor pairs so L_eq ≈ R_eq at same geometry */
  float gain_l0, gain_r0;     /* L0/R0 pair (side wall sensors) */
  float gain_l45, gain_r45;   /* L45/R45 pair (diagonal/event sensors) */
  float gain_l90, gain_r90;   /* L90/R90 pair (front sensors) */

  /* Front alignment: angular error from front sensor pair */
  int16_t front_diff_zero;    /* mean(L90_eq - R90_eq) when perpendicular to wall */

  /* Event derivative thresholds for L45/R45 edge detection */
  int16_t l45_drop_th;        /* dL45 < this → wall disappeared (negative value) */
  int16_t l45_rise_th;        /* dL45 > this → wall appeared   (positive value) */
  int16_t r45_drop_th;
  int16_t r45_rise_th;

  /* Front wall interference gating: when L0+R0 sum exceeds this,
   * side sensors are unreliable due to front wall cross-reflection */
  uint16_t front_interference_th;

  uint8_t has_front_diff;  /* 1 if front-diff calibration was performed */

  uint8_t calib_mode;    /* 1=basic, 2=with min/max, 3=full+post */
  uint8_t is_calibrated;
  uint8_t has_open_calib; /* 1 if open-space (A values) calibration done */
} IR_Simple_Calib_t;

/* Initialization */
void IR_Simple_Init(IR_Simple_Calib_t *calib);

/* Mode 1: spatial sampling (robot moves forward during calib) */
void IR_Simple_Calib90and0(IR_Simple_Calib_t *calib, struct Motion_Controller_t *motion_ctrl);
void IR_Simple_Calib45(IR_Simple_Calib_t *calib, struct Motion_Controller_t *motion_ctrl);

/* Open-space sampling: static read with NO walls nearby → A values */
void IR_Simple_CalibOpenSpace(IR_Simple_Calib_t *calib);

/* Front-diff calibration: robot facing front wall perpendicularly.
 * Computes front_diff_zero for angular alignment correction. */
void IR_Simple_CalibFrontDiff(IR_Simple_Calib_t *calib);

/* Compute hysteresis thresholds from A (open) and B (center) values */
void IR_Simple_ComputeHysteresis(IR_Simple_Calib_t *calib);

/* Apply gain equalization to a raw sensor array in-place.
 * Input/output: values[6] = {L90, L45, L0, R0, R45, R90} */
void IR_Simple_ApplyGain(IR_Simple_Calib_t *calib, uint16_t *values);

/* Mode 2+: wall-proximity static sampling */
void IR_Simple_CalibLeftWall(IR_Simple_Calib_t *calib);
void IR_Simple_CalibRightWall(IR_Simple_Calib_t *calib);
void IR_Simple_RefineThresholds(IR_Simple_Calib_t *calib);

/* Mode 3: metal post calibration */
void IR_Simple_CalibLeftPost(IR_Simple_Calib_t *calib);
void IR_Simple_CalibRightPost(IR_Simple_Calib_t *calib);

void IR_Simple_PrintCalib(IR_Simple_Calib_t *calib);

/* Wall detection (single-threshold, for cell-by-cell stopped reads) */
uint8_t IR_Simple_DetectLeftWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw);
uint8_t IR_Simple_DetectFrontWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw);
uint8_t IR_Simple_DetectRightWall(IR_Simple_Calib_t *calib, uint16_t *ir_raw);

/* Hysteresis wall detection (for continuous reads while moving) */
uint8_t IR_Simple_DetectLeftWall_Hyst(IR_Simple_Calib_t *calib, uint16_t raw_l0,
                                      uint8_t *state);
uint8_t IR_Simple_DetectFrontWall_Hyst(IR_Simple_Calib_t *calib,
                                       uint16_t raw_l90, uint16_t raw_r90,
                                       uint8_t *state);
uint8_t IR_Simple_DetectRightWall_Hyst(IR_Simple_Calib_t *calib, uint16_t raw_r0,
                                       uint8_t *state);

#endif /* IR_SIMPLE_CALIB_H */
