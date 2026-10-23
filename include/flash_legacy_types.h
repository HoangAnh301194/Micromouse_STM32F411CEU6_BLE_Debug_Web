#ifndef FLASH_LEGACY_TYPES_H
#define FLASH_LEGACY_TYPES_H

#include <stdint.h>

/*
 * Exact field layouts imported from main for reading the old on-flash
 * calibration and maze records. These are DATA TYPES ONLY; maze solving and
 * calibration algorithms have not been brought back into this branch.
 *
 * main's persistent map used a 5x5 grid (not the web UI's 16x16 grid).
 * Do not change these definitions in place: doing so changes the legacy
 * storage format without a migration.
 */
#define FLASH_LEGACY_MAZE_SIZE 5U
#define MAZE_PERSISTENT_MAGIC 0xA5U

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

typedef struct {
  uint8_t walls[FLASH_LEGACY_MAZE_SIZE][FLASH_LEGACY_MAZE_SIZE];      /* Accumulated wall knowledge */
  uint8_t confidence[FLASH_LEGACY_MAZE_SIZE][FLASH_LEGACY_MAZE_SIZE]; /* Bitfield: observed directions */
  uint8_t visit_count[FLASH_LEGACY_MAZE_SIZE]
                     [FLASH_LEGACY_MAZE_SIZE]; /* Times robot passed through cell */

  uint8_t total_runs;           /* How many explore runs completed */
  uint8_t center_reached_count; /* How many times center was reached */
  uint8_t is_valid;             /* MAZE_PERSISTENT_MAGIC = data valid */
  uint8_t fully_explored;       /* 1 = all reachable cells observed */
} Maze_Persistent_t;

#endif /* FLASH_LEGACY_TYPES_H */
