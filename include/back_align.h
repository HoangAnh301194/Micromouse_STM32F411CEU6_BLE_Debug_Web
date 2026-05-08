#ifndef BACK_ALIGN_H
#define BACK_ALIGN_H

#include "motion_controller.h"
#include <stdint.h>

/*
 * 1. Geometric Constants for Canonical Pose
 *    CELL_SIZE_MM is defined in motion_controller.h (included above)
 */
#define REAR_TO_POSE_REF_MM                                                    \
  45.0f /* Distance from rear reference plane to wheel axle (pose zero) */
#define BACK_ALIGN_RECOVER_MM                                                  \
  ((CELL_SIZE_MM * 0.5f) - REAR_TO_POSE_REF_MM - 10.0f)

#define BACK_ALIGN_REVERSE_SPEED 200.0f /* Speed to reverse into the wall */
#define BACK_ALIGN_REVERSE_TIMEOUT                                             \
  500 /* Max ms to allow for reverse bump before assuming failure */
#define BACK_ALIGN_SETTLE_MS 100 /* Settle time after turn before reversing */

/*
 * 2. Pre-Turn Context
 * Holds wall data snapshotted before the turn begins to determine eligibility.
 */
typedef struct {
  uint8_t left_wall_before_turn;
  uint8_t front_wall_before_turn;
  uint8_t right_wall_before_turn;
} TurnContext_t;

/*
 * 3. Primitive States
 */
typedef enum {
  BACK_ALIGN_IDLE,
  BACK_ALIGN_TURNING,
  BACK_ALIGN_SETTLE_AFTER_TURN,
  BACK_ALIGN_REVERSE_TO_ANCHOR,
  BACK_ALIGN_ANCHOR_REACHED,
  BACK_ALIGN_POSE_RESET,
  BACK_ALIGN_FORWARD_RECOVER,
  BACK_ALIGN_DONE,
  BACK_ALIGN_FAIL
} BackAlignState_t;

/*
 * 4. Primitive Context Object
 */
typedef struct {
  BackAlignState_t state;

  /* Input parameters */
  uint8_t turn_type; /* 'L', 'R', or 'B' */
  float turn_angle;  /* Derived from turn_type */
  uint8_t new_dir;   /* The intended facing direction (0=N, 1=E, 2=S, 3=W) after
                        the maneuver */

  /* Timing data */
  uint32_t state_start_time;

  /* Stall detection for reverse phase */
  int32_t last_enc_left;
  int32_t last_enc_right;
  uint32_t last_stall_check_time;

} BackAlign_t;

/*
 * 5. Public API
 */

/**
 * @brief Evaluates if the current situation allows for a back-alignment
 * primitive.
 * @param turn_cmd 'L', 'R', or 'B'
 * @param ctx Snapshot of walls BEFORE turning
 * @return 1 if eligible, 0 otherwise
 */
uint8_t BackAlign_IsEligible(uint8_t turn_cmd, const TurnContext_t *ctx);

/**
 * @brief Starts the macro-maneuver.
 * @param ba Pointer to the runtime context
 * @param turn_cmd 'L', 'R', or 'B'
 * @param new_dir The ordinal direction (0..3) the robot will face AFTER the
 * turn
 * @param motion Pointer to the motion controller to send commands
 */
void BackAlign_Start(BackAlign_t *ba, uint8_t turn_cmd, uint8_t new_dir,
                     Motion_Controller_t *motion);

/**
 * @brief Pumps the state machine. Should be called repeatedly during
 * EXPLORE_BACK_ALIGN state.
 * @param ba Pointer to the runtime context
 * @param motion Pointer to the motion controller
 */
void BackAlign_Update(BackAlign_t *ba, Motion_Controller_t *motion);

/**
 * @brief Checks if the canonical pose recovery has successfully completed.
 * @return 1 if done, 0 otherwise
 */
uint8_t BackAlign_IsDone(const BackAlign_t *ba);

#endif // BACK_ALIGN_H
