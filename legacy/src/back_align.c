#include "back_align.h"
#include "hardware.h"     // For Hardware_SetMotor, Hardware_ResetEncoder, etc.
#include "system_timer.h" // For millis()
#include "uart.h"         // For debugging/logging
#include <stdio.h>
#include <stdlib.h> // For abs()

/* Stall detection: if encoders move less than this in STALL_CHECK_INTERVAL_MS,
 * robot is pressed against the wall */
#define STALL_CHECK_INTERVAL_MS 200
#define STALL_ENCODER_THRESHOLD                                                \
  5 /* encoder counts — if both wheels move less                             \
       than this, consider stalled */

/* Reverse PWM duty (negative = backward). int8_t range: -100 to 100 */
#define REVERSE_PWM -40

/* Determine if current pre-turn wall state supports reversing */
uint8_t BackAlign_IsEligible(uint8_t turn_cmd, const TurnContext_t *ctx) {
  if (turn_cmd == 'R' && ctx->left_wall_before_turn)
    return 1;
  if (turn_cmd == 'L' && ctx->right_wall_before_turn)
    return 1;
  if (turn_cmd == 'B' && ctx->front_wall_before_turn)
    return 1;
  return 0;
}

/* Initialize the primitive and fire off the initial turn motion */
void BackAlign_Start(BackAlign_t *ba, uint8_t turn_cmd, uint8_t new_dir,
                     Motion_Controller_t *motion) {
  ba->turn_type = turn_cmd;
  ba->new_dir = new_dir;

  // Map command to geometric angle
  if (turn_cmd == 'R') {
    ba->turn_angle = -90.0f;
  } else if (turn_cmd == 'L') {
    ba->turn_angle = 90.0f;
  } else if (turn_cmd == 'B') {
    ba->turn_angle = 180.0f;
  } else {
    ba->turn_angle = 0.0f;
  }

  // Execute Stage 1: Absolute Turn
  if (ba->turn_angle != 0.0f) {
    UART_SendString("[BA] Start: Turning\r\n");
    Motion_Turn(motion, ba->turn_angle);
    ba->state = BACK_ALIGN_TURNING;
  } else {
    // Fallback for unexpected case
    ba->state = BACK_ALIGN_FAIL;
  }

  ba->state_start_time = millis();
  ba->last_enc_left = 0;
  ba->last_enc_right = 0;
  ba->last_stall_check_time = 0;
}

/* State machine pump */
void BackAlign_Update(BackAlign_t *ba, Motion_Controller_t *motion) {
  char buf[64];

  switch (ba->state) {
  case BACK_ALIGN_TURNING:
    if (Motion_IsComplete(motion)) {
      UART_SendString("[BA] Turn complete, settling...\r\n");
      ba->state_start_time = millis();
      ba->state = BACK_ALIGN_SETTLE_AFTER_TURN;
    }
    break;

  case BACK_ALIGN_SETTLE_AFTER_TURN:
    if ((millis() - ba->state_start_time) >= BACK_ALIGN_SETTLE_MS) {
      UART_SendString("[BA] Settled, reversing to anchor...\r\n");

      /* Stop motion controller so it doesn't interfere */
      Motion_Stop(motion);

      /* Reset encoders for stall detection */
      Hardware_ResetEncoder(0);
      Hardware_ResetEncoder(1);

      /* Drive motors in reverse directly */
      Hardware_SetMotor(0, REVERSE_PWM); /* Right motor backward */
      Hardware_SetMotor(1, REVERSE_PWM); /* Left motor backward */

      ba->state_start_time = millis();
      ba->last_stall_check_time = millis();
      ba->last_enc_left = 0;
      ba->last_enc_right = 0;
      ba->state = BACK_ALIGN_REVERSE_TO_ANCHOR;
    }
    break;

  case BACK_ALIGN_REVERSE_TO_ANCHOR: {
    uint32_t now = millis();

    /* Hard timeout: stop no matter what */
    if ((now - ba->state_start_time) >= BACK_ALIGN_REVERSE_TIMEOUT) {
      Hardware_StopMotors();
      UART_SendString("[BA] Anchor Reached (Timeout)\r\n");
      ba->state = BACK_ALIGN_ANCHOR_REACHED;
      break;
    }

    /* Stall detection: check encoder movement periodically */
    if ((now - ba->last_stall_check_time) >= STALL_CHECK_INTERVAL_MS) {
      int32_t enc_l = Hardware_GetEncoderCount(1);
      int32_t enc_r = Hardware_GetEncoderCount(0);
      int32_t delta_l = abs(enc_l - ba->last_enc_left);
      int32_t delta_r = abs(enc_r - ba->last_enc_right);

      sprintf(buf, "[BA] Stall: dL=%ld dR=%ld\r\n", delta_l, delta_r);
      UART_SendString(buf);

      if (delta_l < STALL_ENCODER_THRESHOLD &&
          delta_r < STALL_ENCODER_THRESHOLD) {
        /* Both wheels stalled — we hit the wall */
        Hardware_StopMotors();
        UART_SendString("[BA] Anchor Reached (Stall detected)\r\n");
        ba->state = BACK_ALIGN_ANCHOR_REACHED;
        break;
      }

      ba->last_enc_left = enc_l;
      ba->last_enc_right = enc_r;
      ba->last_stall_check_time = now;
    }
  } break;

  case BACK_ALIGN_ANCHOR_REACHED:
    UART_SendString("[BA] Resetting pose...\r\n");

    /* Snap heading to nearest 90° cardinal direction */
    Motion_SnapHeading();

    /* Reset encoders for the forward recovery */
    Hardware_ResetEncoder(0);
    Hardware_ResetEncoder(1);

    ba->state = BACK_ALIGN_POSE_RESET;
    break;

  case BACK_ALIGN_POSE_RESET:
    sprintf(buf, "[BA] Recovering %.1f mm\r\n", BACK_ALIGN_RECOVER_MM);
    UART_SendString(buf);

    /* Move forward precisely to canonical cell center */
    Motion_Straight(motion, BACK_ALIGN_RECOVER_MM);
    ba->state = BACK_ALIGN_FORWARD_RECOVER;
    break;

  case BACK_ALIGN_FORWARD_RECOVER:
    if (Motion_IsComplete(motion)) {
      UART_SendString("[BA] Done! At canonical pose.\r\n");
      ba->state = BACK_ALIGN_DONE;
    }
    break;

  case BACK_ALIGN_IDLE:
  case BACK_ALIGN_DONE:
  case BACK_ALIGN_FAIL:
    // Terminal or uninitialized states do nothing
    break;
  }
}

uint8_t BackAlign_IsDone(const BackAlign_t *ba) {
  return (ba->state == BACK_ALIGN_DONE || ba->state == BACK_ALIGN_FAIL) ? 1 : 0;
}
