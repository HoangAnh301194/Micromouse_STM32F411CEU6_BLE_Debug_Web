#ifndef ENCODER_ONEBOX_H
#define ENCODER_ONEBOX_H
#include <stdint.h>
#include "encoder_onebox_core.h"
#include "mpu6050.h"

#if defined(MODE_MENU_ONLY)
typedef struct {
    uint8_t reason;
    int32_t count_l, count_r;
    uint32_t elapsed_ms;
    uint8_t motor_locked;
    int16_t pivot_target;
} EncoderOneBoxResult;

/* Real motor telemetry: absolute signed quadrature counts (same as
 * Log Receive) and actual APPLIED motor PWM in permille (-1000..1000).
 * elapsed_ms is measured since the last accepted Start. No SPP I/O in ISR. */
typedef struct {
    uint32_t elapsed_ms;
    int32_t left, right;
    int16_t pwm_l, pwm_r;
    int32_t ref_l, ref_r, speed_l, speed_r;
    int32_t delta_l, delta_r;
    uint8_t motion_stage;
    int16_t pivot_target;
    float yaw, angle, gz;
    uint32_t age_us;
    uint8_t imu_valid;
} EncoderOneBoxTelemetry;

uint8_t EncoderOneBox_GetTelemetry(EncoderOneBoxTelemetry *out);
uint8_t EncoderOneBox_SetGains(const OneBoxGains *left, const OneBoxGains *right);
void EncoderOneBox_GetGains(OneBoxGains *left, OneBoxGains *right);
void EncoderOneBox_Init(void);
uint8_t EncoderOneBox_StartPivot(MPU6050_Handle_t *mpu, int16_t target);
/* Calibration-only slower pivot; shares all safety guards with ordinary pivot. */
uint8_t EncoderOneBox_StartCalibrationPivot(MPU6050_Handle_t *mpu, int16_t target);
uint8_t EncoderOneBox_Start(void); /* SPP 'G' only, after menu 1 Box Forward */
void EncoderOneBox_HostPing(void);  /* SPP 'H' <=100ms, 350ms safety lease */
void EncoderOneBox_Abort(void);     /* SPP 'P' or menu navigation */
void EncoderOneBox_ControlTick(void); /* called from 1kHz TIM11 */
uint8_t EncoderOneBox_IsRunning(void);
/* After physical PA0 emergency stop, consume the press/release instead
 * of also treating it as a menu Next/Select event. */
uint8_t EncoderOneBox_SuppressMenuButton(void);
uint8_t EncoderOneBox_PopResult(EncoderOneBoxResult *out);
const char *EncoderOneBox_Reason(uint8_t reason);
#endif
#endif
