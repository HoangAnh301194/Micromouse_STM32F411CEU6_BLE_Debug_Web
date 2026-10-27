#include "motion.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "pinout.h"
#include "stm32f4xx.h"

#include <math.h>

#define PI_F                    3.14159265f
#define MM_PER_PULSE            (PI_F * WHEEL_DIAMETER_MM / (float)ENCODER_PPR)
#define STRAIGHT_PWM_DEFAULT    35.0f
#define STRAIGHT_PWM_MIN        15.0f
#define STRAIGHT_RAMP_PWM_S     120.0f
#define TURN_PWM_MAX            50.0f
#define TURN_PWM_MIN            20.0f
#define TURN_TOLERANCE_DEG      3.0f
#define TURN_RATE_TOL_DPS       10.0f
#define TURN_TIMEOUT_S          1.5f

typedef struct {
    volatile Motion_State_t state;
    volatile uint8_t done;

    float target_distance_mm;
    float target_angle_deg;
    float start_yaw_deg;

    volatile float distance_mm;
    volatile float angle_error_deg;

    float current_pwm;
    float elapsed_s;

    PID_Handle_t heading_pid;
    PID_Handle_t turn_pid;
} Motion_Context_t;

static Motion_Context_t motion;
/* Separate reusable primitive controller; never shares live state with
 * the legacy heading and turn PIDs. */
static MotionPrimitiveController pivot;

static float NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float ClampF(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int16_t ClampPwm(float value)
{
    if (value > 100.0f) return 100;
    if (value < -100.0f) return -100;
    return (int16_t)value;
}

static void FinishMotion(void)
{
    motion.state = MOTION_IDLE;
    motion.done = 1U;
    motion.current_pwm = 0.0f;
    Motor_Stop();
}

void Motion_Init(void)
{
    MotionPrimitive_Init(&pivot);
    motion.state = MOTION_IDLE;
    motion.done = 1U;
    motion.target_distance_mm = 0.0f;
    motion.target_angle_deg = 0.0f;
    motion.start_yaw_deg = 0.0f;
    motion.distance_mm = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.current_pwm = 0.0f;
    motion.elapsed_s = 0.0f;

    /* Initial values copied from the proven controller as a conservative baseline. */
    PID_Init(&motion.heading_pid, 2.0f, 0.0f, 0.01f, 35.0f, 40.0f);
    PID_Init(&motion.turn_pid, 1.1f, 0.01f, 0.001f, TURN_PWM_MAX, 50.0f);

    Motor_Stop();
}

void Motion_CommandStraight(float distance_mm, float current_yaw_deg)
{
    uint32_t primask;

    if (distance_mm <= 0.0f) return;

    primask = __get_PRIMASK();
    __disable_irq();

    Encoder_Reset();
    PID_Reset(&motion.heading_pid);

    motion.target_distance_mm = distance_mm;
    motion.start_yaw_deg = current_yaw_deg;
    motion.distance_mm = 0.0f;
    motion.angle_error_deg = 0.0f;
    motion.current_pwm = STRAIGHT_PWM_MIN;
    motion.elapsed_s = 0.0f;

    /* Publish state last so the ISR never sees a half-initialized command. */
    motion.done = 0U;
    motion.state = MOTION_STRAIGHT;

    if (!primask) __enable_irq();
}

void Motion_CommandTurn(float angle_deg, float current_yaw_deg)
{
    uint32_t primask;

    if (fabsf(angle_deg) < 1.0f) return;

    primask = __get_PRIMASK();
    __disable_irq();

    Encoder_Reset();
    PID_Reset(&motion.turn_pid);

    motion.target_angle_deg = angle_deg;
    motion.start_yaw_deg = current_yaw_deg;
    motion.angle_error_deg = angle_deg;
    motion.distance_mm = 0.0f;
    motion.current_pwm = 0.0f;
    motion.elapsed_s = 0.0f;

    motion.done = 0U;
    motion.state = MOTION_TURN;

    if (!primask) __enable_irq();
}

MotionPrimitiveResult Motion_RequestPrimitive(MotionPrimitiveKind kind,
                                               MotionPrimitiveDirection direction,
                                               float current_yaw_deg)
{
    uint32_t primask = __get_PRIMASK();
    MotionPrimitiveResult result;
    __disable_irq();

    /* Reject unsupported motion / concurrent motion before touching
     * encoders, PID, motor output, or changing state. */
    if (!motion.done || motion.state != MOTION_IDLE ||
        Motor_OutputsLocked()) {
        if (!primask) __enable_irq();
        return MOTION_PRIMITIVE_BUSY;
    }
    result = MotionPrimitive_Start(&pivot, kind, direction, current_yaw_deg);
    if (result == MOTION_PRIMITIVE_OK) {
        Encoder_Reset();
        motion.target_angle_deg = (float)direction *
                                  (kind==MOTION_PRIMITIVE_PIVOT_180?180.0f:90.0f);
        motion.start_yaw_deg = current_yaw_deg;
        motion.angle_error_deg = motion.target_angle_deg;
        motion.distance_mm = 0.0f;
        motion.current_pwm = 0.0f;
        motion.elapsed_s = 0.0f;
        motion.done = 0U;
        motion.state = MOTION_PIVOT_PRIMITIVE; /* publish last */
    }
    if (!primask) __enable_irq();
    return result;
}

MotionPrimitiveResult Motion_Turn90(MotionPrimitiveDirection direction,
                                     float current_yaw_deg)
{
    return Motion_RequestPrimitive(MOTION_PRIMITIVE_PIVOT_90,
                                   direction,current_yaw_deg);
}
MotionPrimitiveResult Motion_Turn180(MotionPrimitiveDirection direction,
                                      float current_yaw_deg)
{
    return Motion_RequestPrimitive(MOTION_PRIMITIVE_PIVOT_180,
                                   direction,current_yaw_deg);
}
MotionPrimitiveStatus Motion_GetPrimitiveStatus(void)
{
    return pivot.status;
}
MotionPrimitiveResult Motion_GetPrimitiveResult(void)
{
    return pivot.result;
}
MotionPivotPhase Motion_GetPrimitivePhase(void)
{
    return pivot.phase;
}

void Motion_Stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Invalidate the command before touching PWM to close the stop/restart race. */
    if (motion.state == MOTION_PIVOT_PRIMITIVE)
        MotionPrimitive_Abort(&pivot,MOTION_PRIMITIVE_STOP_REQUEST);
    motion.done = 1U;
    motion.state = MOTION_IDLE;
    motion.current_pwm = 0.0f;
    Motor_Stop();

    if (!primask) __enable_irq();
}

void Motion_Update(float dt_s,
                   int32_t encoder_left,
                   int32_t encoder_right,
                   float yaw_deg,
                   float gyro_z_dps)
{
    float left_mm;
    float right_mm;

    if (motion.done || motion.state == MOTION_IDLE) return;
    if (dt_s <= 0.0f || dt_s > 0.02f) return;

    motion.elapsed_s += dt_s;

    left_mm = (float)encoder_left * MM_PER_PULSE;
    right_mm = (float)encoder_right * MM_PER_PULSE;
    motion.distance_mm = 0.5f * (left_mm + right_mm);

    if (motion.state == MOTION_PIVOT_PRIMITIVE) {
        MotionPrimitiveOutput out;
        MotionPrimitive_Update(&pivot,dt_s,yaw_deg,gyro_z_dps,&out);
        motion.angle_error_deg = (float)pivot.direction *
                                 out.angle_error_deg;
        motion.current_pwm = (float)out.right_pm * 0.1f;
        if (out.status != MOTION_PRIMITIVE_RUNNING) {
            FinishMotion(); /* DONE and FAULT both force output neutral. */
            return;
        }
        if (out.immediate_neutral) Motor_Stop();
        else Motor_SetPairPermille(out.left_pm,out.right_pm);
        return;
    }

    if (motion.state == MOTION_STRAIGHT) {
        float heading_error;
        float correction;
        float left_pwm;
        float right_pwm;

        if (motion.distance_mm >= motion.target_distance_mm) {
            FinishMotion();
            return;
        }

        motion.current_pwm += STRAIGHT_RAMP_PWM_S * dt_s;
        if (motion.current_pwm > STRAIGHT_PWM_DEFAULT) {
            motion.current_pwm = STRAIGHT_PWM_DEFAULT;
        }

        heading_error = NormalizeAngle(yaw_deg - motion.start_yaw_deg);
        motion.angle_error_deg = heading_error;

        /* PID_Compute uses setpoint-input. Passing error as setpoint preserves
           the sign convention used by the previous controller. */
        correction = PID_Compute(&motion.heading_pid, heading_error, 0.0f, dt_s);

        left_pwm = ClampF(motion.current_pwm + correction, 0.0f, 100.0f);
        right_pwm = ClampF(motion.current_pwm - correction, 0.0f, 100.0f);

        Motor_SetPair(ClampPwm(left_pwm), ClampPwm(right_pwm));
        return;
    }

    if (motion.state == MOTION_TURN) {
        float traveled = NormalizeAngle(yaw_deg - motion.start_yaw_deg);
        float error = NormalizeAngle(motion.target_angle_deg - traveled);
        float output;
        float magnitude;

        motion.angle_error_deg = error;

        if ((fabsf(error) <= TURN_TOLERANCE_DEG &&
             fabsf(gyro_z_dps) <= TURN_RATE_TOL_DPS) ||
            motion.elapsed_s >= TURN_TIMEOUT_S) {
            FinishMotion();
            return;
        }

        output = PID_Compute(&motion.turn_pid, error, 0.0f, dt_s);
        magnitude = fabsf(output);
        magnitude = ClampF(magnitude, TURN_PWM_MIN, TURN_PWM_MAX);
        output = (output < 0.0f) ? -magnitude : magnitude;

        Motor_SetPair(ClampPwm(-output), ClampPwm(output));
    }
}

uint8_t Motion_IsDone(void) { return motion.done; }
Motion_State_t Motion_GetState(void) { return motion.state; }
float Motion_GetDistanceMm(void) { return motion.distance_mm; }
float Motion_GetAngleErrorDeg(void) { return motion.angle_error_deg; }
