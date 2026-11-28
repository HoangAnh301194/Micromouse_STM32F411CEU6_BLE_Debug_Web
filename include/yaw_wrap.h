#ifndef YAW_WRAP_H
#define YAW_WRAP_H
/* Signed remainder preserves the existing positive/negative rotation convention. */
static inline float Yaw_Wrap360(float angle)
{
    /* Normal integration crosses at most one revolution per update. */
    if (angle >= 360.0f || angle <= -360.0f) {
        angle = __builtin_fmodf(angle, 360.0f);
    }
    return angle == 0.0f ? 0.0f : angle;
}
/* Consecutive samples must differ physically by less than 180 degrees. */
static inline float Yaw_SampleDelta(float now, float previous)
{
    float d = Yaw_Wrap360(now-previous);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}
#endif
