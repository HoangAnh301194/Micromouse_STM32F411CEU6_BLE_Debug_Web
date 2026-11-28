#if defined(MODE_MENU_ONLY)
#include "ir_calibration_360.h"
#include "control.h"
#include "ir_sensor.h"
#include "motor.h"
#include "system_timer.h"
#include "bt_debug.h"
#include "flash_storage.h"
#include <string.h>

#define IRC_SAMPLE_PERIOD_MS       20U
#define IRC_REQUIRED_SAMPLES       150U
#define IRC_CAPTURE_TIMEOUT_MS     20000UL
#define IRC_OPEN_TRANSITION_MS     4000UL
#define IRC_MIN_SEPARATION_ADC     96U
#define IRC_HIST_BINS              64U
#define IRC_HIST_BIN_SHIFT         6U
#define IRC_SENSOR_MAX_AGE_US      5000UL

static const char *IRC_SensorName(uint8_t i)
{
    static const char *names[6] = {"L0", "L45", "L90", "R0", "R45", "R90"};
    return (i < 6U) ? names[i] : "?";
}

static uint16_t IRC_Quantile(const uint16_t hist[IRC_HIST_BINS],
                             uint32_t count, uint32_t percentile)
{
    uint32_t goal = (count * percentile + 99U) / 100U;
    uint32_t seen = 0U;
    uint8_t bin;
    if (count == 0U) return 0U;
    for (bin = 0U; bin < IRC_HIST_BINS; ++bin) {
        seen += hist[bin];
        if (seen >= goal) {
            uint32_t value = ((uint32_t)bin << IRC_HIST_BIN_SHIFT) + 32U;
            return (uint16_t)(value > 4095U ? 4095U : value);
        }
    }
    return 4095U;
}

static void IRC_ResetStage(IRC_Context *c, uint8_t open_stage, uint32_t now)
{
    uint8_t i;
    c->stage_started_ms = now;
    c->last_sample_ms = now - IRC_SAMPLE_PERIOD_MS;
    c->last_scan_count = 0U;
    if (open_stage) {
        c->open_samples = 0U;
        memset(c->open_histogram, 0, sizeof(c->open_histogram));
        for (i = 0U; i < 6U; ++i) {
            c->open_min[i] = 4095U;
            c->open_max[i] = 0U;
        }
    } else {
        c->wall_samples = 0U;
        memset(c->wall_histogram, 0, sizeof(c->wall_histogram));
        for (i = 0U; i < 6U; ++i) {
            c->wall_min[i] = 4095U;
            c->wall_max[i] = 0U;
        }
    }
}

void IRC_Init(IRC_Context *c)
{
    if (c) memset(c, 0, sizeof(*c));
}

uint8_t IRC_IsMotionActive(const IRC_Context *c)
{
    return c && (c->state == IRC_CAPTURE_WALL ||
                 c->state == IRC_WAIT_OPEN ||
                 c->state == IRC_CAPTURE_OPEN);
}

void IRC_Arm(IRC_Context *c, uint32_t now, uint8_t physical_start)
{
    if (!c || IRC_IsMotionActive(c)) return;
    IRC_Init(c);
    c->state = IRC_CAPTURE_WALL;
    c->started_ms = now;
    c->physical_start = physical_start ? 1U : 0U;
    IRC_ResetStage(c, 0U, now);
    BT_SendString("CAL,START,mode=1,stages=2,samples=150,save=AUTO_IF_VALID\r\n");
    BT_SendString("CAL,STAGE,WALL,setup=FRONT_LEFT_RIGHT,no_rear=1,action=KEEP_STILL\r\n");
}

void IRC_Abort(IRC_Context *c, const char *why)
{
    if (!c || !IRC_IsMotionActive(c)) return;
    c->state = IRC_FAULT;
    strncpy(c->fault, why ? why : "ABORT", sizeof(c->fault) - 1U);
    c->fault[sizeof(c->fault) - 1U] = 0;
    BT_Printf("CAL,FAULT,reason=%s\r\n", c->fault);
    BT_Printf("CAL,END,result=FAILED,reason=%s\r\n", c->fault);
}

uint8_t IRC_ConfirmOpen(IRC_Context *c, uint32_t now)
{
    if (!c || c->state != IRC_WAIT_OPEN) {
        BT_SendString("CAL,ADVANCE_REJECT,expected=WAIT_OPEN\r\n");
        return 0U;
    }
    IRC_ResetStage(c, 1U, now);
    c->state = IRC_CAPTURE_OPEN;
    BT_SendString("CAL,STAGE,OPEN,setup=NO_WALLS,action=KEEP_STILL\r\n");
    return 1U;
}

static uint8_t IRC_ReadSample(IRC_Context *c, uint32_t now)
{
    IR_Data_t frame;
    uint32_t age;
    uint16_t values[6];
    uint16_t (*hist)[IRC_HIST_BINS];
    uint16_t *minimum, *maximum;
    uint32_t *sample_count;
    uint8_t i, open_stage = (c->state == IRC_CAPTURE_OPEN) ? 1U : 0U;

    if ((uint32_t)(now - c->last_sample_ms) < IRC_SAMPLE_PERIOD_MS) return 0U;
    Control_GetIRSnapshot(&frame);
    age = frame.scan_count ? (uint32_t)(micros() - frame.completed_us) : 0xFFFFFFFFUL;
    if (age > IRC_SENSOR_MAX_AGE_US) return 0U;
    if (frame.scan_count == c->last_scan_count) return 0U;

    c->last_sample_ms = now;
    c->last_scan_count = frame.scan_count;
    values[0] = frame.value[IR_L0];
    values[1] = frame.value[IR_L45];
    values[2] = frame.value[IR_L90];
    values[3] = frame.value[IR_R0];
    values[4] = frame.value[IR_R45];
    values[5] = frame.value[IR_R90];

    if (open_stage) {
        hist = c->open_histogram;
        minimum = c->open_min;
        maximum = c->open_max;
        sample_count = &c->open_samples;
    } else {
        hist = c->wall_histogram;
        minimum = c->wall_min;
        maximum = c->wall_max;
        sample_count = &c->wall_samples;
    }

    for (i = 0U; i < 6U; ++i) {
        uint8_t bin = (uint8_t)(values[i] >> IRC_HIST_BIN_SHIFT);
        if (values[i] < minimum[i]) minimum[i] = values[i];
        if (values[i] > maximum[i]) maximum[i] = values[i];
        if (hist[i][bin] < 65535U) ++hist[i][bin];
    }
    ++(*sample_count);

    BT_Printf("CAL_IR,stage=%u,ms=%lu,L0=%u,L45=%u,L90=%u,R0=%u,R45=%u,R90=%u\r\n",
        (unsigned)open_stage, (unsigned long)(now - c->started_ms),
        (unsigned)values[0], (unsigned)values[1], (unsigned)values[2],
        (unsigned)values[3], (unsigned)values[4], (unsigned)values[5]);
    return 1U;
}

static uint8_t IRC_ComputeThresholds(IRC_Context *c)
{
    uint8_t i;
    c->valid = 1U;
    c->failed_channels = 0U;

    for (i = 0U; i < 6U; ++i) {
        uint16_t wall_p10 = IRC_Quantile(c->wall_histogram[i], c->wall_samples, 10U);
        uint16_t wall_p90 = IRC_Quantile(c->wall_histogram[i], c->wall_samples, 90U);
        uint16_t open_p90 = IRC_Quantile(c->open_histogram[i], c->open_samples, 90U);
        uint16_t open_p95 = IRC_Quantile(c->open_histogram[i], c->open_samples, 95U);
        uint16_t gap = (wall_p10 > open_p90) ? (uint16_t)(wall_p10 - open_p90) : 0U;

        c->wall_p10[i] = wall_p10;
        c->wall_p90[i] = wall_p90;
        c->open_p90[i] = open_p90;
        c->open_p95[i] = open_p95;

        if (gap < IRC_MIN_SEPARATION_ADC) {
            c->wall_off[i] = 0U;
            c->wall_on[i] = 0U;
            c->valid = 0U;
            ++c->failed_channels;
        } else {
            c->wall_off[i] = (uint16_t)(open_p90 + gap / 3U);
            c->wall_on[i] = (uint16_t)(open_p90 + (2U * (uint32_t)gap) / 3U);
            if (c->wall_on[i] <= c->wall_off[i]) {
                c->valid = 0U;
                ++c->failed_channels;
            }
        }

        BT_Printf("CAL,THRESH,index=%u,name=%s,open_p95=%u,open_p90=%u,wall_p10=%u,wall_p90=%u,gap=%u,off=%u,on=%u\r\n",
            (unsigned)i, IRC_SensorName(i), (unsigned)open_p95,
            (unsigned)open_p90, (unsigned)wall_p10, (unsigned)wall_p90,
            (unsigned)gap, (unsigned)c->wall_off[i], (unsigned)c->wall_on[i]);
    }
    return c->valid;
}

static void IRC_FinishOpenStage(IRC_Context *c)
{
    if (c->wall_samples < IRC_REQUIRED_SAMPLES ||
        c->open_samples < IRC_REQUIRED_SAMPLES) {
        IRC_Abort(c, "SAMPLES");
        return;
    }
    c->state = IRC_REVIEW;
    if (IRC_ComputeThresholds(c)) {
        BT_Printf("CAL,REVIEW,samples_wall=%lu,samples_open=%lu,failed=0,quality=CANDIDATE_ONLY\r\n",
                  (unsigned long)c->wall_samples, (unsigned long)c->open_samples);
        if (!IRC_Save(c)) {
            c->state = IRC_FAULT;
            BT_SendString("CAL,END,result=SAVE_FAILED,flash=FAILED\r\n");
        }
    } else {
        BT_Printf("CAL,REVIEW,samples_wall=%lu,samples_open=%lu,failed=%u,quality=INSUFFICIENT\r\n",
                  (unsigned long)c->wall_samples, (unsigned long)c->open_samples,
                  (unsigned)c->failed_channels);
        BT_SendString("CAL,END,result=INSUFFICIENT,quality=INSUFFICIENT,flash=SKIPPED\r\n");
    }
}

void IRC_Update(IRC_Context *c, uint32_t now)
{
    if (!IRC_IsMotionActive(c)) return;

    if (c->state == IRC_WAIT_OPEN) {
        uint32_t elapsed = (uint32_t)(now - c->stage_started_ms);
        if (elapsed >= IRC_OPEN_TRANSITION_MS) {
            (void)IRC_ConfirmOpen(c, now);
        } else {
            uint8_t seconds = (uint8_t)((IRC_OPEN_TRANSITION_MS - elapsed + 999U) / 1000U);
            if (seconds != c->wait_seconds) {
                c->wait_seconds = seconds;
                BT_Printf("CAL,COUNTDOWN,seconds=%u\r\n", (unsigned)seconds);
            }
        }
        return;
    }

    if ((uint32_t)(now - c->stage_started_ms) > IRC_CAPTURE_TIMEOUT_MS) {
        IRC_Abort(c, "CAPTURE_TIMEOUT");
        return;
    }

    (void)IRC_ReadSample(c, now);
    if (c->state == IRC_CAPTURE_WALL &&
        c->wall_samples >= IRC_REQUIRED_SAMPLES) {
        c->state = IRC_WAIT_OPEN;
        c->stage_started_ms = now;
        BT_Printf("CAL,STAGE_DONE,stage=WALL,samples=%lu\r\n",
                  (unsigned long)c->wall_samples);
        c->wait_seconds = (uint8_t)(IRC_OPEN_TRANSITION_MS / 1000U);
        BT_SendString("CAL,WAIT,stage=OPEN,action=MOVE_TO_OPEN_SPACE,delay_ms=4000\r\n");
        BT_SendString("CAL,COUNTDOWN,seconds=4\r\n");
    } else if (c->state == IRC_CAPTURE_OPEN &&
               c->open_samples >= IRC_REQUIRED_SAMPLES) {
        BT_Printf("CAL,STAGE_DONE,stage=OPEN,samples=%lu\r\n",
                  (unsigned long)c->open_samples);
        IRC_FinishOpenStage(c);
    }
}

uint8_t IRC_Save(IRC_Context *c)
{
    IR_Simple_Calib_t data;
    Flash_Status_t status;
    if (!c || c->state != IRC_REVIEW || !c->valid || !Motor_OutputsLocked())
        return 0U;

    memset(&data, 0, sizeof(data));
    data.open_l0 = c->open_p95[0];
    data.open_l45 = c->open_p95[1];
    data.open_l90 = c->open_p95[2];
    data.open_r0 = c->open_p95[3];
    data.open_r45 = c->open_p95[4];
    data.open_r90 = c->open_p95[5];

    data.wall_on_l0 = c->wall_on[0]; data.wall_off_l0 = c->wall_off[0];
    data.wall_on_l45 = c->wall_on[1]; data.wall_off_l45 = c->wall_off[1];
    data.wall_on_l90 = c->wall_on[2]; data.wall_off_l90 = c->wall_off[2];
    data.wall_on_r0 = c->wall_on[3]; data.wall_off_r0 = c->wall_off[3];
    data.wall_on_r45 = c->wall_on[4]; data.wall_off_r45 = c->wall_off[4];

    data.wall_threshold_left = data.wall_on_l90;
    data.wall_threshold_right = data.wall_on_r90;
    data.wall_threshold_front_left = data.wall_on_l0;
    data.wall_threshold_front_right = data.wall_on_r0;
    data.wall_threshold_l45 = data.wall_on_l45;
    data.wall_threshold_r45 = data.wall_on_r45;

    data.l0_min = (c->wall_min[0] < c->open_min[0]) ? c->wall_min[0] : c->open_min[0];
    data.l0_max = (c->wall_max[0] > c->open_max[0]) ? c->wall_max[0] : c->open_max[0];
    data.l45_min = (c->wall_min[1] < c->open_min[1]) ? c->wall_min[1] : c->open_min[1];
    data.l45_max = (c->wall_max[1] > c->open_max[1]) ? c->wall_max[1] : c->open_max[1];
    data.r0_min = (c->wall_min[3] < c->open_min[3]) ? c->wall_min[3] : c->open_min[3];
    data.r0_max = (c->wall_max[3] > c->open_max[3]) ? c->wall_max[3] : c->open_max[3];
    data.r45_min = (c->wall_min[4] < c->open_min[4]) ? c->wall_min[4] : c->open_min[4];
    data.r45_max = (c->wall_max[4] > c->open_max[4]) ? c->wall_max[4] : c->open_max[4];

    data.gain_l0 = data.gain_r0 = 1.0f;
    data.gain_l45 = data.gain_r45 = 1.0f;
    data.gain_l90 = data.gain_r90 = 1.0f;
    data.calib_mode = 1U;
    data.has_open_calib = 1U;
    data.is_calibrated = IR_CALIB_MAGIC;

    /* This two-pose calibration measures wall/no-wall separation, not a
     * geometric cell-center reference. Leave center_l0/r0 and front_diff
     * unset rather than inventing a center setpoint from arbitrary placement. */
    data.center_l0 = 0U;
    data.center_r0 = 0U;
    data.has_front_diff = 0U;

    IR_Stop();
    Control_Stop();
    Motor_LockOutputs();
    status = Flash_SaveIRCalibChecked(&data);
    Control_Start();
    IR_StartScan();

    if (status == FLASH_OK) {
        c->state = IRC_SAVED;
        BT_SendString("CAL,SAVED,flash=OK,auto_apply=0,has_open_calib=1\r\n");
        BT_SendString("CAL,END,result=SUCCESS,quality=WALL_OPEN,flash=SAVED,auto_apply=0\r\n");
        return 1U;
    }
    BT_Printf("CAL,SAVE_REJECT,flash_status=%u\r\n", (unsigned)status);
    return 0U;
}
#endif
