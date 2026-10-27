/* Low-priority SSD1306 shared-I2C renderer.
 * Main-context only; the MPU6050's IRQ acquisition owns I2C1 first.
 * Dirty GDDRAM runs: short 4-byte commands then <=8-byte DMA bursts.
 * No wait on bus flags and no OLED work in TIM11 or gyro ISR. */
#if defined(OLED_SHARED_I2C)
#include "oled_shared.h"
#include "ssd1306.h"
#include "system_timer.h"
#include <string.h>
#define OLED_PERIOD_MS 200U
#define OLED_DATA_CHUNK 8U
#define OLED_MIN_SCL_HZ 300000UL
#define OLED_GUARD_US 150U
static uint8_t shadow[SSD1306_BUFFER_SIZE];
static uint8_t frame[SSD1306_BUFFER_SIZE];
static uint8_t packet[10]; /* DMA source: must survive until completed. */
static uint16_t offset, pending_pos;
static uint8_t pending_len, frame_active, phase, present, enabled;
static uint32_t last_frame_ms;
static OLED_SharedStats stats;
/* Phase 0 idle, 1 page/column command in flight, 2 pixel DMA in flight. */
void OLED_SharedInit(uint8_t found) {
    memset(shadow,0,sizeof(shadow));
#if defined(SYSTEM_TEST_MODE)
    /* System-test draws a blocking splash BEFORE MPU starts.
     * Mirror those bytes so the later dirty renderer also clears pixels
     * that were lit by the splash, avoiding stale OLED text. */
    if (found) memcpy(shadow,SSD1306_GetFramebuffer(),sizeof(shadow));
#endif
    memset(frame,0,sizeof(frame));
    memset(&stats,0,sizeof(stats));
    offset=0U;pending_pos=0U;pending_len=0U;
    frame_active=0U;phase=0U;enabled=0U;present=found?1U:0U;
    last_frame_ms=millis();
    if(present) I2C_OledAsync_Init(); /* AFTER MPU boot and bus speed probe. */
}
void OLED_SharedEnable(uint8_t on) { enabled=(on&&present)?1U:0U; }
uint8_t OLED_SharedEnabled(void) { return enabled; }
uint8_t OLED_SharedReady(void) { return present; }
void OLED_SharedGetStats(OLED_SharedStats *out) { if(out) *out=stats; }
static uint8_t CanStart(const MPU6050_Handle_t *h,uint8_t len) {
    uint32_t hz,seq,tick,age,estimate;
    if(!h||!h->calibrated||h->calibrating||h->sample_phase) return 0U;
#if defined(MPU_TEMP_MONITOR)
    if(h->temperature_pending) return 0U;
#endif
#if defined(MPU_TIM11_TICK_SCHED)
    seq=h->control_sample_tick_seq;
    tick=h->control_sample_tick_us;
    if(!seq || seq!=h->control_last_started_tick_seq) return 0U;
#else
    return 0U;
#endif
    hz=I2C_GetConfiguredSclHz();
    if(hz<OLED_MIN_SCL_HZ) return 0U; /* OLED is optional. */
    age=(uint32_t)(micros()-tick);
    estimate=((uint32_t)(len+1U)*9UL*1000000UL+hz-1UL)/hz+
             OLED_GUARD_US;
    if(age<200U || age+estimate>=900U) return 0U;
#if defined(MPU_TIM11_TICK_SCHED)
    if(seq!=h->control_sample_tick_seq) return 0U;
#endif
    return 1U;
}
static uint8_t TrySend(const MPU6050_Handle_t *h,uint8_t len) {
    I2C_Status s;
    if(!CanStart(h,len)) { ++stats.admission_skips;return 0U; }
    s=I2C_OledAsync_Start(SSD1306_I2C_ADDR,packet,len);
    if(s==I2C_OK) return 1U;
    if(s==I2C_BUSY) ++stats.busy_skips;
    else ++stats.errors;
    return 0U;
}
void OLED_SharedService(const MPU6050_Handle_t *h) {
    I2C_Status status;
    uint32_t dur;
    uint16_t pos;
    uint8_t count;
    if(!present) return;
    if(phase==1U || phase==2U) {
        status=I2C_OledAsync_Poll(&dur);
        if(status==I2C_BUSY) return;
        if(status!=I2C_OK) {
            ++stats.errors;phase=0U;frame_active=0U;
            last_frame_ms=millis();return;
        }
        if(phase==1U) {
            phase=3U; /* Command done; defer data without resending it. */
        } else {
            memcpy(&shadow[pending_pos],&frame[pending_pos],pending_len);
            stats.data_bytes+=pending_len;
            ++stats.chunks;
            offset=(uint16_t)(pending_pos+pending_len);
            phase=0U;
        }
    }
    if(phase==3U) {
        if(!enabled) {
            phase=0U;frame_active=0U;return;
        }
        /* I2C gyro transactions do not change SSD1306 column/page pointer.
         * Shorten burst to remaining slack, or defer to next 1ms window. */
        count=pending_len;
        if(CanStart(h,2U)) {
            while(count>1U && !CanStart(h,(uint8_t)(count+1U))) --count;
        }
        packet[0]=0x40U;
        memcpy(&packet[1],&frame[pending_pos],count);
        if(TrySend(h,(uint8_t)(count+1U))) {
            pending_len=count;
            phase=2U;
        }
        return;
    }
    if(!enabled) return;
    if(!frame_active) {
        if((uint32_t)(millis()-last_frame_ms)<OLED_PERIOD_MS) return;
        memcpy(frame,SSD1306_GetFramebuffer(),sizeof(frame));
        last_frame_ms=millis();offset=0U;frame_active=1U;
    }
    while(offset<SSD1306_BUFFER_SIZE && frame[offset]==shadow[offset])
        ++offset;
    if(offset>=SSD1306_BUFFER_SIZE) {
        frame_active=0U;++stats.frames;return;
    }
    pos=offset;count=0U;
    while(count<OLED_DATA_CHUNK && pos+count<SSD1306_BUFFER_SIZE &&
          (pos+count)/SSD1306_WIDTH==pos/SSD1306_WIDTH &&
          frame[pos+count]!=shadow[pos+count]) ++count;
    /* Estimate command + data bus occupancy before reserving a page.
     * Otherwise a command could run without a feasible data slot. */
    while(count>0U && !CanStart(h,(uint8_t)(count+6U))) --count;
    if(!count) { ++stats.admission_skips;return; }
    packet[0]=0x00U;
    packet[1]=(uint8_t)(0xB0U+pos/SSD1306_WIDTH);
    packet[2]=(uint8_t)((pos%SSD1306_WIDTH)&0x0FU);
    packet[3]=(uint8_t)(0x10U+((pos%SSD1306_WIDTH)>>4));
    if(TrySend(h,4U)) {
        pending_pos=pos;pending_len=count;
        phase=1U;++stats.commands;
    }
}
#endif
