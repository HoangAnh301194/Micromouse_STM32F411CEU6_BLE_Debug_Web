#include "ir_filter.h"
#include <string.h>

static uint16_t median3(uint16_t a,uint16_t b,uint16_t c)
{
    if(a>b){uint16_t t=a;a=b;b=t;}
    if(b>c){uint16_t t=b;b=c;c=t;}
    if(a>b){uint16_t t=a;a=b;b=t;}
    return b;
}
static uint16_t to_adc(float f)
{
    if(f<=0.0f)return 0U;
    if(f>=4095.0f)return 4095U;
    return (uint16_t)(f+0.5f);
}
void IR_FilterReset(IR_Filter_t *state)
{
    if(state)memset(state,0,sizeof(*state));
}
void IR_FilterStep(IR_Filter_t *s,
                   const uint16_t raw[IR_FILTER_CHANNELS],
                   IR_FilterFrame_t *out)
{
    uint8_t i;
    if(!s||!raw||!out)return;
    for(i=0U;i<IR_FILTER_CHANNELS;++i){
        uint16_t r=raw[i]>4095U?4095U:raw[i];
        uint16_t med;
        s->ring[i][s->ring_index]=r;
        if(s->ring_count==0U)med=r;
        else if(s->ring_count==1U)
            med=median3(s->ring[i][0],s->ring[i][0],s->ring[i][1]);
        else med=median3(s->ring[i][0],s->ring[i][1],s->ring[i][2]);
        if(!s->initialized){
            s->ema10[i]=(float)r;
            s->ema25[i]=(float)r;
            s->median_ema25[i]=(float)med;
        }else{
            s->ema10[i]+=0.10f*((float)r-s->ema10[i]);
            s->ema25[i]+=0.25f*((float)r-s->ema25[i]);
            s->median_ema25[i]+=0.25f*((float)med-s->median_ema25[i]);
        }
        out->raw[i]=r;
        out->median3[i]=med;
        out->ema10[i]=to_adc(s->ema10[i]);
        out->ema25[i]=to_adc(s->ema25[i]);
        out->median_ema25[i]=to_adc(s->median_ema25[i]);
    }
    if(s->ring_count<3U)++s->ring_count;
    s->ring_index=(uint8_t)((s->ring_index+1U)%3U);
    s->initialized=1U;
}
