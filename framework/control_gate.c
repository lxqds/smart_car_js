#include "control_gate.h"
#include <string.h>
static void stop(ControlGate *g, ControlState state)
{ g->speed_mm_s=0; g->steering_mdeg=0; g->state=state; }
void ControlGate_Init(ControlGate *g, ControlLimits limits)
{
    memset(g,0,sizeof(*g)); g->limits=limits; g->state=CONTROL_LINK_WAIT;
    if (!limits.timeout_ms || limits.timeout_ms >= 0x80000000U ||
        limits.speed_limit_mm_s < 0 || limits.steering_limit_mdeg < 0)
        g->state=CONTROL_FAULT;
}
void ControlGate_SetLink(ControlGate *g, uint8_t ready)
{
    if(g->state==CONTROL_FAULT) return;
    if(!ready){stop(g,CONTROL_LINK_WAIT);g->have_sequence=0;}
    else if(g->state==CONTROL_LINK_WAIT)stop(g,CONTROL_DISARMED);
}
int ControlGate_Submit(ControlGate *g,const ControlIntent *in,uint32_t now)
{
    if(g->state==CONTROL_LINK_WAIT||g->state==CONTROL_FAULT)return 0;
    ControlGate_Tick(g,now);
    uint32_t delta=in->sequence-g->last_sequence;
    if(g->have_sequence && (!delta || delta>=0x80000000U))return 0;
    if(in->deadman>1 || in->arm_request>1 ||
       in->speed_mm_s>g->limits.speed_limit_mm_s || in->speed_mm_s < -g->limits.speed_limit_mm_s ||
       in->steering_mdeg>g->limits.steering_limit_mdeg || in->steering_mdeg < -g->limits.steering_limit_mdeg) {
        stop(g,CONTROL_DISARMED);return 0;
    }
    g->last_sequence=in->sequence;g->have_sequence=1;g->last_control_ms=now;
    if(!in->deadman){stop(g,CONTROL_DISARMED);return 1;}
    if(g->state==CONTROL_DISARMED){
        if(in->arm_request && in->speed_mm_s==0 && in->steering_mdeg==0)
            g->state=CONTROL_ARMED;
        return 1;
    }
    if(g->limits.allow_outputs){g->speed_mm_s=in->speed_mm_s;g->steering_mdeg=in->steering_mdeg;}
    return 1;
}
void ControlGate_Tick(ControlGate *g,uint32_t now)
{
    if(g->state==CONTROL_ARMED && (uint32_t)(now-g->last_control_ms)>=g->limits.timeout_ms)
        stop(g,CONTROL_DISARMED);
}
void ControlGate_Estop(ControlGate *g){stop(g,CONTROL_FAULT);}
