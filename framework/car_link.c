#include "car_link.h"
#include <string.h>

void CarLink_Init(CarLink *l,ControlLimits limits)
{ memset(l,0,sizeof(*l));limits.allow_outputs=0;ControlGate_Init(&l->gate,limits); }
void CarLink_Disconnect(CarLink *l)
{ l->session=0;ControlGate_SetLink(&l->gate,0); }
void CarLink_Tick(CarLink *l,uint32_t now) { ControlGate_Tick(&l->gate,now); }
static uint8_t apply(CarLink *l,const CarFrame *q,uint32_t now)
{
    if(q->command==CAR_HELLO) {
        if(q->length) return CAR_BAD_LENGTH;
        if(!q->session) return CAR_BAD_SESSION;
        if(l->gate.state==CONTROL_FAULT) return CAR_BAD_STATE;
        if(q->session!=l->session) {
            ControlGate_SetLink(&l->gate,0);
            l->session=q->session;ControlGate_SetLink(&l->gate,1);
        }
        return CAR_OK; /* Same-session retry never re-arms or clears sequence. */
    }
    if(!l->session || q->session!=l->session) return CAR_BAD_SESSION;
    if(q->command==CAR_DISARM || q->command==CAR_ESTOP || q->command==CAR_STATUS) {
        if(q->length) return CAR_BAD_LENGTH;
        if(q->command==CAR_DISARM && l->gate.state!=CONTROL_FAULT) {
            /* Stop even for an old request, but never lower the replay barrier.
             * A newer DISARM fences off delayed ARM/CONTROL frames. */
            uint32_t delta=q->sequence-l->gate.last_sequence;
            if(!l->gate.have_sequence || (delta && delta<0x80000000U)) {
                l->gate.last_sequence=q->sequence;l->gate.have_sequence=1;
            }
            l->gate.state=CONTROL_DISARMED;
            l->gate.speed_mm_s=0;l->gate.steering_mdeg=0;
        }
        if(q->command==CAR_ESTOP) ControlGate_Estop(&l->gate);
        return CAR_OK;
    }
    if(q->command!=CAR_ARM && q->command!=CAR_CONTROL) return CAR_UNSUPPORTED;
    if(q->length!=9) return CAR_BAD_LENGTH;
    if(l->gate.state==CONTROL_FAULT) return CAR_BAD_STATE;
    uint32_t delta=q->sequence-l->gate.last_sequence;
    if(l->gate.have_sequence && (!delta || delta>=0x80000000U)) return CAR_STALE;
    ControlIntent intent={q->sequence,Car_GetI32(q->payload),Car_GetI32(q->payload+4),q->payload[8],0};
    if(intent.deadman>1 || intent.speed_mm_s>l->gate.limits.speed_limit_mm_s ||
       intent.speed_mm_s < -l->gate.limits.speed_limit_mm_s ||
       intent.steering_mdeg>l->gate.limits.steering_limit_mdeg ||
       intent.steering_mdeg < -l->gate.limits.steering_limit_mdeg) return CAR_BAD_VALUE;
    if(q->command==CAR_ARM) {
        if(intent.speed_mm_s || intent.steering_mdeg || !intent.deadman) return CAR_BAD_VALUE;
        if(l->gate.state!=CONTROL_DISARMED) return CAR_BAD_STATE;
        intent.arm_request=1;
    } else if(l->gate.state!=CONTROL_ARMED && intent.deadman) return CAR_BAD_STATE;
    return ControlGate_Submit(&l->gate,&intent,now)?CAR_OK:CAR_BAD_VALUE;
}
void CarLink_Handle(CarLink *l,const CarFrame *q,uint32_t now,CarFrame *r)
{
    CarLink_Tick(l,now); /* Even diagnostics cannot extend control freshness. */
    uint8_t result=apply(l,q,now);
    memset(r,0,sizeof(*r));r->command=CAR_ACK;r->sequence=q->sequence;r->session=q->session;
    r->length=4;r->payload[0]=q->command;r->payload[1]=result;
    r->payload[2]=(uint8_t)l->gate.state;r->payload[3]=0; /* physical outputs disabled */
    if(q->command==CAR_HELLO && result==CAR_OK) {
        r->command=CAR_CAPS;r->length=16;
        r->payload[0]=1;r->payload[1]=CAR_MAX_PAYLOAD;r->payload[2]=(uint8_t)l->gate.state;
        Car_PutU32(r->payload+4,l->gate.limits.timeout_ms);
        Car_PutU32(r->payload+8,(uint32_t)l->gate.limits.speed_limit_mm_s);
        Car_PutU32(r->payload+12,(uint32_t)l->gate.limits.steering_limit_mdeg);
    }
}
