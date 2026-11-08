#ifndef CAR_LINK_H
#define CAR_LINK_H
#include "car_protocol.h"
#include "control_gate.h"
enum { CAR_OK, CAR_BAD_SESSION, CAR_BAD_LENGTH, CAR_BAD_STATE,
       CAR_STALE, CAR_BAD_VALUE, CAR_UNSUPPORTED };
typedef struct {
    ControlGate gate;
    uint32_t session;
} CarLink;
/* This draft is simulation-only: allow_outputs is forcibly zero. */
void CarLink_Init(CarLink *link, ControlLimits limits);
void CarLink_Disconnect(CarLink *link);
void CarLink_Tick(CarLink *link, uint32_t now);
void CarLink_Handle(CarLink *link,const CarFrame *request,uint32_t now,CarFrame *reply);
#endif
