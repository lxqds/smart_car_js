#ifndef CONTROL_GATE_H
#define CONTROL_GATE_H
#include <stdint.h>

/* Platform-independent draft. Not wired to UART, main, timers or GPIO. */
typedef enum {CONTROL_LINK_WAIT, CONTROL_DISARMED, CONTROL_ARMED, CONTROL_FAULT} ControlState;
typedef struct {
    uint32_t sequence;
    int32_t speed_mm_s;
    int32_t steering_mdeg;
    uint8_t deadman, arm_request;
} ControlIntent;
typedef struct {
    uint32_t timeout_ms;
    int32_t speed_limit_mm_s, steering_limit_mdeg;
    uint8_t allow_outputs;
} ControlLimits;
typedef struct {
    ControlState state;
    ControlLimits limits;
    uint32_t last_sequence, last_control_ms;
    uint8_t have_sequence;
    int32_t speed_mm_s, steering_mdeg;
} ControlGate;
void ControlGate_Init(ControlGate *gate, ControlLimits limits);
void ControlGate_SetLink(ControlGate *gate, uint8_t ready);
int ControlGate_Submit(ControlGate *gate, const ControlIntent *intent, uint32_t now);
void ControlGate_Tick(ControlGate *gate, uint32_t now);
void ControlGate_Estop(ControlGate *gate);
#endif
