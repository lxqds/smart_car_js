#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

/* Draft v1. Portable, allocation-free; not connected to production UART. */
#define CAR_MAX_PAYLOAD 32U
#define CAR_MAX_FRAME (16U + CAR_MAX_PAYLOAD)
#define CAR_PARTIAL_TIMEOUT_MS 100U
enum {
    CAR_HELLO=1, CAR_CAPS=2, CAR_ARM=3, CAR_DISARM=4,
    CAR_CONTROL=5, CAR_ESTOP=6, CAR_STATUS=7, CAR_ACK=8
};
typedef struct {
    uint8_t command;
    uint16_t length;
    uint32_t sequence, session;
    uint8_t payload[CAR_MAX_PAYLOAD];
} CarFrame;
typedef void (*CarFrameHandler)(void *context, const CarFrame *frame);
typedef struct {
    uint8_t bytes[CAR_MAX_FRAME];
    size_t used;
    uint32_t last_byte_ms, crc_errors, header_errors, timeouts;
} CarParser;
uint16_t Car_Crc16(const uint8_t *data, size_t length);
size_t Car_Encode(const CarFrame *frame, uint8_t *out, size_t capacity);
void CarParser_Init(CarParser *parser);
void CarParser_Feed(CarParser *parser, uint8_t byte, uint32_t now,
                    CarFrameHandler handler, void *context);
uint32_t Car_GetU32(const uint8_t *p);
int32_t Car_GetI32(const uint8_t *p);
void Car_PutU32(uint8_t *p, uint32_t value);
#endif
