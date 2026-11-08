#include "car_protocol.h"
#include <string.h>

uint32_t Car_GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) |
           ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
int32_t Car_GetI32(const uint8_t *p)
{
    uint32_t n=Car_GetU32(p);
    return n<=INT32_MAX ? (int32_t)n : -1-(int32_t)(UINT32_MAX-n);
}
void Car_PutU32(uint8_t *p, uint32_t n)
{ for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(n>>(i*8)); }
uint16_t Car_Crc16(const uint8_t *p, size_t length)
{
    uint16_t crc=0xffff;
    while(length--) {
        crc^=(uint16_t)*p++ << 8;
        for(unsigned i=0;i<8;i++)
            crc=(uint16_t)((crc<<1)^((crc&0x8000U)?0x1021U:0U));
    }
    return crc;
}
size_t Car_Encode(const CarFrame *f, uint8_t *out, size_t capacity)
{
    size_t n=16U+f->length;
    if(f->length>CAR_MAX_PAYLOAD || capacity<n) return 0;
    out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=f->command;
    out[4]=(uint8_t)f->length;out[5]=(uint8_t)(f->length>>8);
    Car_PutU32(out+6,f->sequence);Car_PutU32(out+10,f->session);
    memcpy(out+14,f->payload,f->length);
    uint16_t crc=Car_Crc16(out+2,n-4);
    out[n-2]=(uint8_t)crc;out[n-1]=(uint8_t)(crc>>8);
    return n;
}
void CarParser_Init(CarParser *p) { memset(p,0,sizeof(*p)); }
static void remove_bytes(CarParser *p,size_t count)
{
    p->used-=count;
    memmove(p->bytes,p->bytes+count,p->used);
}
void CarParser_Feed(CarParser *p,uint8_t byte,uint32_t now,
                    CarFrameHandler handler,void *context)
{
    if(p->used && (uint32_t)(now-p->last_byte_ms)>=CAR_PARTIAL_TIMEOUT_MS) {
        p->used=0;p->timeouts++;
    }
    p->last_byte_ms=now;
    /* At most 48 bytes retained and inspected per call. No allocation/waits. */
    p->bytes[p->used++]=byte;
    while(p->used) {
        if(p->bytes[0]!=0xa5) {remove_bytes(p,1);continue;}
        if(p->used<2) return;
        if(p->bytes[1]!=0x5a) {remove_bytes(p,1);continue;}
        if(p->used<6) return;
        size_t length=(size_t)p->bytes[4] | ((size_t)p->bytes[5]<<8);
        if(p->bytes[2]!=1 || length>CAR_MAX_PAYLOAD) {
            p->header_errors++;remove_bytes(p,1);continue;
        }
        size_t total=16U+length;
        if(p->used<total) return;
        uint16_t crc=(uint16_t)(p->bytes[total-2] | ((uint16_t)p->bytes[total-1]<<8));
        if(crc!=Car_Crc16(p->bytes+2,total-4)) {
            p->crc_errors++;remove_bytes(p,1);continue;
        }
        CarFrame f={0};
        f.command=p->bytes[3];f.length=(uint16_t)length;
        f.sequence=Car_GetU32(p->bytes+6);f.session=Car_GetU32(p->bytes+10);
        memcpy(f.payload,p->bytes+14,length);
        remove_bytes(p,total);
        /* Callback must be bounded and must not re-enter this parser. */
        handler(context,&f);
    }
}
