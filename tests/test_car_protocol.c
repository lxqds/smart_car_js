#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../framework/car_link.h"

static unsigned count;
static CarFrame last;
static void collect(void *context,const CarFrame *f) { (void)context;last=*f;count++; }
static void dump(void *context,const CarFrame *f)
{
    (void)context;
    printf("%u %lu %lu ",f->command,(unsigned long)f->sequence,(unsigned long)f->session);
    for(unsigned i=0;i<f->length;i++)printf("%02x",f->payload[i]);
    puts("");
}
static void feed(CarParser *p,const uint8_t *wire,size_t n,uint32_t now)
{ for(size_t i=0;i<n;i++){CarParser_Feed(p,wire[i],now,collect,NULL);assert(p->used<CAR_MAX_FRAME);} }
static unsigned parse_hex(const char *s,uint8_t *out,unsigned max)
{
    size_t n=strlen(s);assert(!(n%2) && n/2<=max);
    for(size_t i=0;i<n/2;i++){unsigned x;assert(sscanf(s+i*2,"%2x",&x)==1);out[i]=(uint8_t)x;}
    return (unsigned)n/2;
}
static void link_tests(void)
{
    CarLink l;ControlLimits limits={200,500,15000,1};
    CarLink_Init(&l,limits);assert(!l.gate.limits.allow_outputs);
    CarFrame q={0},r;q.command=CAR_ARM;q.session=123;q.length=9;q.payload[8]=1;
    CarLink_Handle(&l,&q,0,&r);assert(r.payload[1]==CAR_BAD_SESSION);
    q.command=CAR_HELLO;q.length=0;CarLink_Handle(&l,&q,0,&r);
    assert(r.command==CAR_CAPS && r.payload[3]==0);
    q.command=CAR_ARM;q.sequence=1;q.length=9;
    CarLink_Handle(&l,&q,1,&r);assert(r.payload[1]==CAR_OK && l.gate.state==CONTROL_ARMED);
    /* Lost ACK: identical retry does not execute again or renew freshness. */
    CarLink_Handle(&l,&q,150,&r);assert(r.payload[1]==CAR_STALE && l.gate.last_control_ms==1);
    q.command=CAR_STATUS;q.length=0;CarLink_Handle(&l,&q,201,&r);
    assert(l.gate.state==CONTROL_DISARMED);
    q.command=CAR_CONTROL;q.length=9;q.sequence=2;Car_PutU32(q.payload,100);
    CarLink_Handle(&l,&q,202,&r);assert(r.payload[1]==CAR_BAD_STATE);
    q.command=CAR_ARM;Car_PutU32(q.payload,0);CarLink_Handle(&l,&q,203,&r);
    assert(l.gate.state==CONTROL_ARMED);
    q.command=CAR_CONTROL;q.sequence=3;Car_PutU32(q.payload,(uint32_t)-123);
    CarLink_Handle(&l,&q,204,&r);assert(r.payload[1]==CAR_OK && l.gate.speed_mm_s==0);
    q.payload[8]=0;q.sequence=4;CarLink_Handle(&l,&q,205,&r);assert(l.gate.state==CONTROL_DISARMED);
    q.command=CAR_DISARM;q.length=0;q.sequence=6;CarLink_Handle(&l,&q,205,&r);
    q.command=CAR_ARM;q.length=9;q.sequence=5;q.payload[8]=1;Car_PutU32(q.payload,0);
    CarLink_Handle(&l,&q,205,&r);assert(r.payload[1]==CAR_STALE && l.gate.state==CONTROL_DISARMED);
    q.command=CAR_HELLO;q.length=0;q.session=456;CarLink_Handle(&l,&q,206,&r);
    q.session=123;q.command=CAR_ARM;q.length=9;q.payload[8]=1;
    CarLink_Handle(&l,&q,207,&r);assert(r.payload[1]==CAR_BAD_SESSION);
    q.session=456;q.command=99;q.length=0;CarLink_Handle(&l,&q,208,&r);assert(r.payload[1]==CAR_UNSUPPORTED);
    q.command=CAR_ARM;q.length=8;CarLink_Handle(&l,&q,209,&r);assert(r.payload[1]==CAR_BAD_LENGTH);
    q.length=9;q.payload[8]=2;CarLink_Handle(&l,&q,210,&r);assert(r.payload[1]==CAR_BAD_VALUE);
    q.command=CAR_ESTOP;q.length=0;CarLink_Handle(&l,&q,211,&r);assert(l.gate.state==CONTROL_FAULT);
    CarLink_Disconnect(&l);q.command=CAR_HELLO;q.session=789;
    CarLink_Handle(&l,&q,212,&r);assert(r.payload[1]==CAR_BAD_STATE && l.gate.state==CONTROL_FAULT);
}
int main(int argc,char **argv)
{
    uint8_t wire[CAR_MAX_FRAME];CarParser p;CarParser_Init(&p);
    if(argc==3 && !strcmp(argv[1],"decode")) {
        const char *s=argv[2];assert(!(strlen(s)%2));
        for(size_t i=0;s[i];i+=2){unsigned x;assert(sscanf(s+i,"%2x",&x)==1);CarParser_Feed(&p,(uint8_t)x,0,dump,NULL);}
        return 0;
    }
    if(argc==6 && !strcmp(argv[1],"encode")) {
        CarFrame f={0};f.command=(uint8_t)strtoul(argv[2],NULL,10);
        f.sequence=(uint32_t)strtoul(argv[3],NULL,10);f.session=(uint32_t)strtoul(argv[4],NULL,10);
        f.length=(uint16_t)parse_hex(argv[5],f.payload,CAR_MAX_PAYLOAD);
        size_t n=Car_Encode(&f,wire,sizeof(wire));for(size_t i=0;i<n;i++)printf("%02x",wire[i]);puts("");return 0;
    }
    assert(Car_Crc16((const uint8_t *)"123456789",9)==0x29b1);
    CarFrame f={0};f.command=CAR_CONTROL;f.sequence=0xffffffffU;f.session=0x12345678;f.length=9;
    Car_PutU32(f.payload,(uint32_t)-123);Car_PutU32(f.payload+4,15000);f.payload[8]=1;
    size_t n=Car_Encode(&f,wire,sizeof(wire));assert(n==25);
    for(size_t split=0;split<=n;split++) {
        CarParser_Init(&p);count=0;feed(&p,wire,split,0);feed(&p,wire+split,n-split,1);
        assert(count==1 && Car_GetI32(last.payload)==-123);
    }
    count=0;for(unsigned i=0;i<1000;i++)feed(&p,wire,n,i);assert(count==1000);
    for(size_t bad=0;bad<n;bad++) {
        uint8_t corrupt[CAR_MAX_FRAME];memcpy(corrupt,wire,n);corrupt[bad]^=0x80;
        CarParser_Init(&p);count=0;feed(&p,corrupt,n,0);feed(&p,wire,n,1);feed(&p,wire,n,2);
        assert(count>=1 && last.sequence==f.sequence);
    }
    CarParser_Init(&p);count=0;feed(&p,wire,8,UINT32_MAX-50U);feed(&p,wire,n,49);
    assert(count==1 && p.timeouts==1);
    f.length=CAR_MAX_PAYLOAD;memset(f.payload,0xa5,sizeof(f.payload));n=Car_Encode(&f,wire,sizeof(wire));
    feed(&p,wire,n,50);assert(last.length==32 && !p.used);
    assert(!Car_Encode(&f,wire,47));f.length=33;assert(!Car_Encode(&f,wire,sizeof(wire)));
    uint32_t rng=1;for(unsigned i=0;i<100000;i++){rng=rng*1664525U+1013904223U;CarParser_Feed(&p,(uint8_t)(rng>>24),50,collect,NULL);assert(p.used<CAR_MAX_FRAME);}
    link_tests();puts("PASS protocol parser, noise/CRC/length/resync/wrap and no-output session link");
    return 0;
}
