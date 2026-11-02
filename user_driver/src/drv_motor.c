#include "drv_motor.h"
#include <string.h>
#include <math.h>
static int16_t signed_be(const uint8_t *p) {
    uint16_t u=((uint16_t)p[0]<<8)|p[1];
    return (int16_t)(u>=0x8000u?(int32_t)u-65536:(int32_t)u);
}
bool Dji_Decode(DjiFeedback *s,const uint8_t *b,uint8_t dlc,uint32_t now) {
    if(dlc!=8u) return false;
    uint16_t enc=((uint16_t)b[0]<<8)|b[1];
    if(enc>8191u) return false;
    if(s->seen) {
        if((uint32_t)(now-s->stamp_ms)>=ROBOT_MOTOR_TIMEOUT_MS) s->position_lost=true;
        int d=(int)enc-(int)s->encoder;
        if(d>4096)d-=8192; else if(d< -4096)d+=8192;
        s->total_counts+=d;
    }
    s->encoder=enc;s->rpm=signed_be(b+2);s->current=signed_be(b+4);
    s->temperature=b[6];s->stamp_ms=now;s->seen=true;++s->sequence;return true;
}
float Dji_JointDegrees(const DjiFeedback *s,const DjiConfig *c) {
    return (float)s->total_counts*(360.0f/8192.0f)*c->direction/c->reduction;
}
float Dm_Unpack(unsigned x,float max,unsigned bits) {
    return (float)x*(2.0f*max)/(float)((1u<<bits)-1u)-max;
}
bool Dm_Decode(DmFeedback *s,const DmConfig *c,const uint8_t *b,uint8_t dlc,uint32_t now) {
    if(dlc!=8u || (b[0]&0x0fu)!=(c->motor_id&0x0fu))return false;
    s->state=b[0]>>4;
    s->position=Dm_Unpack(((unsigned)b[1]<<8)|b[2],c->p_max,16);
    s->velocity=Dm_Unpack(((unsigned)b[3]<<4)|(b[4]>>4),c->v_max,12);
    s->torque=Dm_Unpack(((unsigned)(b[4]&15u)<<8)|b[5],c->t_max,12);
    s->mos_temperature=b[6];s->rotor_temperature=b[7];
    s->stamp_ms=now;s->seen=true;++s->sequence;return true;
}
void Dji_GroupFrame(MotorFrame *f,uint8_t bus,uint16_t id,const int16_t v[4]) {
    memset(f,0,sizeof(*f));f->bus=bus;f->id=id;f->dlc=8;
    for(unsigned i=0;i<4;i++) {uint16_t u=(uint16_t)v[i];f->data[2*i]=(uint8_t)(u>>8);f->data[2*i+1]=(uint8_t)u;}
}
void Dm_SpecialFrame(MotorFrame *f,const DmConfig *c,bool enable) {
    memset(f,0,sizeof(*f));f->bus=2;f->id=0x100u+c->motor_id;f->dlc=8;
    memset(f->data,0xff,8);f->data[7]=enable?0xfc:0xfd;
}
/* 显式生成 IEEE754 小端报文；不依赖未对齐访问或指针强转。 */
static void pack_float_le(uint8_t *b,float x) {
    uint32_t u;_Static_assert(sizeof(float)==4,"IEEE754 binary32 required");
    memcpy(&u,&x,4);for(unsigned i=0;i<4;i++)b[i]=(uint8_t)(u>>(8*i));
}
void Dm_PositionFrame(MotorFrame *f,const DmConfig *c,float p,float v) {
    memset(f,0,sizeof(*f));f->bus=2;f->id=0x100u+c->motor_id;f->dlc=8;
    pack_float_le(f->data,p);pack_float_le(f->data+4,fabsf(v));
}
