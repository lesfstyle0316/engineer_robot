#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H
#include "robot_config.h"
#include <stdbool.h>
typedef struct {
    uint8_t bus, dlc; uint16_t id; uint8_t data[8];
} MotorFrame;
typedef struct {
    bool seen, position_lost;
    uint16_t encoder; int16_t rpm, current;
    uint8_t temperature;
    int64_t total_counts; /* 上电第一帧为0，之后连续累计，不重置yaw零点 */
    uint32_t stamp_ms, sequence;
} DjiFeedback;
typedef struct {
    bool seen; uint8_t state, mos_temperature, rotor_temperature;
    float position, velocity, torque;
    uint32_t stamp_ms, sequence;
} DmFeedback;
bool Dji_Decode(DjiFeedback *s,const uint8_t *b,uint8_t dlc,uint32_t now);
bool Dm_Decode(DmFeedback *s,const DmConfig *c,const uint8_t *b,uint8_t dlc,uint32_t now);
void Dji_GroupFrame(MotorFrame *f,uint8_t bus,uint16_t id,const int16_t values[4]);
void Dm_SpecialFrame(MotorFrame *f,const DmConfig *c,bool enable);
void Dm_PositionFrame(MotorFrame *f,const DmConfig *c,float pos,float max_velocity);
float Dji_JointDegrees(const DjiFeedback *s,const DjiConfig *c);
float Dm_Unpack(unsigned value,float max,unsigned bits);
#endif
