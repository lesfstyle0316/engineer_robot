#include "drv_pid.h"
#include <math.h>
float Control_Clamp(float x,float lo,float hi) { return x<lo?lo:(x>hi?hi:x); }
float Control_Slew(float x,float goal,float step) { return x+Control_Clamp(goal-x,-step,step); }
void Pid_Reset(Pid *p) { p->integral=0.0f; }
float Pid_Update(Pid *p,const PidConfig *c,float e,float dt) {
    if(!isfinite(e) || !isfinite(dt) || dt<=0.0f) { Pid_Reset(p);return 0.0f; }
    float candidate=Control_Clamp(p->integral+c->ki*e*dt,-c->integral_limit,c->integral_limit);
    float raw=c->kp*e+candidate;
    /* 条件积分，饱和时只有减轻饱和的方向可以继续积分。 */
    if(fabsf(raw)<=c->output_limit || (raw>0.0f && e<0.0f) || (raw<0.0f && e>0.0f))
        p->integral=candidate;
    return Control_Clamp(c->kp*e+p->integral,-c->output_limit,c->output_limit);
}
