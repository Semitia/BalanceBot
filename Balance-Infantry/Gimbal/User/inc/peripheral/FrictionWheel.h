#ifndef _FRICTION_WHEEL_H
#define _FRICTION_WHEEL_H

#include "pid.h"
#include "M2006.h" //摩擦轮电机
#include "M3508.h"

#define LEFT_FRICTION_WHEEL 0
#define RIGHT_FRICTION_WHEEL 1

/*  摩擦轮结构体  */
typedef struct FrictionWheel_t
{
    PID_t PidFrictionSpeed[2];
    M3508_Recv friction_motor_recv[2];
    M3508_Info friction_motor_msgs[2];
    float send_to_motor_current[2];
} FrictionWheel_t;

extern FrictionWheel_t friction_wheels;

void FrictionWheel_Init(void);
void FrictionWheel_Set(float speed1, float speed2); //度/s

#endif // !_FRICTION_WHEEL_H
