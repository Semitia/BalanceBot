#ifndef __MOTION_H
#define __MOTION_H
#include "sys.h"

#define car_d 37.4266
#define wheel 0.016515
#define delta_t 0.01

//the Threshold of yaw control 进行转向命令的角度差阈值
#define YAW_THR 5
/*
//车身参数
typedef struct __car_t{
    float d;

}car_t;
*/

/*
typedef struct __pisotion_t{
    float x;
    float y;
    float theta;
}position_t;
*/



//state_t next, past, now;//两个时刻的状态，要做全局变量；尽量取简短的单词
//void forward();
//void reverse();
void state_update(void);
#endif

