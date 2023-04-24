#ifndef _CHASSIS_SOLVER_H
#define _CHASSIS_SOLVER_H

#include "remote_control.h"
#include "debug.h"

#include "Offline_Task.h"
#include "ins.h"
#include "counter.h"
#include "Gimbal.h"

#define SPEED_MAX 2.2f      //最大速度
#define SPEED_W_MAX 3.5f    //最大角速度
#define MAX_LEG_SPEED 0.08f //最大伸腿速度 m/s

#define MAX_SW_YAW_SPEED 90.0f   //云台yaw轴灵敏度(拨杆) 度/s
#define MAX_SW_PITCH_SPEED 10.0f //云台yaw轴灵敏度(拨杆) 度/s

typedef struct ChassisSolver
{
    float target_v;
    float target_yaw_v; //底盘yaw角度
    float leg_speed;

    // yaw轴云台
    float target_gimbal_yaw; //云台yaw角度目标值
    uint32_t last_cnt;
    float delta_t;
} ChassisSolver;

extern ChassisSolver chassis_solver;

void get_control_info(ChassisSolver *infantry);

#endif // !_CHASSIS_SOLVER_H
