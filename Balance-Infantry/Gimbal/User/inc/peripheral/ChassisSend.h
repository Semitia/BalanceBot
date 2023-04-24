#ifndef _CHASSIS_SEND_H
#define _CHASSIS_SEND_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_control.h"
#include "ins.h"
#include "Gimbal.h"
#include "ChassisSolver.h"

// 发送给底盘的全身控制信息
#define ROBOT_STATE_BIAS 0         // 1比特
#define CONTROL_TYPE_BIAS 1        // 2比特
#define CONTROL_MODE_ACTION_BIAS 3 // 4比特
#define GIMBAL_MODE_BIAS 7         // 3比特
#define SHOOT_MODE_BIAS 10         // 3比特

#pragma pack(push, 1)

typedef struct ChassisSendPack1
{
    int16_t infantry_mode;
    int16_t left_ch_ud_value;
    int16_t right_ch_ud_value;
    int16_t right_ch_lr_value;
} ChassisSendPack1;

typedef struct ChassisSendPack2 //云台yaw和pitch角度
{
    float yaw;
    float target_yaw;
} ChassisSendPack2;

#pragma pack(pop)

extern ChassisSendPack1 chassis_send_pack1;
extern ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode(void);
void Pack_YawPitch(void);

#endif // !_CHASSIS_SEND_H
