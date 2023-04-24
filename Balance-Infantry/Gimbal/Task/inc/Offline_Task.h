#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

enum ROBOT_SENSORS_DETECT
{
    NOT_INIT, //未初始化掉线检测
    IMU_ON,   // IMU
    IMU_OFF,
    REMOTE_ON, //遥控器
    REMOTE_OFF
};
#pragma pack(1)
typedef struct
{
    int16_t imu_receive_num[2];
    int16_t remote_receive_num;

    enum ROBOT_SENSORS_DETECT imu_state[2];
    enum ROBOT_SENSORS_DETECT remote_state;
} OfflineDetector;
#pragma pack()

void Offline_task(void *pvParameters);

extern OfflineDetector offline_detector;

#endif // !_OFFLINE_TASK_H
