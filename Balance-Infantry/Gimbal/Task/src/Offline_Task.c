#include "Offline_Task.h"

OfflineDetector offline_detector;

void Offline_task(void *pvParameters)
{
    vTaskDelay(5000); //待机器人初始化后开始检测

    while (1)
    {
        // IMU
        for (int i = 0; i < 2; i++)
        {
            if (global_debugger.imu_debugger[i].recv_msgs_num != offline_detector.imu_receive_num[i])
            {
                offline_detector.imu_state[i] = IMU_ON;
                offline_detector.imu_receive_num[i] = global_debugger.imu_debugger[i].recv_msgs_num;
            }
            else
            {
                offline_detector.imu_state[i] = IMU_OFF;
            }
        }

        //遥控器
        if (global_debugger.remote_debugger.recv_msgs_num != offline_detector.remote_receive_num)
        {
            offline_detector.remote_state = REMOTE_ON;
            offline_detector.remote_receive_num = global_debugger.remote_debugger.recv_msgs_num;
        }
        else
        {
            offline_detector.remote_state = REMOTE_OFF;
        }

        vTaskDelay(200); //所有数据都应该超过5HZ
    }
}
