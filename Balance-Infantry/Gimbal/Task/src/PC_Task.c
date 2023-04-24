/**
 ******************************************************************************
 * @file    PC_Task.c
 * @brief   与PC通信任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "PC_Task.h"

/**
 * @brief 发送PC通信串口通信任务
 * @param[in] void
 */
void PCReceive_task(void)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 4; // 250HZ

    vTaskDelay(5000); //待机器人初始化后开始检测

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        SendtoPC();

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
