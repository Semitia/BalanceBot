/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"

/* 全局变量 */
static unsigned char temptemp[2 * PC_RECVBUF_SIZE]; //用于PC数据处理
unsigned char tempPC[PC_RECVBUF_SIZE];

PCRecvData pc_recv_data;

void PCReceive(unsigned char *PCbuffer)
{
    memcpy(temptemp + PC_RECVBUF_SIZE, PCbuffer, PC_RECVBUF_SIZE);
    for (uint8_t PackPoint = 0; PackPoint < PC_RECVBUF_SIZE + 1; PackPoint++) //防止错位，不一定数组元素的第一个就为
    {
        if (temptemp[PackPoint] == 0xaa)
        {
            memcpy(tempPC, temptemp + PackPoint, PC_RECVBUF_SIZE);
            if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
            {
                //数据解码
                memcpy(&pc_recv_data, tempPC, PC_RECVBUF_SIZE);
            }
            break;
        }
    }
    memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);
}

#define X 0
#define Y 1
#define Z 2
/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
    PCSendData pc_send_data;
    
    pc_send_data.head = 0xaa;
    pc_send_data.tail = 0xbb;
    //三轴加速度
    pc_send_data.accel_x = INS.Accel[X] * 100.0f;
    pc_send_data.accel_y = INS.Accel[Y] * 100.0f;
    pc_send_data.accel_z = INS.Accel[Z] * 100.0f;
    //三轴角速度
    pc_send_data.gyro_x = INS.Gyro[X] * 100.0f;
    pc_send_data.gyro_y = INS.Gyro[Y] * 100.0f;
    pc_send_data.gyro_z = INS.Gyro[Z] * 100.0f;
    //时间戳
    pc_send_data.TimeStamp = xTaskGetTickCountFromISR();

    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
    Append_CRC8_Check_Sum(buff, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    DMA_Cmd(PC_UART_SEND_DMAx_Streamx, ENABLE);
}
