/**
 ******************************************************************************
 * @file    can1_receive.c
 * @brief   CAN1 接收函数
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "can1_receive.h"
#include "remote_control.h"

/*  接收全局数据定义 */

/*  接收函数定义 */
void Can1_FIFO0_Receive0(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive1(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive2(CanRxMsg *rx_message0);
void Can1_FIFO0_Receive3(CanRxMsg *rx_message0);

void Can1_FIFO1_Receive0(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive1(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive2(CanRxMsg *rx_message1);
void Can1_FIFO1_Receive3(CanRxMsg *rx_message1);

/**
 * @brief CAN1 FIFO0数据接收
 * @param[in] rx_message0 CAN1 FIFO0接收数据
 */
void Can1Receive0(CanRxMsg rx_message0)
{
    switch (rx_message0.StdId)
    {
    case CAN1_FIFO0_ID0:
        Can1_FIFO0_Receive0(&rx_message0);
        break;
    case CAN1_FIFO0_ID1:
        Can1_FIFO0_Receive1(&rx_message0);
        break;
    case CAN1_FIFO0_ID2:
        Can1_FIFO0_Receive2(&rx_message0);
        break;
    case CAN1_FIFO0_ID3:
        Can1_FIFO0_Receive3(&rx_message0);
        break;
    default:
        break;
    }
}

/**
 * @brief CAN1 FIFO1数据接收
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1Receive1(CanRxMsg rx_message1)
{
    switch (rx_message1.StdId)
    {
    case CAN1_FIFO1_ID0:
        Can1_FIFO1_Receive0(&rx_message1);
        break;
    case CAN1_FIFO1_ID1:
        Can1_FIFO1_Receive1(&rx_message1);
        break;
    case CAN1_FIFO1_ID2:
        Can1_FIFO1_Receive2(&rx_message1);
        break;
    case CAN1_FIFO1_ID3:
        Can1_FIFO1_Receive3(&rx_message1);
        break;
    default:
        break;
    }
}

/*  接收数据传递到外部 */

/**
 * @brief CAN1 FIFO0 ID0 数据接收(0x100,GYRO)
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */

void Can1_FIFO0_Receive0(CanRxMsg *rx_message0)
{
    //蓝牙数据接收
    uint16_t blue_tooth_recv;
    memcpy(&blue_tooth_recv, rx_message0->Data, 2);
    Blue_Tooth_Deal(&blue_tooth_recv);
}

/**
 * @brief CAN1 FIFO0 ID1 数据接收(0x101,Accel)
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive1(CanRxMsg *rx_message0)
{
    //遥控器数据接收
    static uint8_t data[6];
    memcpy(data, rx_message0->Data, 6);
    RemoteReceive(data);
}

/**
 * @brief CAN1 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive2(CanRxMsg *rx_message0)
{
}

/**
 * @brief CAN1 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN1 FIFO0接收数据
 */
void Can1_FIFO0_Receive3(CanRxMsg *rx_message0)
{
    // 拨弹电机数据接收
    toggle_controller.toggle_recv.angle = (rx_message0->Data[0] << 8) | (rx_message0->Data[1]);
    toggle_controller.toggle_recv.speed = (rx_message0->Data[2] << 8) | (rx_message0->Data[3]);
    toggle_controller.toggle_recv.torque_current = (rx_message0->Data[4] << 8) | (rx_message0->Data[5]);

    global_debugger.toggle_debugger.recv_msgs_num++;
    global_debugger.toggle_debugger.can_dt = GetDeltaT(&global_debugger.toggle_debugger.last_can_cnt);
    if (global_debugger.toggle_debugger.can_dt > 0.0015f)
    {
        global_debugger.toggle_debugger.loss_num++;
    }
}

/**
 * @brief CAN1 FIFO1 ID0 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive0(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN1 FIFO0 ID1 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive1(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN1 FIFO0 ID2 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive2(CanRxMsg *rx_message1)
{
}

/**
 * @brief CAN1 FIFO0 ID3 数据接收()
 * @param[in] rx_message1 CAN1 FIFO1接收数据
 */
void Can1_FIFO1_Receive3(CanRxMsg *rx_message1)
{
}
