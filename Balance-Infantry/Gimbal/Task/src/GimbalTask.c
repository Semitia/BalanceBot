#include "GimbalTask.h"

int8_t dji_motors_send_data[8];
int8_t dji_motors_send_data_2[8];
float testl;

void Gimbal_Powerdown_Cal()
{
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    limitPitchAngle();
    GimbalClear();

    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, 0);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, 0);
}

void Gimbal_Autoaim_Cal()
{
}

void Gimbal_Buff_Cal()
{
}

void Gimbal_SI_Cal()
{
}

void Gimbal_Act_Cal()
{
}

void Gimbal_Test_Cal()
{
    //更新传感器信息
    updateGyro();
    GM6020_Decode(&gimbal_controller.pitch_recv, &gimbal_controller.pitch_info);
    GM6020_Decode(&gimbal_controller.yaw_recv, &gimbal_controller.yaw_info);

    // pitch限制幅值
    limitPitchAngle();
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
    GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, PITCH_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_pitch_vol);

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    // Gimbal_Speed_Calculate(100.0f);
    // GM6020_SendPack(dji_motors_send_data, GM6020_STD_ID_1_4, YAW_MOTOR_CAN_ID - 0x204, (int16_t)gimbal_controller.set_yaw_vol);
}

void Shoot_Powerdown_Cal()
{
    //摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }

    // FrictionWheel_Set(0, 0);
    // M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    // M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    PID_Clear(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL]);
    PID_Clear(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL]);

    friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] = 0;
    friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] = 0;
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    //拨盘电机
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.80);
    float send_current = Toggle_Calculate(TOGGLE_STOP, 0.0f);
    M2006_SendPack(dji_motors_send_data_2, C610_STD_ID_1_4, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

void Shoot_Check_Cal()
{
    //弹舱盖

    //拨盘

    //摩擦轮
}

void Shoot_Fire_Cal()
{
    //弹舱盖

    //拨盘

    //摩擦轮
}

void Shoot_Test_Cal()
{
    //摩擦轮
    // for (int i = 0; i < 2; i++)
    // {
    //     M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    // }
    // FrictionWheel_Set(-1000, 1000);
    // M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_1 - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    // M3508_SendPack(dji_motors_send_data, C620_STD_ID_5_8, FRICTION_WHEEL_CAN_ID_2 - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);

    //拨弹电机
    //拨盘
    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.80);
    float send_current = Toggle_Calculate(TOGGLE_SPEED, 108.0f);

    //打包数据
    M2006_SendPack(dji_motors_send_data_2, C610_STD_ID_1_4, TOGGLE_MOTOR_CAN_ID - 0x200, send_current);
}

void Shoot_Autoaim_Cal()
{
    //弹舱盖

    //拨盘

    //摩擦轮
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ

    memset(dji_motors_send_data, 0, 8);
    memset(dji_motors_send_data_2, 0, 8);
    FrictionWheel_Init();
    GimbalPidInit();
    TogglePidInit();

    vTaskDelay(2000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        switch (remote_controller.gimbal_action)
        {
        case GIMBAL_POWERDOWN: //掉电模式
            Gimbal_Powerdown_Cal();
            break;
        case GIMBAL_ACT_MODE: //云台运动模式
            Gimbal_Act_Cal();
            break;
        case GIMBAL_AUTO_AIM_MODE: //自瞄模式
            Gimbal_Autoaim_Cal();
            break;
        case GIMBAL_BUFF_MODE:
            Gimbal_Buff_Cal();
            break;
        case GIMBAL_SI_MODE:
            Gimbal_SI_Cal();
            break;
        case GIMBAL_TEST_MODE:
            Gimbal_Test_Cal();
            break;
        default:
            Gimbal_Powerdown_Cal();
            break;
        }

        switch (remote_controller.shoot_action)
        {
        case SHOOT_POWERDOWN_MODE: //掉电模式
            Shoot_Powerdown_Cal();
            break;
        case SHOOT_CHECK_MODE: //自检模式
            Shoot_Check_Cal();
            break;
        case SHOOT_FIRE_MODE: //开火模式
            Shoot_Fire_Cal();
            break;
        case SHOOT_TEST_MODE: //弹道测试模式
            Shoot_Test_Cal();
            break;
        case SHOOT_AUTO_AIM_MODE: //自瞄模式
            Shoot_Autoaim_Cal();
            break;

        default:
            Shoot_Powerdown_Cal();
            break;
        }

        CanSend(DJI_MOTORS_CAN, dji_motors_send_data, C620_STD_ID_5_8, 8);
        CanSend(CHASSIS_CAN_COMM_CANx, dji_motors_send_data_2, C610_STD_ID_1_4, 8);
        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
