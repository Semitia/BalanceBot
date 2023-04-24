/**
 ******************************************************************************
 * @file    ChassisSolver.c
 * @brief   底盘解算器
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChassisSolver.h"

ChassisSolver chassis_solver;

void DJIKeyMouseUpdate(ChassisSolver *infantry)
{
    //记得设置好接口可以跳转回遥控器模式

    // 按键检测
    volatile uint16_t keyValue = remote_controller.dji_remote.keyValue;
    remote_controller.dji_remote.keyChangeOn = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
                                               remote_controller.dji_remote.keyValue;
    remote_controller.dji_remote.keyChangeOff = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
                                                remote_controller.dji_remote.last_keyValue;
    remote_controller.dji_remote.last_keyValue = keyValue;
}

void blueToothStateUpdate(ChassisSolver *infantry)
{
    switch (remote_controller.blue_tooth_key)
    {
    case BLUE_TOOTH_UP:
        infantry->target_v = 0.5f;
        break;
    case BLUE_TOOTH_DOWN:
        infantry->target_v = -0.5f;
        break;
    case BLUE_TOOTH_STABLE:
        infantry->target_v = 0;
        infantry->target_yaw_v = 0;
        break;
    case BLUE_TOOTH_LEFT:
        infantry->target_yaw_v += 0.5f;
        break;
    case BLUE_TOOTH_RIGHT:
        infantry->target_yaw_v -= 0.5f;
        break;
    default:
        break;
    }
    remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
}

void DJIRemoteUpdate(ChassisSolver *infantry)
{
    //判断状态
    if (offline_detector.remote_state == REMOTE_OFF || remote_controller.dji_remote.rc.s[RIGHT_SW] == Down)
    {
        setRobotState(OFFLINE_MODE);
        setControlModeAction(NOT_CONTROL_MODE);
        infantry->target_v = 0;
        infantry->target_yaw_v = 0;
        infantry->target_gimbal_yaw = QEKF_INS.YawTotalAngle;
    }
    //右拨杆
    // else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Up)
    // {
    //     //底盘
    //     setRobotState(CONTROL_MODE);
    //     setControlModeAction(BALANCE);

    //     //云台
    //     setShootAction(SHOOT_POWERDOWN_MODE);
    //     setGimbalAction(GIMBAL_POWERDOWN);
    //     infantry->target_v = 0;
    //     infantry->target_yaw_v = SPEED_W_MAX;
    //     infantry->target_gimbal_yaw = QEKF_INS.YawTotalAngle;
    // }
    else if (remote_controller.dji_remote.rc.s[LEFT_SW] == Mid && remote_controller.dji_remote.rc.s[RIGHT_SW] == Up)
    {
        setRobotState(OFFLINE_MODE);
        setControlModeAction(NOT_CONTROL_MODE);
        setShootAction(SHOOT_POWERDOWN_MODE);
        setGimbalAction(GIMBAL_POWERDOWN);
    }
    else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Mid && remote_controller.dji_remote.rc.s[LEFT_SW] == Mid)
    {
        setRobotState(CONTROL_MODE);
        setControlModeAction(BALANCE);
        setShootAction(SHOOT_POWERDOWN_MODE);
        setGimbalAction(GIMBAL_POWERDOWN);

        //对于底盘(在底盘控制的)
        infantry->target_v = (remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * SPEED_MAX / CH_RANGE;
        // infantry->target_yaw_v = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * SPEED_W_MAX / CH_RANGE;
        // infantry->leg_speed = (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_LEG_SPEED / CH_RANGE;
        gimbal_controller.target_yaw_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;

        //在云台控制的
        gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
    }
    else if (remote_controller.dji_remote.rc.s[RIGHT_SW] == Mid && remote_controller.dji_remote.rc.s[LEFT_SW] == Down)
    {
        setRobotState(OFFLINE_MODE);
        setControlModeAction(NOT_CONTROL_MODE);
        infantry->target_v = 0;
        // infantry->target_yaw_v = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * SPEED_W_MAX / CH_RANGE;
        infantry->leg_speed = 0;
        infantry->target_gimbal_yaw = QEKF_INS.YawTotalAngle;
    }
}

/**
 * @brief 根据遥控器或者蓝牙更新控制状态
 * @param[in] infantry
 */
void get_control_info(ChassisSolver *infantry)
{
    switch (remote_controller.control_type)
    {
    case BLUE_TOOTH:
        blueToothStateUpdate(infantry);
        break;
    case DJI_REMOTE_CONTROL:
        DJIRemoteUpdate(infantry);
        break;
    case KEY_MOUSE:
        DJIKeyMouseUpdate(infantry);
    default:
        break;
    }
}
