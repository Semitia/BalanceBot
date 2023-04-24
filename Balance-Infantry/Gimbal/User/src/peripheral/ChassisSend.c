#include "ChassisSend.h"

ChassisSendPack1 chassis_send_pack1;
ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode()
{
    chassis_send_pack1.infantry_mode = (remote_controller.robot_state << ROBOT_STATE_BIAS) |
                                       (remote_controller.control_type << CONTROL_TYPE_BIAS) |
                                       (remote_controller.control_mode_action << CONTROL_MODE_ACTION_BIAS) |
                                       (remote_controller.gimbal_action << GIMBAL_MODE_BIAS) |
                                       (remote_controller.shoot_action << SHOOT_MODE_BIAS);

    chassis_send_pack1.left_ch_ud_value = remote_controller.dji_remote.rc.ch[LEFT_CH_UD];
    chassis_send_pack1.right_ch_lr_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_LR];
    chassis_send_pack1.right_ch_ud_value = remote_controller.dji_remote.rc.ch[RIGHT_CH_UD];
}

void Pack_YawPitch()
{
    chassis_send_pack2.target_yaw = chassis_solver.target_gimbal_yaw;
    chassis_send_pack2.yaw = QEKF_INS.YawTotalAngle;
}
