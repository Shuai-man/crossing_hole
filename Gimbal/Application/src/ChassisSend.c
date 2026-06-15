#include "ChassisSend.h"

ChassisSendPack1 chassis_send_pack1;
ChassisSendPack2 chassis_send_pack2;



void Pack_InfantryMode()
{
  chassis_send_pack1.robot_state = remote_controller.robot_state;
  chassis_send_pack1.control_type = remote_controller.control_type;
  chassis_send_pack1.chassis_mode_action = remote_controller.chassis_mode_action;
  chassis_send_pack1.gimbal_mode = remote_controller.gimbal_action;
  chassis_send_pack1.super_power = remote_controller.super_power_state;
  chassis_send_pack1.is_pc_on = global_debugger.pc_receive_debugger.state;
  chassis_send_pack1.autoaim_id = pc_recv_data.detect_number;
	chassis_send_pack1.gimbal_position = remote_controller.gimbal_position;
  chassis_send_pack1.yaw_pose = gimbal_controller.DM_Yaw_Motor.P_Receive;
  chassis_send_pack1.robot_speed_x = (int8_t)(chassis_solver.chassis_speed_x * 100.0f);
  chassis_send_pack1.robot_speed_y = (int8_t)(chassis_solver.chassis_speed_y * 100.0f);
  chassis_send_pack1.robot_speed_w = (int8_t)(chassis_solver.chassis_speed_w * 100.0f);
}

void Pack_Chassis2(void)
{
//  chassis_send_pack2.yaw_motor_angle = gimbalMotors.gimbal_motors[YAW].Yaw_msg;
//  chassis_send_pack2.gimbal_pitch = (int16_t)(gimbal_controller.gyro_pitch_angle * 100.0f);
//  chassis_send_pack2.gimbal_yaw_speed = (int16_t)(gimbal_controller.gyro_yaw_speed * 100.0f);
}
