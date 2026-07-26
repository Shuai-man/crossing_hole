#include "ChassisSend.h"

#include "remote_control.h"
#include "ins.h"
#include "Gimbal.h"
#include "KeyMouse.h"
#include "pc_serial.h"
#include "FrictionWheel.h"
#include "debug.h"

ChassisSendPack1 chassis_send_pack1;
ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode(void)
{
  chassis_send_pack1.robot_state = remote_controller.robot_state;
  chassis_send_pack1.control_type = remote_controller.control_type;
  chassis_send_pack1.chassis_mode_action = remote_controller.chassis_mode_action;
  chassis_send_pack1.gimbal_mode = remote_controller.gimbal_action;
  chassis_send_pack1.super_power = remote_controller.super_power_state;
	if(global_debugger.pc_receive_debugger.state == ON)
	{
	  chassis_send_pack1.is_pc_on = 1;	
	}
	else
	{
		chassis_send_pack1.is_pc_on = 0;
	}
  chassis_send_pack1.aim_mode = pc_send_data.mode_want;
	chassis_send_pack1.gimbal_position = remote_controller.gimbal_position;
  chassis_send_pack1.yaw_pose = gimbal_controller.DM_Yaw_Motor.P_Receive;
  chassis_send_pack1.robot_speed_x = (int8_t)(chassis_solver.chassis_speed_x * 100.0f);
  chassis_send_pack1.robot_speed_y = (int8_t)(chassis_solver.chassis_speed_y * 100.0f);
  chassis_send_pack1.robot_speed_w = (int8_t)(chassis_solver.chassis_speed_w * 100.0f);
  chassis_send_pack1.set_friction_speed = friction_wheels.friction_speed *10.0f;
}

void Pack_Chassis2(void)
{
  float yaw_speed_target = 0.0f;

  if (remote_controller.chassis_mode_action == FOLLOW_GIMBAL)
  {
    if (remote_controller.gimbal_position == DOWN)
    {
      yaw_speed_target = CHASSIS_THROUGH_HOLE_YAW_SPEED_FF_COEF * gimbal_controller.pos_yaw_td.dx;
    }
    else
    {
      yaw_speed_target = CHASSIS_FOLLOW_YAW_SPEED_FF_COEF * gimbal_controller.pos_yaw_td.dx;
    }
  }

  chassis_send_pack2.gimbal_pitch = (int16_t)(gimbal_controller.gyro_pitch_angle * 100.0f);
  chassis_send_pack2.turn_yaw_speed = (int16_t)(yaw_speed_target * 100.0f);
}
