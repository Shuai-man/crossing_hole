/**
 ******************************************************************************
 * @file    Judge.c
 * @brief   云台数据接收
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalReceive.h"
#include "ChassisController.h"

GimbalReceivePack1 gimbal_receiver_pack1;
GimbalReceivePack2 gimbal_receiver_pack2;
int8_t gimbal_receive_1_update; // 更新标志，说明收到了一帧消息
int8_t gimbal_receive_2_update;
int8_t gimbal_receive_3_update;
int16_t jump_up_cnt = 0;

float transition_mode_counter; // 计时器

void Gimbal_msgs_Decode1()
{
  enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
  enum CONTROL_TYPE contro_type = (enum CONTROL_TYPE)gimbal_receiver_pack1.control_type;
  enum CHASSIS_MODE_ACTION chassis_mode_action = (enum CHASSIS_MODE_ACTION)gimbal_receiver_pack1.chassis_mode_action;
  enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
	enum GIMBAL_POSITION gimbal_position = (enum GIMBAL_POSITION)gimbal_receiver_pack1.gimbal_position;
  enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack1.super_power;
	
  setRobotState(robot_state);
  setControlMode(contro_type);
  setControlModeAction(chassis_mode_action);
  setGimbalAction(gimbal_action);
  setSuperPower(power_state);
	setGimbalPosition(gimbal_position);

  if (infantry.yaw_motor_type == YAW_DM_MOTOR)
  {
    float DM_P_MAX = 65536.0f;
    infantry.yaw_radian = gimbal_receiver_pack1.yaw_pose / DM_P_MAX * PI_2;
    infantry.yaw_angle = gimbal_receiver_pack1.yaw_pose / DM_P_MAX * 360.0f;
  }
	
  // 运动百分比 //不改电机id只能先改接收来调方向
  infantry.target_yaw_v_percent = gimbal_receiver_pack1.robot_speed_w / 100.0f;
  infantry.target_y_v_percent = gimbal_receiver_pack1.robot_speed_x / 100.0f;
  infantry.target_x_v_percent = gimbal_receiver_pack1.robot_speed_y / 100.0f;
}

void Gimbal_msgs_Decode2()
{

}
