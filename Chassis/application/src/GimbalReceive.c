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

void Gimbal_msgs_Update1(void)
{
  if (global_debugger.gimbal_comm_debugger[0].state == ON)
  {
    enum ROBOT_STATE robot_state = (enum ROBOT_STATE)gimbal_receiver_pack1.robot_state;
    enum CHASSIS_MODE_ACTION chassis_mode_action = (enum CHASSIS_MODE_ACTION)gimbal_receiver_pack1.chassis_mode_action;
    enum GIMBAL_ACTION gimbal_action = (enum GIMBAL_ACTION)gimbal_receiver_pack1.gimbal_mode;
    enum GIMBAL_POSITION gimbal_position = (enum GIMBAL_POSITION)gimbal_receiver_pack1.gimbal_position;
    enum PowerControlState power_state = (enum PowerControlState)gimbal_receiver_pack1.super_power;

    setRobotState(robot_state);
    setChassisAction(chassis_mode_action);
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
  else
  {
    setRobotState(OFFLINE_MODE);
    setChassisAction(NOT_CONTROL_MODE);
    setGimbalAction(GIMBAL_POWERDOWN);
    setSuperPower(POWER_TO_BATTERY);
    setGimbalPosition(POWER_DOWN);  
    infantry.target_yaw_v_percent = 0.0f;
    infantry.target_y_v_percent = 0.0f;
    infantry.target_x_v_percent = 0.0f;
  }
}

void Gimbal_msgs_Update2(void)
{
}
