#ifndef _CHASSIS_SEND_H
#define _CHASSIS_SEND_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "remote_control.h"
#include "ins.h"
#include "Gimbal.h"
#include "KeyMouse.h"
#include "pc_serial.h"
#include "FrictionWheel.h"

#pragma pack(push, 1)


typedef struct ChassisSendPack2 //bocchi58: 图传UI额外补丁
{
//  int16_t yaw_motor_angle; // yaw轴电机角度
//  int16_t gimbal_pitch;    // 云台pitch角度
//  int16_t gimbal_yaw_speed;

//	uint16_t super_power : 1;	
//	uint16_t fly_state : 1;	
//	uint16_t friction_speed; //应操作手要求发送摩擦轮转速，便于键鼠模式调节
//	uint8_t aim_id; //辅瞄id
//	uint8_t if_shoot;
//	uint8_t shoot_aim_mode;
//	uint8_t wireless_start_flag;	
	uint8_t reverse[8]; 
} ChassisSendPack2;

typedef struct ChassisSendPack1
{
  uint16_t robot_state : 1;
  uint16_t control_type : 1;
	uint16_t chassis_mode_action : 3;
  uint16_t gimbal_mode : 3;	
	
  uint16_t is_pc_on : 1; 	
  uint16_t autoaim_id : 4; // 自瞄ID//这里还能少几位
	uint16_t gimbal_position : 1; //头部模式
  uint16_t super_power : 1;	
	uint16_t through_hole_flag : 1; //过洞缓速限制功率  	
	
	uint16_t yaw_pose; // 云台yaw轴机械角度
 
  int8_t robot_speed_x; // * 10 描述 x方向为云台正方向
  int8_t robot_speed_y; // * 10描述
  int8_t robot_speed_w;
	
	uint8_t reverse ;
	
} ChassisSendPack1;

#pragma pack(pop)

extern ChassisSendPack1 chassis_send_pack1;
extern ChassisSendPack2 chassis_send_pack2;

void Pack_InfantryMode(void);
void Pack_Chassis2(void);

#endif // !_CHASSIS_SEND_H
