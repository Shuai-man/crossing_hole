#include "GimbalSend.h"

#include "Referee.h"
#include "HeatControl.h"

#define ONE_BULLET_HEAT 10

GimbalSendPack_1 gimbal_pack_send_1;
GimbalSendPack_2 gimbal_pack_send_2;

void GimbalSendPack()
{
	if(Robot_Status.current_HP==0)
	{
		gimbal_pack_send_1.alive_flag=0;
	}
	else
	{
		gimbal_pack_send_1.alive_flag=1;
	}
  gimbal_pack_send_1.robot_color = Robot_Status.robot_id < 10 ? 1 : 0;
  gimbal_pack_send_1.robot_level = Robot_Status.robot_level;
	
  gimbal_pack_send_1.shoot_avaiable = heat_controller.available_shoot;
  if(heat_controller.shoot_flag)
  {
    gimbal_pack_send_1.shoot_speed = Shoot.initial_speed;
  }
  else
  {
    gimbal_pack_send_1.shoot_speed = 0;
  }
  gimbal_pack_send_1.heat_cooling = heat_controller.HeatCool;
}
