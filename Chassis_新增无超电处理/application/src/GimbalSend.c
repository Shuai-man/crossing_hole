#include "GimbalSend.h"

GimbalSendPack_1 gimbal_pack_send_1;
GimbalSendPack_2 gimbal_pack_send_2;

void GimbalSendPack()
{
	if(referee_data.Game_Robot_State.remain_HP==0)
	{
		gimbal_pack_send_1.alive_flag=0;
	}
	else
	{
		gimbal_pack_send_1.alive_flag=1;
	}
  gimbal_pack_send_1.robot_color = referee_data.Game_Robot_State.robot_id < 10 ? 1 : 0;
  gimbal_pack_send_1.robot_level = referee_data.Game_Robot_State.robot_level;
	
  gimbal_pack_send_1.shoot_avaiable = heat_controller.available_shoot;
  if(heat_controller.shoot_flag)
  {
    gimbal_pack_send_1.shoot_speed = referee_data.Shoot_Data.bullet_speed;
  }
  else
  {
    gimbal_pack_send_1.shoot_speed = 0;
  }
}
