#include "ChassisGet.h"

ChassisGetPack_1 chassis_pack_get_1;
ChassisGetPack_2 chassis_pack_get_2;
RefereeCheck referee_check;


void Referee_Check(void)
{
  //前三个都是0，说明裁判系统未连接
  if(chassis_pack_get_1.alive_flag + chassis_pack_get_1.robot_color + chassis_pack_get_1.robot_level == 0)
  {
    referee_check.is_connected = 0;//离线
  }
  else
  {
    referee_check.is_connected = 1;//在线
  }
}
