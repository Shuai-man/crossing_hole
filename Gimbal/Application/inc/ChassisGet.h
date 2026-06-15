#ifndef _CHASSIS_GET_H
#define _CHASSIS_GET_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#pragma pack(push, 1)

typedef struct ChassisGetPack_1
{
//uint8_t占1个字节，float占4个	
  uint8_t alive_flag : 1;       // 存活标志位		
  uint8_t robot_color : 1;      // 机器人颜色，红色为1，蓝色为0
	uint8_t robot_level : 6;      // 机器人等级

	float shoot_speed;						//发射初速度 25上限
	int8_t shoot_avaiable;				//剩余发弹量

  uint8_t reverse[2] ;						//保留位
	
} ChassisGetPack_1;

// debug
typedef struct ChassisGetPack_2
{
	uint8_t reverse[8] ; 
	
} ChassisGetPack_2;

#pragma pack(pop)

typedef struct RefereeCheck
{
  uint8_t is_connected;
} RefereeCheck;

/* 检查裁判系统是否连接 */
void Referee_Check(void);

extern ChassisGetPack_1 chassis_pack_get_1;
extern ChassisGetPack_2 chassis_pack_get_2;
extern RefereeCheck referee_check;
#endif // !_CHASSIS_GET_H
