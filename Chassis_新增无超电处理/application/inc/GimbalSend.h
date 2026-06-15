#ifndef _GIMBAL_SEND_H
#define _GIMBAL_SEND_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "HeatControl.h"

#pragma pack(push, 1)

typedef struct GimbalSendPack_1
{
//uint8_t占1个字节，float占4个	
  uint8_t alive_flag : 1;     // 存活标志位
  uint8_t robot_color : 1;      // 机器人颜色，红色为1，蓝色为0
  uint8_t robot_level : 6;     // 机器人等级
 
	float shoot_speed;						//发射初速度 25上限
	int8_t shoot_avaiable;				//剩余发弹量

  uint8_t reverse[2] ;            // 保留位

} GimbalSendPack_1;

typedef struct GimbalSendPack_2
{
	uint8_t reverse[8];
} GimbalSendPack_2;


#pragma pack(pop)

extern GimbalSendPack_1 gimbal_pack_send_1;
extern GimbalSendPack_2 gimbal_pack_send_2;

void GimbalSendPack(void);

#endif // !_GIMBAL_SEND_H
