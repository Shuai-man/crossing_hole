#ifndef _GIMBAL_SEND_H
#define _GIMBAL_SEND_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "HeatControl.h"

#pragma pack(push, 1)

typedef struct GimbalSendPack_1
{
    uint8_t is_shootable : 1;     // 热量限制标志位
    uint8_t robot_color : 1;      // 机器人颜色，红色为1，蓝色为0
    uint8_t bullet_level : 2;     // 发射子弹速度上限
    uint8_t buff_state : 4;       // bit 0：机器人血量补血状态 bit 1：枪口热量冷却加速 bit 2：机器人防御加成 bit 3：机器人攻击加成
    uint8_t robot_level : 4;      // 机器人等级
    uint8_t is_balance_robot : 3; // 敌方几号装甲板是平衡步兵,分别对应3，4，5
    uint8_t symbol_flag[4];
} GimbalSendPack_1;

typedef struct GimbalSendPack_2
{
	uint16_t bullet_speed;
	uint8_t available_shoot; //可发射弹丸数量
} GimbalSendPack_2;


#pragma pack(pop)

extern GimbalSendPack_1 gimbal_pack_send_1;
extern GimbalSendPack_2 gimbal_pack_send_2;

void GimbalSendPack(void);

#endif // !_GIMBAL_SEND_H
