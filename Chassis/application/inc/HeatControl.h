#ifndef _HEAT_CONTROL_H
#define _HEAT_CONTROL_H

#include "stdint.h"

#define ONE_BULLET_HEAT 10

typedef struct HeatController
{
    uint32_t dwt;

    int heat_count;
    int shoot_count;
    int last_heat_count;  // 上一次更新时的热量次数
    int last_shoot_count; // 上一次更新热量时的发射子弹数

    int16_t HeatMax;  // 热量上限
    uint16_t HeatCool; // 枪口冷却恢复速度
    int16_t CurHeat;  // 当前热量
    int16_t LastHeat; // 上一次更新时的热量

    int16_t available_shoot;         // 可发射数量

    uint8_t shoot_flag; // 打击标志位
} HeatController;

extern HeatController heat_controller;

void HeatUpdate(void);

#endif // !_HEAT_CONTROL_H
