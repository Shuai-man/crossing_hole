#include "HeatControl.h"

#include "Referee.h"
#include "debug.h"

HeatController heat_controller;
// 计算剩余发弹量，判断是否打弹
void HeatUpdate(void)
{
    if (global_debugger.referee_debugger.state == ON) // 裁判系统更新
    {
        heat_controller.HeatMax = Robot_Status.shooter_barrel_heat_limit;
        heat_controller.CurHeat = Power_Heat_Data.shooter_17mm_barrel_heat;
        heat_controller.available_shoot = (heat_controller.HeatMax - heat_controller.CurHeat) / ONE_BULLET_HEAT;
        heat_controller.HeatCool = ONE_BULLET_HEAT/Robot_Status.shooter_barrel_cooling_value*100.0f;
    }
    if (heat_controller.shoot_count > heat_controller.last_shoot_count) // 弹丸发射
    {
        heat_controller.shoot_flag = 1;
        heat_controller.last_shoot_count = heat_controller.shoot_count;
    }
    else
    {
        heat_controller.shoot_flag = 0;
    }
}
