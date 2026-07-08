#include "HeatControl.h"

HeatController heat_controller;
//计算剩余发弹量，判断是否打弹
void HeatUpdate(void)
{
    if (heat_controller.heat_count != heat_controller.last_heat_count)//裁判系统更新
    {
        heat_controller.last_heat_count = heat_controller.heat_count;
        heat_controller.available_shoot = (heat_controller.HeatMax - heat_controller.CurHeat) / ONE_BULLET_HEAT;
        heat_controller.LastHeat = heat_controller.CurHeat;

        heat_controller.last_shoot_count = heat_controller.shoot_count;//弹丸发射后更新 (10hz更新)
    }
    if(heat_controller.shoot_count > heat_controller.last_shoot_count)//弹丸发射
    {
        heat_controller.shoot_flag = 1;
        heat_controller.last_shoot_count = heat_controller.shoot_count;
    }
    else
    {
        heat_controller.shoot_flag = 0;
    }

}
