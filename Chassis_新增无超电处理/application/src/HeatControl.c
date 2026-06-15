#include "HeatControl.h"

HeatController heat_controller;
//计算剩余发弹量


void HeatUpdate(void)
{
    heat_controller.HeatMax = referee_data.Game_Robot_State.shooter_barrel_heat_limit; 
		heat_controller.HeatCool = referee_data.Game_Robot_State.shooter_barrel_cooling_value ; //热量冷却,暂时没用
    heat_controller.CurHeat = referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;  
	
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
