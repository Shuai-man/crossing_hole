#include "HeatControl.h"

HeatController heat_controller;

void HeatControl(void) //200hz
{
    if (heat_controller.heat_update_flag) // 热量更新
    {
        heat_controller.available_heat = LIMIT_MAX_MIN(heat_controller.HeatMax - heat_controller.CurHeat + heat_controller.HeatCool, heat_controller.HeatMax, 0);
        heat_controller.available_shoot = heat_controller.available_heat / ONE_BULLET_HEAT;
    }
    heat_controller.shoot_flag = heat_controller.available_shoot > heat_controller.shoot_count - heat_controller.last_shoot_count ? 1 : 0;
}


float ShootTime = 0.0f;
float shoot_speed = 0.0f;
uint16_t shoot_time = 0;
void HeatUpdate(void)
{
		//扣除一发热量，防止超
    heat_controller.HeatMax = referee_data.Game_Robot_State.shooter_barrel_heat_limit - ONE_BULLET_HEAT * REMANENT_OF_BULLETS; // 保留的子弹热量
	heat_controller.HeatCool = referee_data.Game_Robot_State.shooter_barrel_cooling_value / 20;  //发射频率为20hz
    //heat_controller.HeatCool = referee_data.Game_Robot_State.shooter_barrel_cooling_value / 50; //发射频率为50hz
    heat_controller.CurHeat = referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;  //发射机构的枪口热量
	
    if (heat_controller.heat_count != heat_controller.last_heat_count)//固定10HZ更新
    {
        heat_controller.last_heat_count = heat_controller.heat_count;
        heat_controller.heat_update_flag = 1;
        heat_controller.last_shoot_count = heat_controller.shoot_count;//弹丸发射后更新 (10hz更新)
    }
	
	shoot_time++;//200hz

    HeatControl();

    heat_controller.heat_update_flag = 0; // 已处理完本次热量更新
}

//根据弹频选择可持续设计的时间
void new_heat_control(void)
{
	heat_controller.HeatCool = (float)(referee_data.Game_Robot_State.shooter_barrel_cooling_value); //机器人当前冷却
	heat_controller.HeatMax = (float)(referee_data.Game_Robot_State.shooter_barrel_heat_limit - referee_data.Power_Heat_Data.shooter_id1_17mm_cooling_heat);//枪口热量上限
	
	ShootTime = (10*heat_controller.HeatMax -heat_controller.HeatCool)/10.0f*(ONE_BULLET_HEAT*SHOOT_FREQUENCY - heat_controller.HeatCool); //可持续发射时间 s
	
	//当没有射击时，将shoot_time置零
	if(gimbal_receiver_pack2.if_shoot == 0)
		shoot_time = 0;
	
	if(shoot_time*0.005 < ShootTime) //未达到可持续射击时间上限
	{
		heat_controller.shoot_flag = 1;
	}else{
		heat_controller.shoot_flag = 0;
	}
	
	//计算可剩余发弹量
	heat_controller.available_shoot = (short)(ShootTime*SHOOT_FREQUENCY - shoot_time*0.005*SHOOT_FREQUENCY);//射击时间上限*弹频 - 已射击时间*弹频
}

