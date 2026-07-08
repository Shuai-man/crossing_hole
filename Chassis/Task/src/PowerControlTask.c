#include "PowerControlTask.h"
float Interval;
uint32_t timtim;

static void Check_Energy_State(void)
{
	if(referee_data.Buff_Musk.remaining_energy & 0x01)
	{
		infantry.energy_state = ENERGY_125;//>125
	}else if(referee_data.Buff_Musk.remaining_energy & 0x02)
	{
		infantry.energy_state = ENERGY_100;//>100
	}else if(referee_data.Buff_Musk.remaining_energy & 0x04)
	{
		infantry.energy_state = ENERGY_50;//>50
	}else if(referee_data.Buff_Musk.remaining_energy & 0x08)
	{
		infantry.energy_state = ENERGY_30;//>30
	}else if(referee_data.Buff_Musk.remaining_energy & 0x010)
	{
		infantry.energy_state = ENERGY_15;//>15
	}else if (referee_data.Buff_Musk.remaining_energy & 0x020)
	{
		infantry.energy_state = ENERGY_5;//>5
	}else if(referee_data.Buff_Musk.remaining_energy & 0x040)
	{
		infantry.energy_state = ENERGY_1;//>1
	}else
	{
		infantry.energy_state = ENERGY_0;//耗尽
	}
}

void PowerControlTask(void const * argument)
{
	portTickType xLastWakeTime;

	static int i = 0;

	CapControllerInit();

	float referee_power;

	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();

		// TODO:首先进行异常处理，万一不能收到裁判系统数据或者裁判系统数据离线
		Check_Energy_State();
		if(infantry.energy_state == ENERGY_0)
		{
			referee_power = LIMIT_MAX_MIN(referee_data.Game_Robot_State.chassis_power_limit, 100, 40);
		}else
		{
			referee_power = LIMIT_MAX_MIN(referee_data.Game_Robot_State.chassis_power_limit, 100, 40);
		}

	  if (remote_controller.super_power_state == POWER_TO_SuperPower)
		{
			NingCapControl(referee_data.Power_Heat_Data.buffer_energy, referee_power, referee_power + 60.0f);
		}
		//else if(gimbal_receiver_pack1.through_hole_flag)
		//{
		//	NingCapControl(referee_data.Power_Heat_Data.buffer_energy, referee_power, 45.0f);
		//}
		else
		{
			NingCapControl(referee_data.Power_Heat_Data.buffer_energy, referee_power, referee_power);//一般进入这
		}

		// 测试
//		NingCapControl(referee_data.Power_Heat_Data.buffer_energy, referee_power, 250.0f);

		if (i % 4 == 0) // 250HZ
		{
			SendCapPack(&cap_send_data, cap_controller.cap_power);
			Interval = DWT_GetDeltaT(&timtim);
			CanSend(SUPER_POWER_CAN, (uint8_t *)(&cap_send_data), SEND_TO_SUPER_POWER_CAN_ID, 8);
			
		}
		
		i++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}
