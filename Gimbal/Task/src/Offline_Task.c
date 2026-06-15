#include "Offline_Task.h"

uint8_t pc_on_flag=0;
uint8_t friction_off=0;

void Offline_task(void const * argument)
{
    vTaskDelay(5000); // 待机器人初始化后开始检测

	//判断逻辑：没有新数据更新，判断为掉线
	
    while (1)
    {
			//IMU
			LossDetect(&global_debugger.imu_debugger);

			//遥控器
			LossDetect(&global_debugger.DT7_debugger);
			LossDetect(&global_debugger.VTM_debugger);

			//pitch电机
			LossDetect(&global_debugger.pitch_debugger);

			//yaw电机
			LossDetect(&global_debugger.yaw_debugger);

			//摩擦轮电机
			for(int i=0;i<2;i++)
			{
				LossDetect(&global_debugger.friction_debugger[i]);
			}

			//拨弹电机
			LossDetect(&global_debugger.toggle_debugger);

			//抬升电机
			LossDetect(&global_debugger.lift_debugger);

			//tof测距模块
			LossDetect(&global_debugger.tof_debugger);

			//板间通信
			for(int i=0;i<2;i++)
			{
				LossDetect(&global_debugger.receive_chassis_debugger[i]);
			}
			
			//检查裁判系统是否连接
			Referee_Check();
				
			//PC通信
			LossDetect(&global_debugger.pc_receive_debugger);
			if(global_debugger.pc_receive_debugger.state!=ON)
			{	
				pc_on_flag=0;
			  LED_Off(GREEN_LED_Pin);
				LED_On(RED_LED_Pin);
			}
			else
			{						
				if(pc_on_flag==0)
				{
					PC_On();
					pc_on_flag=1;
				}				
				LED_Off(RED_LED_Pin);
				LED_On(GREEN_LED_Pin);								
			}
      

			vTaskDelay(1000); // 所有数据都应该超过5HZ
    }
}
