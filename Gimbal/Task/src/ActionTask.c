#include "ActionTask.h"

/**
 * @brief 状态切换任务,并处理遥控器发送来的数据,将状态信息发送给底盘stm32
 * @param[in] void
 */
void Action_Task(void const * argument)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 4; // 250HZ

    vTaskDelay(100);
		KeyMouse_Init();

    DWT_GetDeltaT(&chassis_solver.last_cnt); // 时间初始化
    vTaskDelay(1);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        chassis_solver.delta_t = DWT_GetDeltaT(&chassis_solver.last_cnt);

        RemoteGet();//更新数据
			
        /* 从遥控器或者蓝牙中获取控制信息 */
        get_control_info(&chassis_solver);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
