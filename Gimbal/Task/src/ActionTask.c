#include "ActionTask.h"

#include "KeyMouse.h"
#include "remote_control.h"
/**
 * @brief 状态切换任务,并处理遥控器发送来的数据,将状态信息发送给底盘stm32
 * @param[in] void
 */
void Action_Task(void const *argument)
{
    portTickType xLastWakeTime;
    KeyMouse_Init();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    xLastWakeTime = xTaskGetTickCount(); // 延时后再开启计数

    while (1)
    {

        chassis_solver.delta_t = DWT_GetDeltaT(&chassis_solver.last_cnt);

        RemoteGet(); // 更新数据

        /* 从遥控器或者蓝牙中获取控制信息 */
        get_control_info(&chassis_solver);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    }
}
