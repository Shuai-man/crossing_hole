/**
 ******************************************************************************
 * @file    ChasisControlTask.c
 * @brief   底盘控制任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChassisControlTask.h"

uint8_t cnt;

/**
 * @brief 底盘控制任务
 * @param[in] void
 */
void ChassisControl_task(void const * argument)
{
    portTickType xLastWakeTime;

//    vTaskDelay(500);

    infantry.delta_t = DWT_GetDeltaT(&infantry.last_cnt);
    InfantryInit(&infantry);

    // 初始化时间差
    infantry.delta_t = DWT_GetDeltaT(&infantry.last_cnt);
    vTaskDelay(1);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        // 获取两帧之间时间差
        infantry.delta_t = DWT_GetDeltaT(&infantry.last_cnt);

        get_sensors_info(&infantry.sensors_info);

        set_robot_speed(&infantry);	//设置功率和最大速度

        wheels_accel(&infantry);	//处理遥控器信号

        main_control(&infantry);	//解算出电机电流值

        wheels_power_limit(&infantry);	//限制输出功率

        /* 执行控制 */
        execute_control(&infantry.excute_info);	//把电流值发送给电机

        /*  喂狗 */
//        xEventGroupSetBits(xCreatedEventGroup, CHASIS_CONTROL_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
