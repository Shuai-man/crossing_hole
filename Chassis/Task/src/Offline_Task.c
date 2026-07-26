#include "Offline_Task.h"

#include "debug.h"

OfflineDetector offline_detector;

void Offline_task(void const *argument)
{
    portTickType last_wake_time;
    vTaskDelay(pdMS_TO_TICKS(1000)); // 待机器人初始化后开始检测

    last_wake_time = xTaskGetTickCount();
    while (1)
    {
        // 6020舵电机
        for (int i = 0; i < 4; i++)
        {
            LossDetect(&global_debugger.steers_comm_debugger[i]);
        }

        // 3508轮电机
        for (int i = 0; i < 4; i++)
        {
            LossDetect(&global_debugger.wheels_comm_debugger[i]);
        }

        // 板间通信
        for (int i = 0; i < 2; i++)
        {
            LossDetect(&global_debugger.gimbal_comm_debugger[i]);
        }
        // 超电板
        LossDetect(&global_debugger.super_power_debugger);

        // 裁判系统
        LossDetect(&global_debugger.referee_debugger);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(200)); // delay时间必须大于losstime，否则会误判为loss
    }
}
