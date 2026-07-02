#include "Offline_Task.h"

OfflineDetector offline_detector;

void Offline_task(void const *argument)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // 待机器人初始化后开始检测

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

        vTaskDelay(pdMS_TO_TICKS(1000)); // 所有数据都应该超过5HZ
    }
}
