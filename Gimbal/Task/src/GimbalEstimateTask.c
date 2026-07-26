/**
 ******************************************************************************
 * @file    GimbalEstimateTask.c
 * @brief   云台位姿估计任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalEstimateTask.h"

#include "ins.h"

/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
void GimbalEstimate_task(void const *argument)
{
    portTickType xLastWakeTime;

    BMI088_Read(&BMI088);
    float ax = BMI088.Accel[X];
    float ay = BMI088.Accel[Y];
    float az = BMI088.Accel[Z];
    float pitch = -atan(ax / az);
    float roll = atan(ay / az);
    INS_Init(pitch, roll);

    vTaskDelay(pdMS_TO_TICKS(100));
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        INS_Task();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
