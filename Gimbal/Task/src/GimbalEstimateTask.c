/**
 ******************************************************************************
 * @file    GimbalEstimateTask.c
 * @brief   云台位姿估计任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalEstimateTask.h"

/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
void GimbalEstimate_task(void const *argument)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ

    BMI088_Read(&BMI088);
    float ax = BMI088.Accel[X];
    float ay = BMI088.Accel[Y];
    float az = BMI088.Accel[Z];
    float pitch = -atan(ax / az);
    float roll = atan(ay / az);
    INS_Init(pitch, roll);

    // 适当延时
    vTaskDelay(100);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        INS_Task();

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
