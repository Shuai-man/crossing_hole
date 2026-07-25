#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define GYRO_PITCH_BIAS 0.0f

#define GIMBAL_TEST 1

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_TASK_H
