#ifndef GIMBAL_ESTIMATE_TASK_H
#define GIMBAL_ESTIMATE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_dwt.h"
#include "ins.h"

void GimbalEstimate_task(void const * argument);

#endif 
