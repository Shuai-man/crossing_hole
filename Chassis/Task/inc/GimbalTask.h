#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#include "can_config.h"
#include "GM6020.h"
#include "M2006.h"
#include "bsp_can.h"

#include "HeatControl.h"
#include "GimbalSend.h"

#include "SystemIdentification.h"

void GimbalTask(void const * argument);

#endif // !_GIMBAL_TASK_H
