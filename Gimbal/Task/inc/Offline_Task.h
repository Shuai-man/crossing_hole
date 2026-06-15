#ifndef _OFFLINE_TASK_H
#define _OFFLINE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "ChassisGet.h"
#include "remote_control.h"
#include "bsp_buzzer.h"


void Offline_task(void const * argument);

#endif // !_OFFLINE_TASK_H
