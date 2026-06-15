#ifndef _BLUE_TOOTH_TASK_H
#define _BLUE_TOOTH_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#include "bluetooth.h"
#include "ChassisControlTask.h"
#include "PowerLimit.h"
void BlueToothTask(void const * argument);

#endif // !_BLUE_TOOTH_TASK_H
