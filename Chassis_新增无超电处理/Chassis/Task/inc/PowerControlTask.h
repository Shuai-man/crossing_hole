#ifndef _POWER_CONTROL_TASK_H
#define _POWER_CONTROL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//#include "ina260.h"
#include "Referee.h"
#include "SuperPower.h"
#include "bsp_dwt.h"
#include "NingCap.h"
#include "wireless.h"
#include "ChassisController.h"


void PowerControlTask(void const * argument);

#endif // !_POWER_CONTROL_TASK_H
