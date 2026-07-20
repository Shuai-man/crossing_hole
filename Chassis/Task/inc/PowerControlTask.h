#ifndef _POWER_CONTROL_TASK_H
#define _POWER_CONTROL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Referee.h"

#include "bsp_dwt.h"
#include "NingCap.h"
#include "wireless.h"
#include "ChassisController.h"

#define POWER_LIMIT_MAX 125.0f //无线充电突破到125w，常态最高100w
#define POWER_LIMIT_MIN 35.0f //节能模式35w，常态最低45w

void PowerControlTask(void const * argument);

#endif // !_POWER_CONTROL_TASK_H
