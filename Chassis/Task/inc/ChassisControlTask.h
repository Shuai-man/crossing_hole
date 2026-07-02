#ifndef _CHASIS_CONTROL_H
#define _CHASIS_CONTROL_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChassisController.h"
#include "bsp_dwt.h"

void ChassisControl_task(void const * argument);

#endif // !_CHASIS_CONTROL_H
