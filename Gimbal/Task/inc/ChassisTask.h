#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ChassisSolver.h"
#include "bsp_can.h"
#include "can_config.h"

#include "ChassisSend.h"

void Chassis_Task(void const * argument);

#endif // !_CHASSIS_TASK_H
