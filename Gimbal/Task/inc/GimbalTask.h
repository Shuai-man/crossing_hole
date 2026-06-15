#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "Gimbal.h"
#include "FrictionWheel.h"
#include "can_config.h"
#include "bsp_can.h"
#include "DM_Motor.h"
#include "Motor_Typdef.h"
#include "ChassisSolver.h"

#include "M3508.h"
#include "Offline_Task.h"

#include "SystemIdentification.h"
#include "SignalGenerator.h"
#include "usb_device.h"

#include "pc_serial.h"
#include "debug.h"

#include "lifting_control.h"

#define GYRO_PITCH_BIAS 0.0f

#define GIMBAL_TEST 1

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_TASK_H
