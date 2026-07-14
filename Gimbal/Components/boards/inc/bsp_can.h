#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"
#include "can_config.h"
#include "Motor_Typdef.h"

#include "FrictionWheel.h"
#include "Gimbal.h"
#include "debug.h"
#include "ToggleBullet.h"
#include "ChassisGet.h"
#include "lifting_control.h"
#include "encoder.h"

void can_filter_init(void);

void CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id);
#endif
