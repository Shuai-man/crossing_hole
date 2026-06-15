#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "can_config.h"
#include "ChassisController.h"
#include "wireless.h"
#include "Gimbalreceive.h"
#include "NingCap.h"

void can_filter_init(void);

void CanSend(CAN_HandleTypeDef *hcan, uint8_t *data, uint32_t std_id, uint8_t data_length);
#endif
