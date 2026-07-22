#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

#include "can.h"

void can_filter_init(void);

void CanSend(CAN_HandleTypeDef *hcan,
             const uint8_t *data,
             uint32_t std_id,
             uint8_t data_length);
#endif
