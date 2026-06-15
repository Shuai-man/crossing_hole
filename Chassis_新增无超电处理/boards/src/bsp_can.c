#include "bsp_can.h"

/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/

void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	// CAN 1 FIFO0 接收中断
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN1_FIFO0_ID0 << 5;
	can_filter_st.FilterIdLow = CAN1_FIFO0_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN1_FIFO0_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN1_FIFO0_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	// CAN 1 FIFO1 接收中断
	can_filter_st.FilterBank = 2;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN1_FIFO1_ID0 << 5;
	can_filter_st.FilterIdLow = CAN1_FIFO1_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN1_FIFO1_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN1_FIFO1_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	// CAN 2 FIFO0 接收中断
	can_filter_st.FilterBank = 15;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN2_FIFO0_ID0 << 5;
	can_filter_st.FilterIdLow = CAN2_FIFO0_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN2_FIFO0_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN2_FIFO0_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	// CAN 2 FIFO1 接收中断
	can_filter_st.FilterBank = 16;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
	can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
	can_filter_st.FilterIdHigh = CAN2_FIFO1_ID0 << 5;
	can_filter_st.FilterIdLow = CAN2_FIFO1_ID1 << 5;
	can_filter_st.FilterMaskIdHigh = CAN2_FIFO1_ID2 << 5;
	can_filter_st.FilterMaskIdLow = CAN2_FIFO1_ID3 << 5;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	// CAN 1 发送中断
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	// CAN 2 发送中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
}

void CanReceiveAll(CAN_HandleTypeDef *hcan,  CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (hcan == DJI_WHEELS_CAN)
    {
        switch (rx_header->StdId)
        {
        case DJI_3508_MOTORS_1:
            infantry.sensors_info.wheels_recv[0].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.wheels_recv[0].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.wheels_recv[0].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.wheels_recv[0].temp = data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[0], 0.0045f);
            break;
        case DJI_3508_MOTORS_2:
            infantry.sensors_info.wheels_recv[1].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.wheels_recv[1].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.wheels_recv[1].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.wheels_recv[1].temp = data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[1], 0.0045f);
            break;
        case DJI_3508_MOTORS_3:
            infantry.sensors_info.wheels_recv[2].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.wheels_recv[2].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.wheels_recv[2].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.wheels_recv[2].temp = data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[2], 0.0045f);
            break;
        case DJI_3508_MOTORS_4:
            infantry.sensors_info.wheels_recv[3].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.wheels_recv[3].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.wheels_recv[3].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.wheels_recv[3].temp = data[6];
            LossUpdate(&global_debugger.wheels_comm_debugger[3], 0.0045f);
            break;
        default:
            break;
        }
    }
    if (hcan == DJI_STEERS_CAN)
    {
        switch (rx_header->StdId)
        {
        case DJI_6020_MOTORS_1:
            infantry.sensors_info.steer_recv[0].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.steer_recv[0].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.steer_recv[0].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.steer_recv[0].temp = data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[0], 0.0045f);
            break;
        case DJI_6020_MOTORS_2:
            infantry.sensors_info.steer_recv[1].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.steer_recv[1].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.steer_recv[1].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.steer_recv[1].temp = data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[1], 0.0045f);
            break;
        case DJI_6020_MOTORS_3:
            infantry.sensors_info.steer_recv[2].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.steer_recv[2].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.steer_recv[2].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.steer_recv[2].temp = data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[2], 0.0045f);
            break;
        case DJI_6020_MOTORS_4:
            infantry.sensors_info.steer_recv[3].angle = (data[0] << 8) | (data[1]);
            infantry.sensors_info.steer_recv[3].speed = (data[2] << 8) | (data[3]);
            infantry.sensors_info.steer_recv[3].torque_current = (data[4] << 8) | (data[5]);
            infantry.sensors_info.steer_recv[3].temp = data[6];

            LossUpdate(&global_debugger.steers_comm_debugger[3], 0.0045f);
            break;
        default:
            break;
        }
    }

    if (hcan == GIMBAL_CAN_COMM_CANx)
    {
        switch (rx_header->StdId)
        {
        case GIMBAL_COMM_CAN_ID_1:
            memcpy(&gimbal_receiver_pack1, data, 8);
            Gimbal_msgs_Decode1();
            LossUpdate(&global_debugger.gimbal_comm_debugger[0], 0.0085f);
            break;
        case GIMBAL_COMM_CAN_ID_2:
            memcpy(&gimbal_receiver_pack2, data, 8);
						Gimbal_msgs_Decode2();
            LossUpdate(&global_debugger.gimbal_comm_debugger[1], 0.0015f);
            break;
        default:
            break;
        }
    }

    if (hcan == SUPER_POWER_CAN)
    {
        if (rx_header->StdId == SUPER_POWER_CAN_ID)
        {
            memcpy(&cap_recv_data, data, 8);
            LossUpdate(&global_debugger.super_power_debugger, 0.0015f);
        }
    }
	
//	if (hcan == WIRELESS_CAN)
//	{
//		if (rx_header->StdId == WIRELESS_CAN_ID)
//        {
//            memcpy(&wireless_recv_data,data, 8);
////            LossUpdate(&global_debugger.wireless_debugger, 0.0015f);
//        }
//	}
}


/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 0邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	CanReceiveAll(hcan, &rx_header, rx_data);

}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo1MsgPendingCallback
 *功能说明:FIFO 1邮箱中断回调函数
 *形    参:
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // FIFO 1邮箱中断回调函数
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	CanReceiveAll(hcan, &rx_header, rx_data);

}


void CanSend(CAN_HandleTypeDef *hcan, uint8_t *data, uint32_t std_id, uint8_t data_length)
{
  CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8]; 
	uint32_t send_mail_box;
	
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = data_length;
  tx_header.StdId = std_id;

  memcpy(tx_data, data, data_length);
  HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}
