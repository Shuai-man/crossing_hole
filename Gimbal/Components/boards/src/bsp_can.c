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
	can_filter_st.FilterBank = 1;
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

void DataReceive(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
	if (hcan == PITCH_MOTOR_CAN && rx_header->StdId == PITCH_MOTOR_MASTER_CAN_ID)
	{
		DM_Motor_Receive(data, &gimbal_controller.DM_Pitch_Motor);
		LossUpdate(&global_debugger.pitch_debugger, PITCH_LOSS_TIME);//最低0.0022
	}
	else if (hcan == YAW_MOTOR_CAN && rx_header->StdId == YAW_MOTOR_MASTER_CAN_ID)
	{
		DM_Motor_Receive(data, &gimbal_controller.DM_Yaw_Motor);
		LossUpdate(&global_debugger.yaw_debugger, YAW_LOSS_TIME); // 1KHZ
	}
	else if (hcan == FRICTION_WHEEL_CAN && rx_header->StdId == LEFT_FRICTION_WHEEL_CAN_ID)
	{
		// 左摩擦轮接收
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].angle = (data[0] << 8) | (data[1]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].speed = (data[2] << 8) | (data[3]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].torque_current = (data[4] << 8) | (data[5]);
		friction_wheels.friction_motor_recv[LEFT_FRICTION_WHEEL].temp = (data[6]);

		LossUpdate(&global_debugger.friction_debugger[LEFT_FRICTION_WHEEL], FRICTION_LOSS_TIME);
	}
	else if (hcan == FRICTION_WHEEL_CAN && rx_header->StdId == RIGHT_FRICTION_WHEEL_CAN_ID)
	{
		// 右摩擦轮接收
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].angle = (data[0] << 8) | (data[1]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].speed = (data[2] << 8) | (data[3]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].torque_current = (data[4] << 8) | (data[5]);
		friction_wheels.friction_motor_recv[RIGHT_FRICTION_WHEEL].temp = (data[6]);

		LossUpdate(&global_debugger.friction_debugger[RIGHT_FRICTION_WHEEL], FRICTION_LOSS_TIME);
	}	
	else if (hcan == TOGGLE_MOTOR_CAN && rx_header->StdId == TOGGLE_MOTOR_CAN_ID)
	{
		// 拨弹电机数据接收
		toggle_controller.toggle_recv.angle = (data[0] << 8) | (data[1]);
		toggle_controller.toggle_recv.speed = (data[2] << 8) | (data[3]);
		toggle_controller.toggle_recv.torque_current = (data[4] << 8) | (data[5]);

		LossUpdate(&global_debugger.toggle_debugger, TOGGLE_LOSS_TIME);
	}
	else if (hcan == LIFT_MOTOR_CAN && rx_header->StdId == LIFT_MOTOR_CAN_ID)
	{
		// 丝杆电机数据接收
		lifting_controller.lift_recv.angle = (data[0] << 8) | (data[1]);
		lifting_controller.lift_recv.speed = (data[2] << 8) | (data[3]);
		lifting_controller.lift_recv.torque_current = (data[4] << 8) | (data[5]);

		LossUpdate(&global_debugger.lift_debugger, LIFT_LOSS_TIME);
	}

	else if (hcan == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_FROM_CHASSIS_CAN_ID_1)
	{
		memcpy(&chassis_pack_get_1, data, 8);

		LossUpdate(&global_debugger.receive_chassis_debugger[0], CHASSIS_LOSS_TIME);
	}
	else if (hcan == CHASSIS_CAN_COMM_CANx && rx_header->StdId == GET_FROM_CHASSIS_CAN_ID_2)
	{
		memcpy(&chassis_pack_get_2, data, 8);
		LossUpdate(&global_debugger.receive_chassis_debugger[1], CHASSIS_LOSS_TIME);
	}
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

	DataReceive(hcan, &rx_header, rx_data);
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

	DataReceive(hcan, &rx_header, rx_data);
}

void CanSend(CAN_HandleTypeDef *hcan, int8_t *data, uint32_t std_id)
{
	CAN_TxHeaderTypeDef TxMsg;

	TxMsg.StdId = std_id;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 0x08;
	
	uint8_t send_Data[8];
	memcpy(send_Data, data, 8);
	
	uint32_t send_mail_box;
	HAL_CAN_AddTxMessage(hcan, &TxMsg, send_Data, &send_mail_box);
}
