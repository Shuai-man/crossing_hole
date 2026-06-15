/**********************************************************************************************************
 * @文件     usart.c
 * @说明     uart初始化,VOFA
 * @版本  	 V1.0
 **********************************************************************************************************/
#include "main.h"

float VOFA_justfloat[VOFA_MAX_CHANNEL];
uint8_t VOFA_send_Data[VOFA_MAX_CHANNEL * 4 + 4];
uint8_t VOFA_recv_Data[RECV_LEN + 5] = {0};

/*
	@brief VOFA串口初始化
	@param none
	@return none
*/
void VOFA_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef dma;

	// 写入结尾数据
	VOFA_send_Data[VOFA_MAX_CHANNEL * 4] = 0x00;
	VOFA_send_Data[VOFA_MAX_CHANNEL * 4 + 1] = 0x00;
	VOFA_send_Data[VOFA_MAX_CHANNEL * 4 + 2] = 0x80;
	VOFA_send_Data[VOFA_MAX_CHANNEL * 4 + 3] = 0x7f;

	nvic.NVIC_IRQChannel = VOFA_DMA_TX_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// USART
	VOFA_USART_GPIO_APBxClock_FUN(VOFA_USART_TX_GPIO_CLK, ENABLE);
	VOFA_USART_GPIO_APBxClock_FUN(VOFA_USART_RX_GPIO_CLK, ENABLE);
	VOFA_USART_APBxClock_FUN(VOFA_USART_CLK, ENABLE);

	GPIO_PinAFConfig(VOFA_USART_TX_PORT, VOFA_USART_TX_SOURCE, VOFA_USART_TX_AF);
	GPIO_PinAFConfig(VOFA_USART_RX_PORT, VOFA_USART_RX_SOURCE, VOFA_USART_RX_AF);

	gpio.GPIO_Pin = VOFA_USART_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_TX_PORT, &gpio);

	gpio.GPIO_Pin = VOFA_USART_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_RX_PORT, &gpio);

	usart.USART_BaudRate = VOFA_USART_BAUD_RATE;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(VOFA_USARTx, &usart);

	// USART_ITConfig(VOFA_USARTx,USART_IT_IDLE, ENABLE); //空闲中断，DMA接收///////////////////////////
	USART_Cmd(VOFA_USARTx, ENABLE);
	USART_DMACmd(VOFA_USARTx, USART_DMAReq_Tx, ENABLE);
	USART_ClearFlag(VOFA_USARTx, USART_FLAG_TC);

	// DMA
	{
		// TX
		VOFA_DMA_TX_AHBxClock_FUN(VOFA_DMA_TX_CLK, ENABLE);
		DMA_DeInit(VOFA_DMA_TX_STREAM);
		dma.DMA_Channel = VOFA_DMA_TX_CHANNEL;
		dma.DMA_PeripheralBaseAddr = (uint32_t) & (VOFA_USARTx->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)VOFA_send_Data;
		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		dma.DMA_BufferSize = (VOFA_MAX_CHANNEL * 4 + 4);
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Normal;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		dma.DMA_MemoryBurst = DMA_Mode_Normal;
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(VOFA_DMA_TX_STREAM, &dma);
		DMA_ClearFlag(VOFA_DMA_TX_STREAM, VOFA_DMA_TX_FLAG_TCIF); // clear all DMA flags
		DMA_ITConfig(VOFA_DMA_TX_STREAM, DMA_IT_TC, ENABLE);	  // open DMA send inttrupt
		DMA_Cmd(VOFA_DMA_TX_STREAM, DISABLE);
	}
}

#if USART_DEVICE_TYPE == VOFA_DEVICE
void VOFA_DMA_TX_INT_FUN(void)
{
	if (DMA_GetITStatus(VOFA_DMA_TX_STREAM, VOFA_DMA_TX_IT_STATUS))
	{
		DMA_ClearFlag(VOFA_DMA_TX_STREAM, DMA_IT_TCIF7);
		DMA_ClearITPendingBit(VOFA_DMA_TX_STREAM, VOFA_DMA_TX_IT_STATUS);
		DMA_Cmd(VOFA_DMA_TX_STREAM, DISABLE);
	}
}
#endif

void VOFA_Send(void)
{
	 VOFA_justfloat[0] = gimbal_controller.set_yaw_speed;
	 VOFA_justfloat[1] = gimbal_controller.set_yaw_angle;
	 VOFA_justfloat[2] = gimbal_controller.gyro_yaw_speed;
	 VOFA_justfloat[3] = gimbal_controller.gyro_yaw_angle;
	 VOFA_justfloat[4] = gimbal_controller.set_yaw_current;
	
	 VOFA_justfloat[5] = gimbal_controller.target_pitch_angle;
	 VOFA_justfloat[6] = gimbal_controller.gyro_pitch_angle;
	 VOFA_justfloat[7] = chassis_solver.chassis_speed_w;
	 VOFA_justfloat[8] = remote_controller.control_mode_action;
	 VOFA_justfloat[9] = pc_recv_data.shoot_flag * 600.0f; // 扩大到原来的600倍

//	VOFA_justfloat[0] = pc_recv_data.pitch_d;
//	VOFA_justfloat[1] = pc_recv_data.yaw_d;
//	VOFA_justfloat[2] = toggle_controller.is_shoot;
//	VOFA_justfloat[3] = pc_recv_data.shoot_flag;
//	VOFA_justfloat[4] = toggle_controller.toggle_recv.speed;
//	VOFA_justfloat[5] = bomb_bay_controller.bomb_bay_info.speed;
//	VOFA_justfloat[6] = toggle_controller.toggle_info.speed;
//	VOFA_justfloat[7] = pc_recv_data.yaw_d;
//	VOFA_justfloat[8] = pc_recv_data.pitch_d;
//	VOFA_justfloat[9] = pc_recv_data.shoot_flag * 600.0f; // 扩大到原来的600倍

	// 将32位的浮点数转换为4个8位的整型
	memcpy(VOFA_send_Data, (uint8_t *)VOFA_justfloat, sizeof(VOFA_justfloat));
	VOFA_DMA_TX_STREAM->NDTR = VOFA_MAX_CHANNEL * 4 + 4;
	DMA_Cmd(VOFA_DMA_TX_STREAM, ENABLE);
}
