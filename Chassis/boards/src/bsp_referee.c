/**
 ******************************************************************************
 * @file    bsp_referee.c
 * @brief   裁判系统串口通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "bsp_referee.h"
#include "debug.h"

// 全局变量定义部分
fifo_s_t Referee_FIFO; // 裁判系统接收数据队列
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

unpack_data_t Referee_Unpack_OBJ; // protocol解析包结构体

uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT]; // 裁判系统串口双缓冲区

void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength)
{
	/* 使能串口DMA */
	SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart4.Instance->CR3, USART_CR3_DMAT);

	/* 使能串口空闲中断 */
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

	/* 确保DMA RX失能 */
	while (hdma_uart4_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_uart4_rx);
	}

	/* 清空标志位 */
	__HAL_DMA_CLEAR_FLAG(&hdma_uart4_rx, DMA_LISR_TCIF1);

	/* 设置接收双缓冲区 */
	hdma_uart4_rx.Instance->PAR = (uint32_t)&(huart4.Instance->DR);
	hdma_uart4_rx.Instance->M0AR = (uint32_t)(Buffer0);
	hdma_uart4_rx.Instance->M1AR = (uint32_t)(Buffer1);

	/* 设置数据长度 */
	__HAL_DMA_SET_COUNTER(&hdma_uart4_rx, BufferLength);

	/* 使能双缓冲区 */
	SET_BIT(hdma_uart4_rx.Instance->CR, DMA_SxCR_DBM);

	/* 使能DMA RX */
	__HAL_DMA_ENABLE(&hdma_uart4_rx);

	/* 确保DMA TX失能 */
	while (hdma_uart4_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_uart4_tx);
	}

	hdma_uart4_tx.Instance->PAR = (uint32_t)&(huart4.Instance->DR);
}

/*******************************************************************************************************
DMA双缓冲接收
********************************************************************************************************/
void Referee_Receive(void)
{
	static uint16_t this_time_rx_len = 0;

	if (huart4.Instance->SR & UART_FLAG_RXNE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart4);
	}
	else if (huart4.Instance->SR & UART_FLAG_IDLE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart4);

		if ((hdma_uart4_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_uart4_rx);
			this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_uart4_rx.Instance->NDTR;
			hdma_uart4_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			hdma_uart4_rx.Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_uart4_rx);
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[0], this_time_rx_len);
			LossUpdate(&global_debugger.referee_debugger, 0.2f);
		}
		else
		{
			__HAL_DMA_DISABLE(&hdma_uart4_rx);
			this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_uart4_rx.Instance->NDTR;
			hdma_uart4_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			hdma_uart4_rx.Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(&hdma_uart4_rx);
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[1], this_time_rx_len);
			LossUpdate(&global_debugger.referee_debugger, 0.2f);
		}
	}
}
