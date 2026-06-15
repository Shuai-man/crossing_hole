/**
 ******************************************************************************
 * @file    bsp_referee.c
 * @brief   裁判系统串口通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "bsp_referee.h"

/*  数据定义  */
uint8_t Refereebuffer[REFEREE_RECVBUF_SIZE] = {0};
uint8_t SendToReferee_Buff[REFEREE_SENDBUF_SIZE] = {0};

/**
 * @brief 裁判系统通信
 * @param[in] void
 */


void REFEREE_SendBytes(uint8_t *data, uint8_t len)
{
//	while(HAL_DMA_GetState(&hdma_uart4_tx) == HAL_DMA_STATE_BUSY){};
    memcpy(SendToReferee_Buff, data, len);
		HAL_UART_Transmit_DMA(&huart4, SendToReferee_Buff, len); 
}

//void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
//{
//    // 判断是否是UART4 TX的DMA中断
//    if(hdma == &hdma_uart4_tx)
//    {
//        __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx, DMA_FLAG_TCIF0_4);
//        
//    }
//}
