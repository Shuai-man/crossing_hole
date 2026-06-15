#include "bluetooth.h"

unsigned char BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE];



//void BLUE_TOOTH_IRQHandler(void) // 串口1中断服务程序
//{
//    uint16_t blue_tooth_recv_data;
//    if (USART_GetITStatus(BLUE_TOOTH_UARTx, USART_IT_RXNE) != RESET)
//    {
//        USART_ClearITPendingBit(BLUE_TOOTH_UARTx, USART_IT_RXNE);
//        blue_tooth_recv_data = USART_ReceiveData(BLUE_TOOTH_UARTx);
//        Blue_Tooth_Deal(&blue_tooth_recv_data);
//    }
//}

// 给蓝牙模块发数据
void BLUE_TOOTH_SendString(const uint8_t *str)
{
    uint8_t i = 0;
    while (*(str + i) != '\n')
    {
			huart2.Instance->DR = *(str + i);
			// 等待发送完成（TXE标志：发送数据寄存器空）
			while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET){};
        i++;
    }
}

/**
 * @brief 与PC通信的发送中断
 * @param[in] void
 */
//void BLUE_TOOTH_SEND_DMAx_Streamx_IRQHandler(void)
//{
//    if (DMA_GetITStatus(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_IT_TCIFx))
//    {
//        DMA_ClearFlag(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_FLAG_TCIFx);
//        DMA_ClearITPendingBit(BLUE_TOOTH_SEND_DMAx_Streamx, BLUE_TOOTH_SEND_DMA_IT_TCIFx);
//        DMA_Cmd(BLUE_TOOTH_SEND_DMAx_Streamx, DISABLE);
//    }
//}

void BLUE_TOOTHSendData(BlueToothSendData *data)
{
    while (HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_BUSY){};
    
    memcpy(BlueToothSend_Buff, data, BlueTooth_SENDBUF_SIZE);
    BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 1] = 0x7f;
    BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 2] = 0x80;
    BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 3] = 0x00;
    BlueToothSend_Buff[BlueTooth_SENDBUF_SIZE - 4] = 0x00;

    HAL_UART_Transmit_DMA(&huart2, BlueToothSend_Buff, BlueTooth_SENDBUF_SIZE);
}
