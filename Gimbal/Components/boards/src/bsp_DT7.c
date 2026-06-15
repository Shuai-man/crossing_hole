#include "bsp_DT7.h"

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
DT7_Remote dt7_remote;

/**
 * @brief          DT7遥控器初始化（UART3+DMA双缓冲接收）
 * @param[in]      none
 * @retval         none
 */
void remote_DT7_init(void)
{
    // 使能UART3 DMA接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // 使能UART3空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 禁用DMA，确保寄存器可安全配置
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    // 配置DMA外设地址：UART3数据寄存器
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    // 配置DMA双缓冲内存地址
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(sbus_rx_buf[0]);
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(sbus_rx_buf[1]);
    // 配置DMA接收数据长度
    hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
    // 开启DMA双缓冲模式
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

/**
 * @brief          遥控器通道死区限制（消除零点漂移）
 * @param[in]      ch: 通道值指针
 * @param[in]      limitValue: 死区阈值
 * @retval         none
 */
void RemoteLimit(unsigned short *ch, unsigned short limitValue)
{
    if ((*ch - CH_MIDDLE < limitValue) && (*ch - CH_MIDDLE > -limitValue))
        *ch = CH_MIDDLE;
}

/**
 * @brief          DT7 SBUS协议数据解码
 * @param[in]      rx_buffer: 接收数据缓冲区
 * @param[out]     remote: 遥控器数据结构体
 * @retval         none
 */
void DT7_Decode(unsigned char rx_buffer[] , DT7_Remote *remote)
{
    // 4个遥控通道解码
    remote->ch[RIGHT_CH_LR] = (rx_buffer[0] | (rx_buffer[1] << 8)) & 0x07ff;
    remote->ch[RIGHT_CH_UD] = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff;
    remote->ch[LEFT_CH_LR] = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;
    remote->ch[LEFT_CH_UD] = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff;

    // 2个拨杆开关解码
    remote->s[RIGHT_SW] = ((rx_buffer[5] >> 4) & 0x0003);
    remote->s[LEFT_SW] = ((rx_buffer[5] >> 6) & 0x0003);

    // 鼠标轴数据解码
    remote->x = rx_buffer[6] | (rx_buffer[7] << 8);
    remote->y = rx_buffer[8] | (rx_buffer[9] << 8);
    remote->z = rx_buffer[10] | (rx_buffer[11] << 8);

    // 鼠标左右按键
    remote->press_l = rx_buffer[12];
    remote->press_r = rx_buffer[13];

    // 键盘按键值
    memcpy(&remote->keyValue, (void *)&rx_buffer[14], 2);

    // 按键值
    remote->poke = (rx_buffer[17] << 8) | rx_buffer[16];

    // 4个通道加入死区限制
    for (int i = 0; i < 4; i++)
    {
        RemoteLimit(&remote->ch[i], 30);
    }
		
    // 调试信息：计算接收间隔、统计接收帧数
		LossUpdate(&global_debugger.DT7_debugger,0.02f);
}

/**
 * @brief          DT7遥控器数据接收处理（空闲中断+DMA双缓冲）
 * @param[in]      none
 * @retval         none
 */
void DT7_RC_Receive(void)
{
    // 接收到单个字节（清除校验错误标志）
	if (huart3.Instance->SR & UART_FLAG_RXNE)
	{
			__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
  // 一帧数据接收完成（空闲中断触发）
	else if (USART3->SR & UART_FLAG_IDLE)
	{
			static uint16_t this_time_rx_len = 0;

			__HAL_UART_CLEAR_PEFLAG(&huart3);

      // 当前使用缓冲区0
			if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
			{
					/* 切换使用内存缓冲区1 */
					// 禁用DMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					// 计算实际接收数据长度
					this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

					// 重置DMA数据长度
					hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

					// 切换至缓冲区1
					hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

					// 重新使能DMA
					__HAL_DMA_ENABLE(&hdma_usart3_rx);

          // 数据长度正确，执行解码
					if (this_time_rx_len == RC_FRAME_LENGTH)
					{
							DT7_Decode(sbus_rx_buf[0],&dt7_remote);
					}
			}
			else
			{
					/* 当前使用内存缓冲区1 */
					// 禁用DMA
					__HAL_DMA_DISABLE(&hdma_usart3_rx);

					// 计算实际接收数据长度
					this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

					// 重置DMA数据长度
					hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

					// 切换至缓冲区0
					DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

					// 重新使能DMA
					__HAL_DMA_ENABLE(&hdma_usart3_rx);

          // 数据长度正确，执行解码
					if (this_time_rx_len == RC_FRAME_LENGTH)
					{
							DT7_Decode(sbus_rx_buf[1],&dt7_remote);
					}
			}
	}	
}
