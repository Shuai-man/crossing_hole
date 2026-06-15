#include "bsp_VTM.h"
#include <string.h>

// ==================== 全局变量定义 ====================
VTM_Remote vtm_remote = {0};
uint8_t vtm_rx_buf[2][VTM_RX_BUF_NUM];  // 双缓冲接收数组

// 外部声明UART6和DMA句柄（CubeMX自动生成）
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/**
 * @brief  位提取函数：从指定缓冲区、指定偏移位提取指定位数的数据
 * @note   数据格式：bit_offset为LSB起始偏移，STM32 UART 低位在前(LSB first)
 * @param  buf: 原始数据缓冲区指针
 * @param  bit_offset: 从buf起始的bit偏移量（从0开始，对应buf[0].bit0）
 * @param  bit_len: 提取数据的位长度（1~32位）
 * @retval 提取到的无符号32位数据（bit0为最低位，bit[bit_len-1]为最高位）
 */
uint32_t VTM_Bit_Extract(uint8_t *buf, uint16_t bit_offset, uint8_t bit_len)
{
    if (bit_len == 0 || bit_len > 32) return 0;

    uint32_t result = 0;
    for (uint8_t i = 0; i < bit_len; i++)
    {
        // 计算当前bit在缓冲区中的绝对位置
        uint32_t abs_bit = bit_offset + i;
        uint16_t byte_idx = abs_bit / 8;
        uint8_t bit_in_byte = abs_bit % 8;

        // 若该bit为1，将result对应位设为1
        if (buf[byte_idx] & (1U << bit_in_byte))
        {
            result |= (1U << i);
        }
    }

    return result;
}

/**
 * @brief  有符号位提取函数（自动符号扩展，适配16位有符号数据解析）
 */
int32_t VTM_Signed_Bit_Extract(uint8_t *buf, uint16_t bit_offset, uint8_t bit_len)
{
    uint32_t unsigned_val = VTM_Bit_Extract(buf, bit_offset, bit_len);
    // 符号扩展：最高位为1时，高位补1
    if (bit_len < 32 && (unsigned_val & (1U << (bit_len - 1))))
    {
        unsigned_val |= (0xFFFFFFFFU << bit_len);
    }
    return (int32_t)unsigned_val;
}

/**
 * @brief  计算CRC16_CCITT_False校验值
 * @note   该函数实现CRC16_CCITT_False校验算法，用于校验数据的完整性。
 * @param  data: 输入数据指针
 * @param  len: 输入数据长度
 * @retval CRC16_CCITT_False
 */
uint16_t VTM_CRC16_CCITT_False(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    
    const uint16_t poly_rev = 0x8408;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            uint16_t bit = (b >> j) & 0x01;
            if ((crc & 0x0001) ^ bit) {
                crc = (crc >> 1) ^ poly_rev;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// ==================== 解析VTM数据帧 ====================
void VTM_Frame_Parse(uint8_t *buf, VTM_Remote *data)
{
    // 1. 校验帧头
    uint8_t header1 = (uint8_t)VTM_Bit_Extract(buf, 0, 8);
    uint8_t header2 = (uint8_t)VTM_Bit_Extract(buf, 8, 8);
    data->frame_ok = (header1 == VTM_FRAME_HEADER1) && (header2 == VTM_FRAME_HEADER2);
    if (!data->frame_ok) {
        memset(data, 0, sizeof(VTM_Remote));
        return;
    }

    // 2. 校验CRC16_CCITT_False
    uint16_t crc_calc = VTM_CRC16_CCITT_False(buf, 19);
    data->crc_recv = (uint16_t)VTM_Bit_Extract(buf, 152, 16);
    data->crc_ok = (crc_calc == data->crc_recv);
    if (!data->crc_ok) {
        return;
    }

    // 3. 解析有效数据
    // 4个通道数据
    data->ch[RIGHT_LR] = (uint16_t)VTM_Bit_Extract(buf, 16, 11);
    data->ch[RIGHT_UD] = (uint16_t)VTM_Bit_Extract(buf, 27, 11);
    data->ch[LEFT_UD] = (uint16_t)VTM_Bit_Extract(buf, 38, 11);
    data->ch[LEFT_LR] = (uint16_t)VTM_Bit_Extract(buf, 49, 11);

    // 档位数据
    data->gear = (uint8_t)VTM_Bit_Extract(buf, 60, 2);

    // 按钮数据
    data->pause_btn = (VTM_Bit_Extract(buf, 62, 1) == 1);
    data->custom_btn_left = (VTM_Bit_Extract(buf, 63, 1) == 1);
    data->custom_btn_right = (VTM_Bit_Extract(buf, 64, 1) == 1);

    // 拨轮数据
    data->wheel = (uint16_t)VTM_Bit_Extract(buf, 65, 11);

    // 后侧按钮数据
    data->trigger_btn = (VTM_Bit_Extract(buf, 76, 1) == 1);

    // 鼠标数据
    data->mouse_x = (int16_t)VTM_Signed_Bit_Extract(buf, 80, 16);
    data->mouse_y = (int16_t)VTM_Signed_Bit_Extract(buf, 96, 16);
    data->mouse_z = (int16_t)VTM_Signed_Bit_Extract(buf, 112, 16);

    // 鼠标按钮数据
    data->mouse_left = (VTM_Bit_Extract(buf, 128, 2) == 1);
    data->mouse_right = (VTM_Bit_Extract(buf, 130, 2) == 1);
    data->mouse_mid = (VTM_Bit_Extract(buf, 132, 2) == 1);

    // 键盘数据
    data->keyboard = (uint16_t)VTM_Bit_Extract(buf, 136, 16);

    // 调试信息
		LossUpdate(&global_debugger.VTM_debugger,0.0025f);
}

// ==================== 初始化UART6 DMA接收和空闲中断 ====================
void remote_VTM_init(void)
{
    // 1. 使能UART6 DMA接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    // 使能UART6空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // 2. 先禁用DMA，确保寄存器可安全配置
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN);

    // 3. 配置DMA参数
    hdma_usart6_rx.Instance->PAR = (uint32_t)&(USART6->DR);  // 外设地址：UART6数据寄存器
    hdma_usart6_rx.Instance->M0AR = (uint32_t)vtm_rx_buf[0]; // 内存地址0
    hdma_usart6_rx.Instance->M1AR = (uint32_t)vtm_rx_buf[1]; // 内存地址1
    hdma_usart6_rx.Instance->NDTR = VTM_RX_BUF_NUM;          // 传输数据长度
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);     // 开启双缓冲模式
    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}


void VTM_RC_Receive(void)
{
		uint8_t *pBuf = NULL;
    // 1. 检测UART6空闲中断（一帧数据接收完成）
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))  
    {
        // 清除空闲中断标志（读取SR和DR寄存器）
        (void)USART6->SR;
        (void)USART6->DR;

        // 2. 判断当前使用的缓冲数组
        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
			pBuf = vtm_rx_buf[0];
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
        }
        else
        {
			pBuf = vtm_rx_buf[1];
            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);
        }

        // 3. 帧头匹配并解析VTM数据
        for(uint8_t i=0; i<VTM_RX_BUF_NUM-1; i++)
        {
            if(pBuf[i] == VTM_FRAME_HEADER1 && pBuf[i+1] == VTM_FRAME_HEADER2)
            {
                // 解析VTM数据帧
                VTM_Frame_Parse(&pBuf[i], &vtm_remote);
                break;
            }
        }
    }

    // 清除校验位中断标志
    __HAL_UART_CLEAR_PEFLAG(&huart6);
}


// 虚拟拨杆 宏定义
#define LONG_PRESS_RESET_TIME 500

/**
 * @brief  虚拟开关函数：将遥控器按键转换为虚拟拨杆状态
 * @param  remote: 遥控器数据结构体指针
 */
void Virtual_SW(VTM_Remote *remote)
{
    static uint16_t left_timer = 0;
    static uint16_t right_timer = 0;
    static uint16_t pause_timer = 0;

    // ===================== 左侧自定义按键 → 虚拟上拨杆 =====================
    // 按键按下上升沿：置1
    if(remote->custom_btn_left == 1 && remote->last_btn_left == 0)
    {
        remote->sw[Left_up] = 1;
    }
    // 长按复位：按下超过设定时间，自动清零
    if(remote->custom_btn_left == 1)
    {
        left_timer++;
        if(left_timer >= LONG_PRESS_RESET_TIME)
        {
            remote->sw[Left_up] = 0;
            left_timer = 0;
        }
    }
    else
    {
        left_timer = 0;  // 松开按键，清零计时器
    }
    remote->last_btn_left = remote->custom_btn_left;

    // ===================== 右侧自定义按键 → 虚拟上拨杆 =====================
    // 按键按下上升沿：置1
    if(remote->custom_btn_right == 1 && remote->last_btn_right == 0)
    {
        remote->sw[Right_up] = 1;
    }
    // 长按复位
    if(remote->custom_btn_right == 1)
    {
        right_timer++;
        if(right_timer >= LONG_PRESS_RESET_TIME)
        {
            remote->sw[Right_up] = 0;
            right_timer = 0;
        }
    }
    else
    {
        right_timer = 0;
    }
    remote->last_btn_right = remote->custom_btn_right;

    // ===================== 暂停按键 → 虚拟暂停拨杆 =====================
    // 按键按下上升沿：置1
    if(remote->pause_btn == 1 && remote->last_pause_btn == 0)
    {
        remote->sw[Pause] = 1;
    }
    // 长按复位
    if(remote->pause_btn == 1)
    {
        pause_timer++;
        if(pause_timer >= LONG_PRESS_RESET_TIME)
        {
            remote->sw[Pause] = 0;
            pause_timer = 0;
        }
    }
    else
    {
        pause_timer = 0;
    }
    remote->last_pause_btn = remote->pause_btn;
}
