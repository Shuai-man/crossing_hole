#include "encoder.h"
#include "can.h"
#include "bsp_can.h"

Encoder_t encoder; // 编码器数据结构
volatile uint32_t Encoder_UpdateCounter = 0U; // 外置编码器有效接收帧累计数

void SetEncoderZero(void)//设置当前位置为0编码值
{
  int8_t data[8]={0x04,ENCODER_ID,0x06,0x00};
  CanSend(ENCODER_CAN,data,ENCODER_ID);
}
