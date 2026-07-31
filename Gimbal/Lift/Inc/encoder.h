#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

/*
[设备ID] ＋ [长度LEN] ＋ [编码器地址] ＋ [指令FUNC] ＋ [数据DATA]
下发：[0x01]|[0x04][0x01][0x01][0x00]
返回：[0x01]|[0x07][0x01][0x01][0x45][0x23][0x01][0x00]
低字节在前,编码器值：0X00012345（十进制：74565）
*/

// 已经设置了编码器自动回传，所以只写接收和解码部分
// 下发和接收用的CANID是一样的
// ENCODER_ID 0x01 默认ID
#pragma pack(1)
typedef struct
{
  uint8_t len;   // 长度，0x07
  uint8_t addr;  // 地址，0x01
  uint8_t func;  // 功能码，0x01
  uint8_t data1; // 数据1
  uint8_t data2; // 数据2
  uint8_t data3; // 数据3
  uint8_t data4; // 数据4
} Encoder_Frame_t;
#pragma pack()

typedef struct
{
  Encoder_Frame_t frame;
  volatile uint32_t value; // CAN中断更新、控制任务读取的绝对位置计数
  uint8_t set_zero;        // 设置零点
} Encoder_t;
extern Encoder_t encoder; // 编码器数据结构

/*
 * 每收到一帧有效的编码器CAN数据就加1。
 * 升降保护通过它区分“编码器真的更新了”与“1kHz控制循环重复读取了旧值”。
 */
extern volatile uint32_t Encoder_UpdateCounter;
void SetEncoderZero(void);
#endif
