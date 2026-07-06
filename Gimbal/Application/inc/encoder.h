#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

/*
[设备ID] ＋ [长度LEN] ＋ [编码器地址] ＋ [指令FUNC] ＋ [数据DATA]
下发：[0x01]|[0x04][0x01][0x01][0x00]
返回：[0x01]|[0x07][0x01][0x01][0x45][0x23][0x01][0x00]
低字节在前,编码器值：0X00012345（十进制：74565）
*/

//已经设置了编码器自动回传，所以只写接收和解码部分
//下发和接收用的CANID是一样的
//ENCODER_ID 0x01 默认ID
#pragma pack(1)
typedef struct {
    uint8_t len;        //长度，0x07
    uint8_t addr;       //地址，0x01
    uint8_t func;       //功能码，0x01
    uint8_t data1;      //数据1
    uint8_t data2;      //数据2
    uint8_t data3;      //数据3
    uint8_t data4;      //数据4
} Encoder_Frame_t;

typedef struct {
  uint8_t DeviceID; //设备ID，0x01
  Encoder_Frame_t frame;
  uint32_t value;
} Encoder_t;

#define DOWN_ENCODER 0.0f
#define UP_ENCODER 0.0f

extern uint8_t encoder_receive_data[7]; // 接收数据缓冲区
extern Encoder_t encoder; // 编码器数据结构
#endif
