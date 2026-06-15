#ifndef BSP_RC_H
#define BSP_RC_H

// #include "struct_typedef.h"//
#include "main.h"
#include "usart.h"
#include "debug.h"
#include <string.h>

// #include "remote_control.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

typedef struct
{
    unsigned short ch[4]; // 通道
    unsigned short s[2];  // 拨杆
    unsigned short poke;  // 拨轮

    unsigned short Previous_rc_Left_SW;  // 上一次接收的左拨杆状态，用于检测状态变化
    unsigned short Previous_rc_Right_SW; // 上一次接收的右拨杆状态，用于检测状态变化

    short x;
    short y;
    short z;
    unsigned char press_l;
    unsigned char press_r;

    uint16_t keyValue;
} DT7_Remote;

#define CH_MIDDLE 1024
#define CH_RANGE 660 // 通道范围，CH_MIDDLE - CH_RANGE ~ CH_MIDDLE + CH_RANGE

// 拨杆位置
enum SWPos
{
    Lost = 0,
    Up = 1,
    Mid = 3,
    Down = 2
};
enum WhichSW
{
    LEFT_SW = 1,
    RIGHT_SW = 0
};
enum WhichCH
{
    RIGHT_CH_LR = 0, // 右拨杆左右
    RIGHT_CH_UD = 1, // 右拨杆上下
    LEFT_CH_LR = 2,  // 左拨杆左右
    LEFT_CH_UD = 3,  // 左拨杆上下
};

void remote_DT7_init(void);
void DT7_Decode(unsigned char rx_buffer[], DT7_Remote *remote);
void DT7_RC_Receive(void);
void RemoteLimit(unsigned short *ch, unsigned short limitValue);

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern DT7_Remote dt7_remote;
#endif
