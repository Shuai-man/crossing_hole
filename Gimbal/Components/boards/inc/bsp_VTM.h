#ifndef __BSP_VTM_H
#define __BSP_VTM_H

#include "main.h"
#include "usart.h"
#include "debug.h"
#include <stdint.h>
#include <stdbool.h>

// ==================== VTM帧头 ====================
#define VTM_FRAME_HEADER1    0xA9
#define VTM_FRAME_HEADER2    0x53
#define VTM_FRAME_LEN        21    // 21字节
#define VTM_DATA_LEN         19    // CRC前19字节

// ==================== VTM帧DMA接收缓冲区 ====================
#define VTM_RX_BUF_NUM    VTM_FRAME_LEN  // 21字节
extern uint8_t vtm_rx_buf[2][VTM_RX_BUF_NUM];  //[0]/[1]双缓冲区

#pragma pack(1)
// ==================== VTM数据帧 ====================
typedef struct {
    uint16_t ch[4];//通道
    uint8_t gear;
    bool pause_btn,custom_btn_left,custom_btn_right,trigger_btn;
    uint16_t wheel;
    int16_t mouse_x,mouse_y,mouse_z;
    bool mouse_left,mouse_right,mouse_mid;
    uint16_t keyboard;
    uint16_t crc_recv;
    bool crc_ok,frame_ok;
		
	// 虚拟拨杆
	  bool sw[3]; 
		bool last_pause_btn,last_btn_left,last_btn_right;
} VTM_Remote;

enum Which_gear{
	gear_C=0,
	gear_N=1,
	gear_S=2,
};

enum VTM_CH{
	RIGHT_LR=0, // 右拨杆左右
	RIGHT_UD=1, // 右拨杆上下
	LEFT_UD=2,  // 左拨杆上下
	LEFT_LR=3,  // 左拨杆左右
};

enum Virtual_SW{
	Left_up=0,  // 左拨杆上
	Right_up=1, // 右拨杆上
	Pause=2,    // 暂停按钮
};
#pragma pack()

extern VTM_Remote vtm_remote;

void remote_VTM_init(void);  
void VTM_Frame_Parse(uint8_t *buf, VTM_Remote *data);
uint16_t VTM_CRC16_CCITT_False(uint8_t *data, uint16_t len);
uint32_t VTM_Bit_Extract(uint8_t *buf, uint16_t bit_offset, uint8_t bit_len);
int32_t VTM_Signed_Bit_Extract(uint8_t *buf, uint16_t bit_offset, uint8_t bit_len);
void VTM_RC_Receive(void);
void Virtual_SW(VTM_Remote *remote);
void VTM_Init(void);


#endif
