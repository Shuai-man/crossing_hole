#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "algorithmOfCRC.h"
#include "ins.h"

#include "ChassisGet.h"
// #include "robot_config.h"

typedef enum AUTOAIM_MODE
{
	NOT_USE_AIM = 0,
	STD_AUTO_AIM,
	BUFF,
	STATIC_AUTO_AIM,
	BIG_BUFF,
} AUTOAIM_MODE;

/*   发送数据定义 */
#pragma pack(push, 1) // 不进行字节对齐
typedef struct PCSendData
{
	// 数据顺序不能变,注意24字节对齐
	//	int8_t start_flag; // 一样
	//	float pitch_now;
	//	float yaw_now;
	//	float roll_now; // 0
	//	float actual_bullet_speed;
	//	uint8_t shoot_avaiable; //可发弹量
	//	uint8_t aim_request; // 右键
	//	uint8_t mode_want; // 模式 辅瞄模式：0 大符模式：1 小符模式：2
	//	uint8_t number_want; // 不用，但是不能删，否则解码出问题
	//	uint8_t enemy_color; //自己：蓝 1 红 0
	//	uint16_t crc16;
	uint8_t frame_header1; // 0x55
	uint8_t frame_header2; // 0xAA
	float pitch_now;
	float yaw_now;
	float pitch_omega;				 // pitch角速度
	float yaw_omega;					 // yaw角速度
	float pitch_tff;					 // pitch力矩
	float yaw_tff;						 // yaw力矩
	float actual_bullet_speed; // 弹速
	uint8_t shoot_avaiable;		 // 可发弹量
	uint8_t aim_request;			 // 右键
	uint8_t mode_want;				 // 模式 辅瞄模式：0 大符模式：1 小符模式：2
	uint8_t number_want;			 // 不用，但是不能删，否则解码出问题
	uint8_t enemy_color;			 // 自己：蓝 1 红 0
	uint16_t crc16;
	uint8_t frame_tail; // 0x0D
} PCSendData;

typedef struct PCRecvData
{

	// int8_t start_flag;
	// uint8_t detect_number; // 是否识别到;
	// uint8_t shoot_flag;		 // 上位机决定打弹与否
	// float pitch;					 // 在上位机叫pitch_setpoint;
	// float yaw;						 // 在上位机叫yaw_setpoint;
	// uint16_t crc16;
	uint8_t frame_header1; // 0xAA
	uint8_t frame_header2; // 0x55
	uint8_t mode_select;   //控制模式选择 0x11为位置控制模式，0x22 为前馈力矩+位置+速度模式
	uint8_t detect_number; // 是否识别到;
	uint8_t shoot_flag;		 // 上位机决定打弹与否
	float pitch_setpoint;					 // 目标pitch
	float yaw_setpoint;						 // 目标yaw
	float pitch_omega_setpoint;					 // 目标pitch角速度
	float yaw_omega_setpoint;						 // 目标yaw角速度
	float pitch_tff_setpoint;					 // 目标pitch力矩
	float yaw_tff_setpoint;						 // 目标yaw力矩
	uint16_t crc16;
	uint8_t frame_tail; // 0x0D

} PCRecvData;
#pragma pack(pop) // 不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];
#endif // !_PC_SERIAL_H
