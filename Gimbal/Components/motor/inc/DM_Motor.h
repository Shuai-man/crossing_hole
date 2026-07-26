#ifndef _DM_MOTOR_H_
#define _DM_MOTOR_H_

#include "can.h"
#include "tools.h"
#include <string.h>

//2的12次方，发送的扭矩会映射到这个范围内：-T_MAX~T_MAX
//发送值为0的时候对应-T_MAX，请注意清空扭矩不要发0，而是2048
#define DM_T_Data_MAX 4096  //12位  
#define DM_V_Data_MAX 4096  //12位
#define DM_P_Data_MAX 65536 //16位
//速度、扭矩用12位数据，位置用16位数据，可见电机手册

#define P_MIN -3.141593f
#define P_MAX 3.141593f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum DM_MODE
{
	DM_ENABLE,
	DM_DISABLE,
	DM_MIT_CONTROL
} DM_MODE;

#pragma pack(push, 1)
typedef struct DM_MIT
{
	float P_des;
	float V_des;
	uint16_t Kp;
	uint16_t Kd;
	float t_ff;

	uint16_t Send_P_des;
	uint16_t Send_V_des;
	uint16_t Send_Kp;
	uint16_t Send_Kd;
	uint16_t Send_t_ff;

	float T_Max;
	float P_Max;
	float V_Max;

	char Motor_Enable;

	uint16_t P_Receive;//电机机械角度
	float V_Receive;
	float t_ff_Receive;

	float P_angle;//角度制
	float P_radian;//弧度制

	uint8_t ERR_State;

	// 温度
	uint8_t T_MOS;
	uint8_t T_Rotor;
} DM_MIT;

#pragma pack(pop)

void DM_Motor_Control(DM_MIT *Motor, int8_t *data, DM_MODE mode);
void DM_Motor_Receive(uint8_t *Data_Receive, DM_MIT *Motor);
void DM_Motor_Init(DM_MIT *Motor, float P_Max, float T_Max, float V_Max);

#endif
