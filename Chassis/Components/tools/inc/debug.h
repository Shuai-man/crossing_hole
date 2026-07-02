#ifndef _DEBUG_H
#define _DEBUG_H

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"
#include "bsp_dwt.h"

enum State
{
	OFF,
	TIME_OUT,
	ON,
};

/* 丢帧检测 */
typedef struct Loss_Debugger
{
  uint16_t recv_msgs_num;

  /*  接收帧率计算定义 */
  uint32_t last_cnt;
	uint32_t current_cnt;
	float cnt_dt;
	
	enum State state;

	uint16_t loss_num;
	uint32_t loss_time;
} Loss_Debugger;

/* 计时器 */
typedef struct Time_Debugger
{
    uint32_t last_cnt;
    float dt;
} Time_Debugger;

/* 数据接收 */

typedef struct GlobalDebugger
{
    Loss_Debugger gimbal_comm_debugger[2]; // 与云台通信    
    Loss_Debugger steers_comm_debugger[4]; // 舵
    Loss_Debugger wheels_comm_debugger[4]; // 轮
    Loss_Debugger super_power_debugger;    // 超级电容
    Loss_Debugger wireless_debugger; 	   // 无线充电
    Loss_Debugger referee_debugger;        // 裁判系统
} GlobalDebugger;

extern GlobalDebugger global_debugger;
extern float Ozone[8];

int8_t checkIMUOn(void);
void LossUpdate(Loss_Debugger *loss_debugger, float thresh_t);
void LossDetect(Loss_Debugger *loss_debugger);
#endif



