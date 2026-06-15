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
    Loss_Debugger imu_debugger;//IMU检测
		Loss_Debugger DT7_debugger;//DT7遥控器
		Loss_Debugger VTM_debugger;//VTM遥控器
    Loss_Debugger pitch_debugger;//pitch电机
		Loss_Debugger yaw_debugger;//yaw电机
    Loss_Debugger friction_debugger[2];//摩擦轮电机
		Loss_Debugger toggle_debugger;//拨弹电机
		Loss_Debugger lift_debugger;//抬升电机
		Loss_Debugger tof_debugger;//tof测距模块
    Loss_Debugger receive_chassis_debugger[2];//板间通信
    Loss_Debugger pc_receive_debugger;//PC通信
} GlobalDebugger;

extern GlobalDebugger global_debugger;
extern float Ozone[8];

int8_t checkIMUOn(void);
void LossUpdate(Loss_Debugger *loss_debugger, float thresh_t);
void LossDetect(Loss_Debugger *loss_debugger);
#endif
