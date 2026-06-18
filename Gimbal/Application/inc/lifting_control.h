#ifndef _LIFTING_CONTROL_H
#define _LIFTING_CONTROL_H

#include "remote_control.h"
#include "tof.h"
#include "pid.h"
#include "M2006.h"
#include <stdint.h>
#include "debug.h"

#define GIMBAL_HIGH_STATE 177 //此处修改最高处的设定值（单位:mm）
#define GIMBAL_LOW_STATE 30
#define LIFTING_SPEED_MAX 2700.0f
#define LIFT_DIR -1 //向上为正

#define POSE_THRESHOLD 5.0f //位置误差阈值
#define CURRENT_THRESHOLD 0.17f*C610_MAX_CURRENT //电流误差阈值
#define FINISH_CNT 600 //结束计数阈值

#define OBSTACLE_ERR_THRESHOLD   20.0f   // mm, 误差大于此才启用阻塞检测
#define OBSTACLE_DELTA_THRESHOLD 8.0f    // mm, 距离下降超过此值才触发阻塞

typedef struct LiftingController
{
	uint8_t high_state;
	uint8_t low_state;
  uint8_t lift_state;//丝杆状态
	
	uint8_t finished;//结束
	float finish_cnt;

  uint16_t recovery_ref; // 阻挡前记录的last_distance，用作恢复判据
  uint16_t last_distance;//上一次TOF距离
  bool is_up;//是否上升
  bool error;//是否有错误

  PID_t lift_pos_pid;
  PID_t lift_speed_pid; //丝杆速度环
  M2006_Recv lift_recv; //原始数据
  M2006_Info lift_info; //解码数据: 度/秒

  float set_pos;
  float set_speed;

  float send_current;     // 发送电流值

} LiftingController;

extern LiftingController lifting_controller;
void LiftPidInit(void);
void Lifting_Control(void);
void Lifting_State_Check(void);
void Lifting_Error_Check(float target_height);
void Lifting_Finish_Check(void);
#endif
