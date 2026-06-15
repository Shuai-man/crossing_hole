#ifndef _TOGGLE_BULLET_H
#define _TOGGLE_BULLET_H

#include "main.h"

#include <stdbool.h>

#include "pid.h"
#include "M2006.h"
#include "ChassisGet.h"

#include "bsp_dwt.h"

#define ONE_GRID_ANGLE     45.0f   // 度
#define SIGN_ROTATE         1.0f   // 拨盘正转方向
#define DETECTION_PERIOD    0.05f  // 50ms

#define FIRE_TIME_DIFF      0.3f   // 从控制到实际打弹的时间差，单位：s

/* 弹量阈值 */
#define BULLET_LOW_THRESHOLD     10 // 最低弹量阈值
#define BULLET_DEFAULT_REMAINING 20 // 默认剩余发弹量
#define BULLET_MAX_VALID 40 // 最大有效弹量

/* 射频常量 */
#define FREQ_LOW     5.0f //hz
#define FREQ_NORMAL 20.0f //hz
#define FREQ_TEST 50.0f //hz

enum TOGGLE_CONTROL_MODE
{
  TOGGLE_SPEED, // 拨弹速度控制(连发)
  TOGGLE_POS,   // 拨弹单发控制(单发)
  TOGGLE_STOP
};

/* 拨弹控制器 */
typedef struct ToggleController
{
  PID_t toggle_pos_pid;
  PID_t toggle_speed_pid;   // 拨弹速度环
  M2006_Recv toggle_recv;   // 原始数据
  M2006_Info toggle_info;   // 解码数据: 度/秒

  float set_pos;
  float set_speed;
  int8_t is_shoot;          // 发射指令
  bool is_aim;              // 是否瞄准

  float freq;               // 弹频 hz
  float freq_time;          // 转换为s的弹频时间
  float send_current;       // 发送电流值

  uint8_t remaining_bullets;   // 剩余发弹量
  int8_t predict_bullets;      // 预测剩余发弹量  
  uint8_t receive_bullets;     // 接收发弹量
  uint8_t last_receive_bullets;// 上一次接收发弹量

  uint32_t current_cnt;        // 拨盘计数器
  float dt_current;            // 当前时间间隔
  float dt_accumulated;        // 累计时间间隔
} ToggleController;

extern ToggleController toggle_controller;

void Toggle_Init(void);
void Toggle_Calculate(enum TOGGLE_CONTROL_MODE control_mode, float set_point);
void Toggle_AddGrid(ToggleController *controller, uint8_t num);
void Toggle_SelectShootFreq(void);
void Toggle_Fire(uint8_t receive_bullets);
#endif // !_TOGGLE_BULLET_H
