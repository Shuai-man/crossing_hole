#ifndef _CHASIS_CONTROLLER_H
#define _CHASIS_CONTROLLER_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_atan2_f32.h"

#include "tools.h"


#include "user_lib.h"
#include "remote_control.h"
#include "pid.h"

//#include "wheel_ins.h"


#include "can_config.h"
#include "TD.h"

//#include "SuperPower.h"
#include "PowerLimit.h"

#include "M3508.h"
#include "GM6020.h"
#include "bsp_can.h"

#include "robot_config.h"
#include "mecanum.h"
#include "omni.h"
#include "steer.h"

#include "PowerControlTask.h"
#include "NingCap.h"
#include "RefereeTask.h"
#include "Offline_Task.h"


extern float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;

#define RAD_TO_ANGLE_COEF 57.295779513f
#define ANGLE_TO_RAD_COEF 0.0174532925f

#if ROBOT == NIUNIU
#define UI_FRONT_BIAS 8160
#elif ROBOT == CHEN_JING_YUAN
#define UI_FRONT_BIAS 0
#elif ROBOT == NIU_MO_SON
#define UI_FRONT_BIAS 0
#elif ROBOT == QI_TIAN_DA_SHENG
#define UI_FRONT_BIAS 0
#endif

typedef struct Vector // 向量
{
  float module; // 模值
  float angle;  // 角度
} Vector;

typedef struct Sensors
{
  GM6020_Recv steer_recv[4];
  GM6020_Info steer_decode[4]; // 解码过后的舵信息,速度未经过滤波
  M3508_Recv wheels_recv[4];
  M3508_Info wheels_decode[4];     // 速度已经经过滤波
  M3508_Info wheels_decode_raw[4]; // 速度未经过滤波
} Sensors;

typedef struct ExcuteTorque
{
  uint8_t steers_send_data[8];
  uint8_t wheels_send_data[8];

  float steers_set_current[4];
  float wheels_set_current[4];
} ExcuteTorque;

// @TODO :增加四向跟随模式

typedef enum
{
  TWO_SIDES_FOLLOW,     // 双向跟随
  FOUR_SIDES_FOLLOW,    // 四向跟随
  FOUR_SIDES_FOLLOW_45, // 45度四向跟随
  TWO_SIDES_LEFT_RIGHT, // 双向左右跟随
} CHASSIS_FOLLOW_TYPE;

/* 底盘跟随方向  */
typedef enum
{
  CHASSIS_FRONT,
  CHASSIS_BACK,
  CHASSIS_LEFT,
  CHASSIS_RIGHT,
} chassis_direction_e;

typedef enum Energy_State{
	ENERGY_FULL,
	ENERGY_50,
	ENERGY_30,
	ENERGY_15,
	ENERGY_5,
	WEAK,
} Energy_State;

typedef struct Infantry
{
  /*  传感器信息 */
  Sensors sensors_info;

  /*  执行机构 */
  ExcuteTorque excute_info;

  /* 移动控制  */
  float target_yaw_v; // rad/s  仅底盘运动模式下才生效，其余模式均由底盘板自动决策
  float target_x_v;   // target_x_v_percent * speed_x_max
  float target_y_v;

  float target_x_v_percent; // 0.0 -1.0f
  float target_y_v_percent;
  float target_yaw_v_percent;

  float speed_x_max;
  float speed_y_max;
  float speed_yaw_max;

  // 实际控制量
  float set_yaw_v; // rad/s
  float set_x_v;
  float set_y_v;

  // 跟踪微分器
  TD_t x_v_td;
  TD_t y_v_td;
  TD_t yaw_v_td;

  // 实际速度
  float x_v;
  float y_v;
  float yaw_v;

  // 轮电机设定速度
  float wheels_set_v[4];

  uint32_t last_cnt;
  float delta_t; // 前后两帧时间差

  CHASSIS_TYPE chassis_type;
  chassis_direction_e chassis_direction;
  CHASSIS_FOLLOW_TYPE chassis_follow_type;
  YAW_MOTOR_TYPE yaw_motor_type;

  // 跟随参数
  float yaw_radian; //yaw电机角度，弧度制，0-2pi
  float yaw_angle; //yaw电机角度，角度制，0-360
  float error_angle;

  // 异常检测
  int16_t abnormal_count; // 异常次数计数
  int8_t is_abnormal;     // 是否正在异常状态

  // 转向PID
  PID_t turn_pos_pid;
	PID_t turn_speed_pid;
  float target_pid_yaw_v;

  // 麦轮底盘跟随前馈
  Feedforward_t Mecanum_Follow_FF;
  float Mecanum_Follow_FF_Coefficient[3];

  // 轮控PID
  PID_t wheels_pid[4];
  float wheels_set_current;

  // 舵控PID
  PID_t steers_speed_pid[4];
  PID_t steers_angle_pid[4];
  float steers_set_current;

  // 舵轮组件
  int8_t steer_wheel_install_direction[4]; // 舵轮轮电机安装方向修正
  uint16_t steer_init_encoder[4];          // 初始编码器角度
  Vector steer_vector[4];                  // 舵向量
  Vector robot_vector;                     // 底盘运动向量
  // 舵轮6020前馈
  Feedforward_t Steer_6020_FF;
  float Steer_6020_FF_Coefficient[3];

  float cos_dir; // 云台与底盘的夹角
  float sin_dir;

  // 功率控制
  PowerLimiter power_limiter;
  float set_power; // 机器人功率设置
  PowerLimitMethod power_limit_method;
  Energy_State energy_state;

  // 测试数据
  float wheels_send_current[4];
	
	// 复活后电机重新上电等待时间
	uint32_t Timer;
	float Motor_Init_Time;
	int All_Motor_On;
	
	uint16_t friction_speed;
	uint8_t aim_id;

} Infantry;

extern Infantry infantry;

void InfantryInit(Infantry *infantry);
void main_control(Infantry *infantry);
void wheels_power_limit(Infantry *infantry);
void execute_control(ExcuteTorque *torque);
void get_sensors_info(Sensors *sensors_info);

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry);

// 加速策略
void wheels_accel(Infantry *infantry);

#endif // !_CHASIS_CONTROLLER_H
