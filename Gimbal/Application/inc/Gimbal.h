#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "main.h"
#include "pid.h"
#include "GM6020.h"
#include "DM_Motor.h"
#include "ins.h"
#include "remote_control.h"
#include "my_filter.h"
#include "TD.h"

#include "Robot_config.h"
#include "RLS_Identification.h"
#include "SystemIdentification.h"

// pitch
#define GIMBAL_PITCH_GYRO_SIGN -1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f       // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN -1.0f // 云台PITCH电机方向，向上为正

// 云台角度限位
#define GIMBAL_ANGLE_MAX 20.0f
#define GIMBAL_ANGLE_MIN -10.0f

#define GIMBAL_PITCH_COMP 4000.0f
#define GIMBAL_PITCH_COMP_COEF 1.0f

// Pitch角度机械零点
#define GIMBAL_PITCH_ZERO 215.172729f

// 云台底盘的yaw轴零点都需要更改
#define GIMBAL_ANGLE_ZERO 9.51965332f

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_YAW_MOTOR_SIGN -1.0f // 用来标记电机的方向，逆时针为正
#define GIMBAL_YAW_GYRO_SIGN 1.0f   // 用来标记gyro的方向，逆时针为正
// #define GIMBAL_YAW_POS_FORWARD_COEF 0.3f // 角度环前馈系数
// #define GIMBAL_YAW_SPEED_FORWARD_COEF 0.f
#define GIMBAL_YAW_J 8.92400551  // 转动惯量
#define GIMBAL_YAW_B 19.584095 // 阻尼系数，与速度有关
#define GIMBAL_YAW_C 26.2818222f   // 库伦摩擦系数，与结构有关

// 系统辨识开关（0=关闭, 1=开启）
#define GIMBAL_SYSID 0

// 摩擦力模型调参
#define BORDER_FRICTION_SPEED 6.0f    // 临界计算摩擦力速度，大于此速度将是全摩擦力补偿
#define FRICTION_CURRENT_COMP 1500.0f // 辨识所得到的摩擦力电流发送值
#define FRICTION_FORWARD_COEF 0.0f    // 前馈补偿系数

/* 底盘跟随方向  */
typedef enum
{
  GIMBAL_FRONT = 0,
  GIMBAL_BACK = 1,
} gimbal_direction_e;

typedef struct GimbalController
{
  DM_MIT DM_Yaw_Motor;
  DM_MIT DM_Pitch_Motor;

  // 转向控制
  // 最小回正角度
  float err_angle;     // 初始角度误差
  float err_angle_180; // 误差余角，用于判断方向

  uint8_t return_flag; // 回正标志位，0是完成，1是开始回正，2是正在回正

  // Pitch 轴
  PID_t pitch_speed_pid;             // 速度环
  PID_t pitch_angle_pid;             // 角度环
  TD_t pos_pitch_td;                 // 位置跟踪微分器

  float comp_pitch_current; // 重力补偿

  // 陀螺仪信息及其解算
  float gyro_pitch_speed;
  float gyro_pitch_angle;
  float gyro_pitch_accel; // 加速度
  uint32_t last_cnt;
  float delta_t; // 两帧计算之间的时间差

  float target_pitch_angle; // 设定的角度值

  // Yaw在底盘控制
  // Yaw 轴
  PID_t yaw_speed_pid;             // 速度环
  PID_t yaw_angle_pid;             // 角度环
  float ff_yaw[3];                 // 0:角度前馈，1:速度前馈，2:加速度前馈
  float ff_tff;                    // 前馈扭矩
  TD_t pos_yaw_td;                 // 位置跟踪微分器

  // 陀螺仪信息及其解算
  float gyro_yaw_speed;
  float gyro_yaw_angle;
  float gyro_yaw_accel; // 加速度

  float target_yaw_angle;

  // pitch 限位计算
  float pitch_max_angle;
  float pitch_min_angle;
  float chassis_err_angle;   // 底盘pitch误差角度
  float chassis_pitch_angle; // 底盘pitch估计角度

  float pitch_out;
  float yaw_out;

  Feedforward_t follow_gimbal_forward; // 底盘转向前馈

  float if_spin_reverse;   // 是否反拨拨盘
  float stuck_time;        // 卡弹持续时间
  float spin_reverse_time; // 反转时间

  float pc_recv_rad[2]; // PC端发送的数据，弧度制

  gimbal_direction_e gimbal_direction;

  // 系统辨识全局变量
  RLS rls_yaw;
  SI_t si_yaw;
  TD_t td_sysid_omega;
  uint8_t gimbal_sysid_done;
  float gimbal_sysid_last_speed;

} GimbalController;

extern GimbalController gimbal_controller;
void GimbalMotorInit(void);
void GimbalPidInit(void);
void GimbalPidChange(void);
void Auto_GimbalPidChange(void);
void Small_Buff_Change(void);
void GimbalClear(void);
void updateGyro(void);
void Gimbal_ErrorAngle(void);

// 系统辨识
void Gimbal_SystemID_Init(void);
void Gimbal_SystemID_Run(void);

// pitch
void limitPitchAngle(void);
float GimbalPitchComp(void);
float Gimbal_Pitch_Calculate(float set_point);

// Yaw
float Gimbal_Yaw_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
float GimbalFrictionModel(void);

float limit_angle(float in);

#endif // !_GIMBAL_H
