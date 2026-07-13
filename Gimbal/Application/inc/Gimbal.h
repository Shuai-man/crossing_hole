#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "main.h"

#include "Robot_config.h"

#include "GM6020.h"
#include "DM_Motor.h"

#include "ins.h"
#include "remote_control.h"
#include "pc_serial.h"

#include "pid.h"
#include "my_filter.h"
#include "TD.h"
#include "RLS_Identification.h"
#include "SystemIdentification.h"

#include "GimbalSystemID.h"

// pitch
#define GIMBAL_PITCH_GYRO_SIGN 1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN -1.0f // 云台PITCH电机方向，向上为正

// 云台角度限位
#define GIMBAL_ANGLE_MAX 30.0f // 实测最大40度
#define GIMBAL_ANGLE_MIN -10.0f

// Pitch角度机械零点
#if ROBOT_SELECT == OLD
#define GIMBAL_PITCH_ZERO 215.172729f 
#elif ROBOT_SELECT ==NEW
#define GIMBAL_PITCH_ZERO 207.674561f
#endif

// 云台底盘的yaw轴零点都需要更改
#if ROBOT_SELECT == OLD
#define GIMBAL_ANGLE_ZERO 9.51965332f 
#elif ROBOT_SELECT ==NEW
#define GIMBAL_ANGLE_ZERO 105.309448f
#endif

// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_YAW_MOTOR_SIGN -1.0f // 用来标记电机的方向，逆时针为正
#define GIMBAL_YAW_GYRO_SIGN 1.0f   // 用来标记gyro的方向，逆时针为正
// 系统辨识参数
#define GIMBAL_YAW_J 3.08758259f // 转动惯量 //实测前馈输出过大，可以适当降低惯量，以减小前馈输出
#define GIMBAL_YAW_B 1.21354508f // 2.4f  // 阻尼系数，与速度有关
#define GIMBAL_YAW_C 315.488129f // 220.0f // 库伦摩擦系数，与结构有关

#define GIMBAL_SYSID 0
// YAW轴 系统辨识
#define GIMBAL_YAW_SYSID 1
// 系统辨识步骤选择（配合 GIMBAL_SYSID=1 使用）
#define GIMBAL_SYSID_STEP_BC 1                // 第一步：稳态速度测试 -> 辨识 B (阻尼) 和 C (库仑摩擦)
#define GIMBAL_SYSID_STEP_J 2                 // 第二步：恒加速测试 -> 辨识 J (转动惯量)，需先用第一步得到 B,C
#define GIMBAL_SYSID_STEP GIMBAL_SYSID_STEP_J // 默认执行第一步

// Pitch轴 重补测试开关（0=关闭, 1=开启）
#define GIMBAL_PITCH_COMP 3
/* 重力补偿 */
#define GIMBAL_PITCH_A 606.6655f  // 重力矩系数1
#define GIMBAL_PITCH_B 1723.191f  // 重力矩系数2
#define GIMBAL_PITCH_C 153.83098f // 摩擦力
#define GIMBAL_PITCH_COMP_MAX 1900.0f
#define GIMBAL_PITCH_COMP_MIN 1500.0f

/*PITCH系统辨识*/
#define GIMBAL_PITCH_SYSID 2
#define GIMBAL_PITCH_CB 13.6324844f // 阻力系数
#define GIMBAL_PITCH_J 2.0f         // 转动惯量

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

/* 系统辨识全局变量 */
typedef struct Gimbal_SI
{
  float sysid_timer;
  RLS rls_sysid;
  SI_t si_sysid;
  TD_t td_omega;
  uint8_t sysid_done;
  float J;
  float B;
  float C;
} Gimbal_SI;

typedef struct GimbalController
{
  uint32_t last_cnt;
  float delta_t; // 两帧计算之间的时间差

  DM_MIT DM_Yaw_Motor;
  DM_MIT DM_Pitch_Motor;

  // 转向控制
  // 最小回正角度
  float err_angle;     // 初始角度误差
  float err_angle_180; // 误差余角，用于判断方向
  uint8_t return_flag; // 回正标志位，0是完成，1是开始回正，2是正在回正
  gimbal_direction_e gimbal_direction;
  /*----------Pitch 轴-----------------*/
  // 陀螺仪信息及其解算
  float gyro_pitch_speed;
  float gyro_pitch_angle;
  float gyro_pitch_accel; // 加速度

  TD_t pos_pitch_td;     // 位置跟踪微分器
  PID_t pitch_speed_pid; // 速度环
  PID_t pitch_angle_pid; // 角度环

  float gravity_comp;       // 重力补偿
  float ff_tff_pitch;       // 前馈扭矩
  float target_pitch_angle; // 设定的角度值

  float pitch_out;

  // pitch 限位计算
  float pitch_max_angle;
  float pitch_min_angle;
  /*----------Yaw 轴-----------------*/
  // 陀螺仪信息及其解算
  float gyro_yaw_speed;
  float gyro_yaw_angle;
  float gyro_yaw_accel; // 加速度

  // Yaw 轴
  TD_t pos_yaw_td;     // 位置跟踪微分器
  PID_t yaw_speed_pid; // 速度环
  PID_t yaw_angle_pid; // 角度环

  float ff_tff; // 前馈扭矩
  float target_yaw_angle;

  float yaw_out;
  /*----------底盘控制-----------------*/
  // 底盘姿态估计
  float chassis_err_angle;   // 底盘pitch误差角度
  float chassis_pitch_angle; // 底盘pitch估计角度

  Feedforward_t follow_gimbal_forward; // 底盘转向前馈

  /*----------反拨检测-----------------*/
  float if_spin_reverse;   // 是否反拨拨盘
  float stuck_time;        // 卡弹持续时间
  float spin_reverse_time; // 反转时间

  /*----------系统辨识-----------------*/
  Gimbal_SI yaw_sysid;
  Gimbal_SI pitch_sysid;

} GimbalController;

extern GimbalController gimbal_controller;
void GimbalMotorInit(void);
void GimbalPidInit(void);
void GimbalClear(void);
void updateGyro(void);
void Gimbal_ErrorAngle(void);

// pitch
void limitPitchAngle(void);
float GimbalPitchComp(void);
float Gimbal_Pitch_Calculate(float set_point);

// Yaw
float Gimbal_Yaw_Calculate(float set_point);
float GimbalFrictionModel(void);

float limit_angle(float in);

#endif // !_GIMBAL_H
