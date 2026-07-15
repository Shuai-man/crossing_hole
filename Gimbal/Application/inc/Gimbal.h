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
#define GIMBAL_PITCH_MOTOR_SIGN -1.0f // 云台PITCH电机方向，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_YAW_MOTOR_SIGN -1.0f // 用来标记电机的方向，逆时针为正
#define GIMBAL_YAW_GYRO_SIGN 1.0f   // 用来标记gyro的方向，逆时针为正

// 云台角度限位
#define GIMBAL_ANGLE_MAX 40.0f // 实测最大40度
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

//系统辨识模式
#define GIMBAL_SYSID 0

// 系统辨识步骤选择（Pitch 推荐按 GRAVITY -> BC -> J 执行）
#define GIMBAL_SYSID_STEP_GRAVITY 1           // 双向等速扫描 -> 辨识重力矩
#define GIMBAL_SYSID_STEP_BC 2                // 多速度双向扫描 -> 辨识 B、C
#define GIMBAL_SYSID_STEP_J 3                 // 同速度窗口加减速配对 -> 辨识 J
#define GIMBAL_SYSID_STEP_ALL 4               // 当前辨识轴自动依次完成全部阶段
#define GIMBAL_SYSID_STEP GIMBAL_SYSID_STEP_ALL

// YAW轴 系统辨识
#define GIMBAL_YAW_SYSID 1
// YAW轴系统辨识参数
#if ROBOT_SELECT == OLD
#define GIMBAL_YAW_J 3.08758259f // 转动惯量 //实测前馈输出过大，可以适当降低惯量，以减小前馈输出
#define GIMBAL_YAW_B 1.21354508f   // 阻尼系数，与速度有关
#define GIMBAL_YAW_C 315.488129f  // 库伦摩擦系数，与结构有关
#elif ROBOT_SELECT == NEW
#define GIMBAL_YAW_J 2.61310053f // 转动惯量 //实测前馈输出过大，可以适当降低惯量，以减小前馈输出
#define GIMBAL_YAW_B 1.9250102f   // 阻尼系数，与速度有关
#define GIMBAL_YAW_C 318.858643f  // 库伦摩擦系数，与结构有关
#endif

// Pitch轴 重补测试开关（0=关闭, 1=开启）
#define GIMBAL_PITCH_COMP 2
/* 重力补偿 */
#if ROBOT_SELECT == OLD
#define GIMBAL_PITCH_SIN 606.6655f  // 重力矩系数1
#define GIMBAL_PITCH_COS 1723.191f  // 重力矩系数2
#elif ROBOT_SELECT == NEW
#define GIMBAL_PITCH_SIN 219.408646f  // 重力矩系数1
#define GIMBAL_PITCH_COS 1499.73376f  // 重力矩系数2
#endif
/*PITCH系统辨识*/
#define GIMBAL_PITCH_SYSID 3
#if ROBOT_SELECT == OLD
#define GIMBAL_PITCH_B 13.6324844f // 阻力系数
#define GIMBAL_PITCH_C 153.83098f // 摩擦力
#define GIMBAL_PITCH_J 2.0f         // 转动惯量
#elif ROBOT_SELECT == NEW
#define GIMBAL_PITCH_B 0.190167725f // 阻力系数
#define GIMBAL_PITCH_C 318.399139f // 摩擦力
#define GIMBAL_PITCH_J 0.330432951f         // 转动惯量
#endif
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
  float B_raw;       // B 的未约束拟合值；轻微负值时 B 会被钳位为 0
  float C;
  float G_sin;       // Pitch 重力矩 sin(theta) 系数
  float G_cos;       // Pitch 重力矩 cos(theta) 系数
  float fit_rmse;    // 当前阶段拟合残差（电机反馈力矩单位）
  float gravity_rmse;
  float bc_rmse;
  float j_rmse;
  float J_pair_min;    // 有效加减速差分点中最小的局部惯量
  float J_pair_max;    // 有效加减速差分点中最大的局部惯量
  float j_alpha_filtered; // 低通角加速度，仅用于观察，不参与阶段平均回归
  float j_signal_rms;  // J*alpha 的 RMS，表示有效惯量力矩强度
  float j_residual_ratio; // j_rmse / j_signal_rms，越小越好
  float j_velocity_ref; // 当前 J 测试速度参考值
  float j_switch_angle; // Pitch为切换角度；Yaw为测试最大参考速度
  float j_target_accel; // 当前运动阶段的目标加速度，减速阶段为负
  /* 最近平均点：B/C保存行程均值；J保存同角度/同速度加减速差分。 */
  float mean_torque;
  float mean_omega;
  float mean_gravity;
  float mean_input;    // B/C为sgn(omega)，J为delta_alpha
  float mean_residual; // B/C为T-G，J为delta_T-B*delta_omega-delta_G
  uint32_t mean_raw_count;
  uint32_t sample_count;
  uint32_t gravity_valid_bins;
  uint32_t bc_sample_count;
  uint32_t j_sample_count;
  uint8_t mean_point_count; // 当前阶段已生成的等权平均点数量
  uint8_t j_motion_phase; // 当前轴J测试运动阶段，具体含义见实现中的阶段枚举
  uint8_t j_pass_count;   // Pitch为上升趟数；Yaw为已完成方向数
  uint8_t sysid_stage;
  uint8_t sysid_valid;
  uint8_t sysid_error;
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
