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

// pitch
#define GIMBAL_PITCH_GYRO_SIGN -1.0f // pitch符号，向上为正
#define GIMBAL_PITCH_BIAS 0.0f      // pitch最低角度(imu测得) - 实际最低角度(机械处测得)

#define GIMBAL_PITCH_MOTOR_SIGN -1.0f // 云台PITCH电机方向，向上为正

//云台角度限位
#define GIMBAL_ANGLE_MAX 20.0f
#define GIMBAL_ANGLE_MIN -10.0f


#define GIMBAL_PITCH_COMP 4000.0f
#define GIMBAL_PITCH_COMP_COEF 1.0f

//Pitch机械零点角度
#define GIMBAL_PITCH_ZERO 209.229126f

//云台底盘的yaw轴零点都需要更改  
#define GIMBAL_ANGLE_ZERO 48.40f


// yaw
// 作为云台控制的yaw角度需要以逆时针为正(角度增加)
#define GIMBAL_YAW_MOTOR_SIGN -1.0f       // 用来标记电机的方向，逆时针为正
#define GIMBAL_YAW_GYRO_SIGN 1.0f        // 用来标记gyro的方向，逆时针为正
#define GIMBAL_YAW_POS_FORWARD_COEF 0.3f // 角度环前馈系数
#define GIMBAL_YAW_SPEED_FORWARD_COEF 0.f
#define GIMBAL_YAW_J 4.15f
#define GIMBAL_YAW_B 18.54f

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
	
    // Pitch 轴
    PID_t pitch_current_pid;           // 电流环
    PID_t pitch_speed_pid;             // 速度环
    PID_t pitch_angle_pid;             // 角度环
    Feedforward_t pitch_speed_forward; // 速度环前馈
    Feedforward_t pitch_angle_forward; // 角度环前馈


    float comp_pitch_current; // 重力补偿

    // 陀螺仪信息及其解算
    float gyro_pitch_speed;
    float gyro_pitch_angle;
    float gyro_last_pitch_angle;
    uint32_t last_cnt;
    float delta_t; // 两帧计算之间的时间差

    float target_pitch_angle; // 设定的角度值

    // Yaw在底盘控制
    // Yaw 轴
    PID_t yaw_current_pid;           // 电流环
    PID_t yaw_speed_pid;             // 速度环
    PID_t yaw_angle_pid;             // 角度环
    Feedforward_t yaw_speed_forward; // 速度环前馈
    Feedforward_t yaw_angle_forward; // 角度环前馈

    // 陀螺仪信息及其解算
    float gyro_yaw_speed;
    float gyro_yaw_angle;
    float gyro_last_yaw_angle;

    float target_yaw_angle;

    TD_t pos_yaw_td; // 位置跟踪微分器
    TD_t speed_yaw_td;

    // pitch 限位计算
    float pitch_max_angle;
    float pitch_min_angle;

    float pitch_out;
		float yaw_out;
		
    float if_spin_reverse;   // 是否反拨拨盘
    float stuck_time;        // 卡弹持续时间
    float spin_reverse_time; // 反转时间

    uint8_t turn_back_start_flag;  // 一键掉头标志位
    uint8_t turn_back_finish_flag; // 上一帧一键掉头标志位

    float pc_recv_rad[2]; // PC端发送的数据，弧度制

    bool return_flag; //yaw轴回正：0是完成，1是正在回正

		
	gimbal_direction_e gimbal_direction;
	
} GimbalController;

extern GimbalController gimbal_controller;
void GimbalMotorInit(void);
void GimbalPidInit(void);
void GimbalPidChange(void);
void Auto_GimbalPidChange(void);
void Small_Buff_Change(void);
void Gimbal_Get_Dir(float target_angle,float zero_angle);
void GimbalClear(void);
void updateGyro(void);
void Gimbal_Return(void);

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
