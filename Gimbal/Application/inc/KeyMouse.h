#ifndef __CHASSIS_SOLVER_H__
#define __CHASSIS_SOLVER_H__

#include "main.h"
#include <stdlib.h>
#include "remote_control.h"
#include "DT7_Controller.h"
#include "VTM_Controller.h"

#include "debug.h"
#include "TD.h"
//#include "Offline_Task.h"

#include "bsp_dwt.h"
#include "Gimbal.h"
#include "pc_serial.h"
#include "ToggleBullet.h"

#include "FrictionWheel.h"

//参考遥控器，假设鼠标速度峰值为300，delta_t为0.01s
#define MOUSE_YAW_SPEED 1.0f/250.0f //t/速度峰值
#define MOUSE_YAW_SENSITIVITY   0.01f*180.0f/400.0f //t*角度/速度峰值
#define MOUSE_PITCH_SENSITIVITY 0.01f*180.0f/400.0f //t*角度/速度峰值
#define MOUSE_SCROLL_SENSITIVITY 0.001f

enum SPEED_STATE_e
{
    NORMAL_SPEED_STATE = 0,
    HIGH_SPEED_STATE
};

/*底盘状态枚举体*/
enum CHASSIS_STATE_e
{
    NOT_CONTROLLING,       // 下电状态
    CONTROLLING,           // 上电状态
    CHASSIS_SENSORS_OFF,   // 传感器离线
    CHASSIS_STATE_ABNORMAL // 底盘状态异常
};

typedef struct ChassisSolver
{
    float chassis_speed_x; // 云台方向速度
    float chassis_speed_y; // 垂直云台方向速度
    float chassis_speed_w; // 小陀螺状态的速度期望
    TD_t speed_x_td; //前后方向速度跟踪微分器
    TD_t speed_y_td; //左右方向速度跟踪微分器
    TD_t speed_w_td; //旋转速度跟踪微分器

    //鼠标返回值是离散的，需要进行平滑处理
    TD_t mouse_x_td; //鼠标x轴速度跟踪微分器
    TD_t mouse_y_td; //鼠标y轴速度跟踪微分器
    
    uint32_t last_cnt;
    float delta_t;

    uint8_t chassis_state; // 底盘状态标志位

    enum SPEED_STATE_e speed_state;
	
	//小陀螺方向变化
	short Rotate_Counter;
	float Rotate_Direction;//取2的余数变向
} ChassisSolver;

extern ChassisSolver chassis_solver;
void KeyMouse_Init(void);
void get_control_info(ChassisSolver *infantry);

void chassis_direction_get(float target_rate, float zeros_rate);

#endif // !_CHASSIS_SOLVER_H
