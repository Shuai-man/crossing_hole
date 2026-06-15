#ifndef __CHASSIS_SOLVER_H__
#define __CHASSIS_SOLVER_H__

#include "main.h"

#include "remote_control.h"
#include "DT7_Controller.h"
#include "VTM_Controller.h"

#include "debug.h"

//#include "Offline_Task.h"

#include "bsp_dwt.h"
#include "Gimbal.h"
#include "pc_serial.h"
#include "ToggleBullet.h"

#include "FrictionWheel.h"



#define SPEED_W_MAX 3.5f    // 最大角速度
#define MAX_SPEED_X 80.0f // 最前进速度 m/s
#define MAX_SPEED_Y 80.0f // 最右左速度 m/s


#define MAX_SW_YAW_SPEED 180  // 云台yaw轴灵敏度(拨杆) 度/s
#define MAX_SW_PITCH_SPEED 80 // 云台yaw轴灵敏度(拨杆) 度/s

#define SELF_ROTATE_MODE 0// 小陀螺模式
#define _FLY_MODE 1// 飞坡模式
#define SHOOT_MODE 2// 射击模式
#define TEST_MODE_SELECT1 SELF_ROTATE_MODE//遥控器左中右上测试模式选择
#define TEST_MODE_SELECT2 SHOOT_MODE//遥控器左上右上测试模式选择

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


typedef struct RobotInfo
{
    enum CHASSIS_STATE_e chassis_state; // 底盘状态
    uint8_t if_communication_lost;      // 是否上下位机通信丢失
    float delta_t;                      //  当前到上一帧底盘数据的时间间隔
    uint32_t last_cnt;

    uint32_t yaw_last_cnt;
    float yaw_delta_t;

    uint32_t pitch_last_cnt;
    float pitch_delta_t;

} RobotInfo;

typedef struct Turn_Back
{
	uint8_t turn_finish;
}Turn_Back;

typedef struct ChassisSolver
{
    float chassis_speed_x; // 云台方向速度
    float chassis_speed_y; // 垂直云台方向速度
    float chassis_speed_w; // 小陀螺状态的速度期望

    // yaw轴云台
    uint32_t last_cnt;
    float delta_t;

    uint8_t chassis_state; // 底盘状态标志位

    enum SPEED_STATE_e speed_state;
	
	//小陀螺方向变化
	short Rotate_Counter;
	float Rotate_Direction;//取2的余数变向
} ChassisSolver;

extern RobotInfo robotInfo;
extern ChassisSolver chassis_solver;

void get_control_info(ChassisSolver *infantry);

void InfoUnpack(ChassisGetPack_1 *pack);
void chassis_direction_get(float target_rate, float zeros_rate);

#endif // !_CHASSIS_SOLVER_H
