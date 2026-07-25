// 遥控器选择
#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


enum CONTROL_TYPE
{
    DJI_REMOTE_CONTROL, // DJI遥控器
    KEY_MOUSE,          // 键鼠
};
enum ROBOT_STATE
{
    OFFLINE_MODE,
    CONTROL_MODE,
};
enum CHASSIS_MODE_ACTION
{
    NOT_CONTROL_MODE,
    NOT_FOLLOW_GIMBAL, // 仅底盘运动模式
    FOLLOW_GIMBAL,     // 云台跟随模式
    CV_ROTATE,         // 恒速度旋转

};

enum PowerControlState // 功率控制状态
{
    POWER_TO_BATTERY,    // 接电源
    POWER_TO_SuperPower, // 接电容
};

enum GIMBAL_ACTION
{
    GIMBAL_POWER_DOWN, // 云台掉电模式
    GIMBAL_ACT_MODE,
    GIMBAL_AUTO_AIM_MODE,
    GIMBAL_SMALL_BUFF_MODE, // 打符模式
    GIMBAL_BIG_BUFF_MODE,   // 大符
    GIMBAL_AUTO_ATM_TEST_MODE, // 自瞄测试模式
};

enum CHASSIS_FORMAT
{
    X_MODE,    // X型
    CROSS_MODE // 十字型
};

enum GIMBAL_POSITION
{
    POWER_DOWN, // 下电
    DOWN,       // 低头
    UP,         // 抬头
};

typedef struct RemoteController
{
    enum ROBOT_STATE robot_state;   // 机器人状态(掉线模式，控制模式)
    enum ROBOT_STATE last_robot_state;
    enum GIMBAL_ACTION gimbal_action; // 云台模式
    enum GIMBAL_ACTION last_gimbal_action;
    enum CHASSIS_MODE_ACTION control_mode_action;
    enum CHASSIS_MODE_ACTION last_control_mode_action; // 底盘模式
    enum PowerControlState super_power_state;          // 主动电容标志位
    enum GIMBAL_POSITION gimbal_position;              // 设置头部模式

} RemoteController;

void setGimbalAction(enum GIMBAL_ACTION action);
void setControlMode(enum CONTROL_TYPE type);
void setRobotState(enum ROBOT_STATE state);
void setChassisAction(enum CHASSIS_MODE_ACTION action);
void initRemoteControl(enum CONTROL_TYPE type);
void setSuperPower(enum PowerControlState super_power_state);
void setGimbalPosition(enum GIMBAL_POSITION gimbal_position);


extern RemoteController remote_controller;

#endif // !_REMOTE_CONTROL_H
