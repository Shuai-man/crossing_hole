// 遥控器选择
#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "bsp_VTM.h"
#include "bsp_DT7.h"
#include "Offline_Task.h"
#include "lifting_control.h"
#include "Gimbal.h"

enum REMOTE_TYPE // 遥控器选择
{
    DT7_RM = 0,
    VTM_RM = 1,
};

enum CONTROL_TYPE
{
    REMOTE_CONTROL = 0, // 遥控器控制
    KEY_MOUSE,          // 键鼠
};

/*
 *控制模式
 *测试模式 比赛模式
 */
enum GAME_MODE
{
    OFF_MODE = 0,  // 0,下电模式
    TEST_MODE = 1, // 测试模式
    GAME_MODE = 2, // 比赛模式
};

/*
 *       机器人状态
 * 控制状态    掉线状态
 * 底盘 云台    全下电
 * 分别控制
 */
enum ROBOT_STATE
{
    OFFLINE_MODE, // 掉线状态
    CONTROL_MODE, // 控制状态
};

// 模式与底盘状态接模式对应
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
};

enum SHOOT_ACTION
{
    SHOOT_POWER_DOWN_MODE, // 掉电模式
    SHOOT_FIRE_MODE,       // 射击模式
};

enum GIMBAL_POSITION
{
    POWER_DOWN, // 下电
    DOWN,       // 低头
    UP,         // 抬头
};

#pragma pack(1)
/*遥控器结构体*/
typedef struct
{
    unsigned short ch[4]; // 摇杆
} Remote;

/*鼠标结构体*/
typedef struct
{
    short x;
    short y;
    short z;
    unsigned char press_l;
    unsigned char press_r;

    // 按键检测
    unsigned char last_press_l;
    unsigned char last_press_r;
    unsigned char mouseChangeOn_l;  // 检测按键值从0转1改变
    unsigned char mouseChangeOff_l; // 检测按键值从1转0转变
    unsigned char mouseChangeOn_r;  // 检测按键值从0转1改变
    unsigned char mouseChangeOff_r; // 检测按键值从1转0转变
} Mouse;

#define KEY_B 0x8000
#define KEY_V 0x4000
#define KEY_C 0x2000
#define KEY_X 0x1000
#define KEY_Z 0x0800
#define KEY_G 0x0400
#define KEY_F 0x0200
#define KEY_R 0x0100
#define KEY_E 0x0080
#define KEY_Q 0x0040
#define KEY_CTRL 0x0020
#define KEY_SHIFT 0x0010
#define KEY_D 0x0008
#define KEY_A 0x0004
#define KEY_S 0x0002
#define KEY_W 0x0001

/*遥键鼠结构体综合*/
typedef struct
{
    Remote rc;         // 遥控器
    Mouse mouse;       // 鼠标
    uint16_t keyValue; // 按键

    // 以下是检测按键触发状态变量
    uint16_t last_keyValue;
    uint16_t keyChangeOn;  // 检测按键值从0转1改变
    uint16_t keyChangeOff; // 检测按键值从1转0转变
} RC_Ctl_t;

#pragma pack()

typedef struct RemoteController
{
    enum CONTROL_TYPE control_type; // 控制类型(遥控器，蓝牙等)
    enum ROBOT_STATE robot_state;   // 机器人状态(掉线模式，控制模式)
    enum GAME_MODE game_mode; // 游戏模式(下电模式，测试模式，比赛模式)
    enum GAME_MODE last_game_mode;
    enum GIMBAL_ACTION gimbal_action; // 云台
    enum SHOOT_ACTION shoot_action; // 打弹模式
    enum CHASSIS_MODE_ACTION chassis_mode_action;// 底盘模式(平衡模式，倒地自救模式)
    enum PowerControlState super_power_state;
    enum GIMBAL_POSITION gimbal_position; // 头部模式

    RC_Ctl_t dji_remote;

    /* 标志位 */
    int8_t fire_flag;
    int8_t reverse_flag;      // 拨盘反拨标志位
    int8_t single_shoot_flag; // 仅单发时有效，打击标志位
    int8_t auto_arm;          // 使用辅瞄

} RemoteController;

void setGimbalAction(enum GIMBAL_ACTION action);
void setShootAction(enum SHOOT_ACTION action);
void setGameModeAction(enum GAME_MODE action);
void setControlMode(enum CONTROL_TYPE type);
void setRobotState(enum ROBOT_STATE state);
void setChassisModeAction(enum CHASSIS_MODE_ACTION action);
void initRemoteControl(enum CONTROL_TYPE type);
void setSuperPower(enum PowerControlState super_power_state);
void setGimbalPosition(enum GIMBAL_POSITION gimbal_position);
void setAllModeOff(void);
void Test_Init(void);
void Game_Init(void);

// 遥控器
void RemoteGet(void);

void RC_Rst(void);

extern RemoteController remote_controller;

#endif // !_REMOTE_CONTROL_H
