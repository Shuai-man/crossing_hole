#include "remote_control.h"

RemoteController remote_controller;

// int size_test = 0;
void setChassisModeAction(enum CHASSIS_MODE_ACTION action)
{
    // size_test = sizeof(remote_controller.control_mode_action); //1 字节
    if (remote_controller.robot_state == CONTROL_MODE)
    {
        remote_controller.chassis_mode_action = action;
        if (action != NOT_CONTROL_MODE)
        {
            remote_controller.last_chassis_mode_action = action;
        }
    }
}

void setGameModeAction(enum GAME_MODE action)
{
    remote_controller.last_game_mode = remote_controller.game_mode;
    remote_controller.game_mode = action;
    // 检测游戏模式是否改变
    if (remote_controller.game_mode != remote_controller.last_game_mode)
    {
        // 初始化游戏模式
        switch (remote_controller.game_mode)
        {
        case OFF_MODE:
            setChassisModeAction(NOT_CONTROL_MODE);
            setShootAction(SHOOT_POWER_DOWN_MODE);
            break;
        case TEST_MODE:
            Test_Init();
            break;
        case GAME_MODE:
            Game_Init();
            break;
        default:
            setAllModeOff();
            break;
        }
    }
}

void setGimbalAction(enum GIMBAL_ACTION action)
{
    remote_controller.gimbal_action = action;
}

void setShootAction(enum SHOOT_ACTION action)
{
    remote_controller.shoot_action = action;
}

void RemoteGet(void)
{
    // 获取遥控器数据
    if (global_debugger.DT7_debugger.state == ON)
    {
        remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] = dt7_remote.ch[RIGHT_CH_LR];
        remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] = dt7_remote.ch[RIGHT_CH_UD];
        remote_controller.dji_remote.rc.ch[LEFT_CH_UD] = dt7_remote.ch[LEFT_CH_UD];
        remote_controller.dji_remote.rc.ch[LEFT_CH_LR] = dt7_remote.ch[LEFT_CH_LR];

        remote_controller.dji_remote.mouse.x = dt7_remote.x;
        remote_controller.dji_remote.mouse.y = dt7_remote.y;
        remote_controller.dji_remote.mouse.z = dt7_remote.z;
        remote_controller.dji_remote.mouse.press_l = dt7_remote.press_l;
        remote_controller.dji_remote.mouse.press_r = dt7_remote.press_r;
        remote_controller.dji_remote.keyValue = dt7_remote.keyValue;
    }
    else if (global_debugger.VTM_debugger.state == ON)
    {
        remote_controller.dji_remote.rc.ch[RIGHT_LR] = vtm_remote.ch[RIGHT_LR];
        remote_controller.dji_remote.rc.ch[RIGHT_UD] = vtm_remote.ch[RIGHT_UD];
        remote_controller.dji_remote.rc.ch[LEFT_UD] = vtm_remote.ch[LEFT_UD];
        remote_controller.dji_remote.rc.ch[LEFT_LR] = vtm_remote.ch[LEFT_LR];

        remote_controller.dji_remote.mouse.x = vtm_remote.mouse_x;
        remote_controller.dji_remote.mouse.y = vtm_remote.mouse_y;
        remote_controller.dji_remote.mouse.z = vtm_remote.mouse_z;
        remote_controller.dji_remote.mouse.press_l = vtm_remote.mouse_left;
        remote_controller.dji_remote.mouse.press_r = vtm_remote.mouse_right;
        remote_controller.dji_remote.keyValue = vtm_remote.keyboard;

        Virtual_SW(&vtm_remote);
    }
    else
    {
        RC_Rst();
    }
}

void RC_Rst(void)
{
    for (int i = 0; i < 4; i++)
    {
        remote_controller.dji_remote.rc.ch[i] = CH_MIDDLE;
    }
    remote_controller.dji_remote.mouse.x = 0;
    remote_controller.dji_remote.mouse.y = 0;
    remote_controller.dji_remote.mouse.z = 0;
    remote_controller.dji_remote.mouse.press_l = 0;
    remote_controller.dji_remote.mouse.press_r = 0;

    remote_controller.dji_remote.keyValue = 0;
}

void setControlMode(enum CONTROL_TYPE type)
{
    remote_controller.control_type = type;
}

void setSuperPower(enum PowerControlState super_power_state)
{
    remote_controller.super_power_state = super_power_state;
}

void setRobotState(enum ROBOT_STATE state)
{
    remote_controller.robot_state = state;
    if (remote_controller.robot_state != CONTROL_MODE)
    {
        remote_controller.chassis_mode_action = NOT_CONTROL_MODE;
        setChassisModeAction(NOT_CONTROL_MODE);
    }
}
void initRemoteControl(enum CONTROL_TYPE type)
{
    setControlMode(type);

    if (type == REMOTE_CONTROL)
    {
        RC_Rst();
    }
}

void setGimbalPosition(enum GIMBAL_POSITION position)
{
    remote_controller.gimbal_position = position;
}

void setAllModeOff(void)
{
    setRobotState(OFFLINE_MODE);
    setChassisModeAction(NOT_CONTROL_MODE);
    setShootAction(SHOOT_POWER_DOWN_MODE);
    setGimbalAction(GIMBAL_POWER_DOWN);
    setGimbalPosition(POWER_DOWN);
}

/*
 *模式初始化
 */
void Test_Init(void)
{
    // 自瞄测试 底盘趴下
    setRobotState(CONTROL_MODE);
    setGimbalAction(GIMBAL_ACT_MODE);
    setGimbalPosition(UP);
    setChassisModeAction(NOT_CONTROL_MODE);
    setSuperPower(POWER_TO_BATTERY);
}
void Game_Init(void)
{
    setRobotState(CONTROL_MODE);
    setGimbalAction(GIMBAL_ACT_MODE);
    setGimbalPosition(UP);
    setChassisModeAction(FOLLOW_GIMBAL); // 当进入到这个模式的时候，先进入底盘跟随模式
    setSuperPower(POWER_TO_BATTERY);
}
