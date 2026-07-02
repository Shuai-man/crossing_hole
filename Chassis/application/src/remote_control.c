#include "remote_control.h"

RemoteController remote_controller;

// int size_test = 0;
void setChassisAction(enum CHASSIS_MODE_ACTION action)
{
    // size_test = sizeof(remote_controller.control_mode_action); //1 字节
    if (remote_controller.robot_state == CONTROL_MODE)
    {
        remote_controller.last_control_mode_action = remote_controller.control_mode_action;
        remote_controller.control_mode_action = action;
    }
}

void setGimbalAction(enum GIMBAL_ACTION action)
{
    remote_controller.last_gimbal_action = remote_controller.gimbal_action;
    remote_controller.gimbal_action = action;
}

void setSuperPower(enum PowerControlState super_power_state)
{
    remote_controller.super_power_state = super_power_state;
}

void setRobotState(enum ROBOT_STATE state)
{
    remote_controller.last_robot_state = remote_controller.robot_state;
    remote_controller.robot_state = state;
    if (remote_controller.robot_state != CONTROL_MODE)
    {
        remote_controller.control_mode_action = NOT_CONTROL_MODE;
        setChassisAction(NOT_CONTROL_MODE);
    }
}

void setGimbalPosition(enum GIMBAL_POSITION gimbal_position)
{
    remote_controller.gimbal_position = gimbal_position;
}

void initRemoteControl(enum CONTROL_TYPE type)
{
    setRobotState(OFFLINE_MODE);
}
