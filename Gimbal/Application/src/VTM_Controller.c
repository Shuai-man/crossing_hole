#include "VTM_Controller.h"

#include "bsp_VTM.h"
#include "Offline_Task.h"
#include "remote_control.h"
#include "KeyMouse.h"
#include "Gimbal.h"
#include "ToggleBullet.h"

VTM_Controller_t vtm_controller;

void VTM_Init(void)
{
	// 清除虚拟拨杆状态
	vtm_remote.sw[Left_up] = 0;
	vtm_remote.sw[Right_up] = 0;
	vtm_remote.sw[Pause] = 0;
	VTM_State_Clear();
}
void VTM_State_Clear(void)
{
	vtm_controller.lift_flag = 0;
	vtm_controller.last_right_up = 0;
}

void VTM_Fire(void)
{
	if (remote_controller.control_type == REMOTE_CONTROL) // 遥控器开火
	{
		if (vtm_remote.trigger_btn == 1)
		{
			setShootAction(SHOOT_FIRE_MODE);
			if ((vtm_remote.ch[RIGHT_LR] < (CH_MIDDLE - 150)) || (vtm_remote.ch[RIGHT_LR] > (CH_MIDDLE + 150)))
			{
				toggle_controller.is_shoot = 1;
			}
			else
			{
				toggle_controller.is_shoot = 0;
			}
		}
		else
		{
			setShootAction(SHOOT_POWER_DOWN_MODE);
			toggle_controller.is_shoot = 0;
		}
	}
	else if (remote_controller.control_type == KEY_MOUSE)
	{
		if (vtm_remote.sw[Right_up] == 1)
		{
			setShootAction(SHOOT_FIRE_MODE);
		}
		else
		{
			setShootAction(SHOOT_POWER_DOWN_MODE);
		}
	}
}

void VTM_Gimbal_Ctrl(float delta_t)
{
	// 云台控制
	gimbal_controller.target_yaw_angle -= (vtm_remote.ch[LEFT_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * delta_t * 0.9f;
	gimbal_controller.target_pitch_angle += (vtm_remote.ch[LEFT_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * delta_t * 0.9f;
}

void VTM_Chassis_Ctrl(void)
{
	// 底盘控制
	if ((vtm_remote.ch[RIGHT_UD] - CH_MIDDLE) > 50 || (vtm_remote.ch[RIGHT_UD] - CH_MIDDLE) < -50)
	{
		chassis_solver.chassis_speed_x = (float)(vtm_remote.ch[RIGHT_UD] - CH_MIDDLE) / CH_RANGE;
	}
	else
	{
		chassis_solver.chassis_speed_x = 0;
	}
	// 左右移动
	if (remote_controller.shoot_action == SHOOT_FIRE_MODE)
	{
		chassis_solver.chassis_speed_y = 0;
	}
	else if ((vtm_remote.ch[RIGHT_LR] - CH_MIDDLE) > 50 || (vtm_remote.ch[RIGHT_LR] - CH_MIDDLE) < -50)
	{
		chassis_solver.chassis_speed_y = (float)(vtm_remote.ch[RIGHT_LR] - CH_MIDDLE) / CH_RANGE;
	}
	else
	{
		chassis_solver.chassis_speed_y = 0;
	}
	// 旋转
	if (remote_controller.gimbal_position == DOWN) // 过洞姿态下，不允许小陀螺
	{
		setChassisModeAction(FOLLOW_GIMBAL);
		chassis_solver.chassis_speed_w = 0.0f;
	}
	else if ((vtm_remote.wheel < (CH_MIDDLE - 50)) || (vtm_remote.wheel > (CH_MIDDLE + 50))) // 阈值
	{
		setChassisModeAction(CV_ROTATE);
		setSuperPower(POWER_TO_SuperPower);
		chassis_solver.chassis_speed_w = (float)(vtm_remote.wheel - CH_MIDDLE) / CH_RANGE;
	}
	else
	{
		setChassisModeAction(FOLLOW_GIMBAL);
		setSuperPower(POWER_TO_BATTERY);
		chassis_solver.chassis_speed_w = 0.0f;
	}
}

void VTM_Lift(void)
{
	if (vtm_remote.custom_btn_right == 1 && vtm_controller.last_right_up == 0)
	{
		vtm_controller.lift_flag = !vtm_controller.lift_flag;
	}
	vtm_controller.last_right_up = vtm_remote.custom_btn_right;
	if (vtm_controller.lift_flag)
	{
		setGimbalPosition(DOWN);
	}
	else
	{ // 默认抬头
		setGimbalPosition(UP);
	}
}

void VTM_Update(float delta_t)
{

	switch (vtm_remote.gear)
	{
	case gear_C: // 下电模式
		setGameModeAction(OFF_MODE);
		VTM_Init();
		break;
	case gear_N: // 键鼠模式
		initRemoteControl(KEY_MOUSE);
		VTM_Init();
		break;
	case gear_S: // 遥控器模式
		if (vtm_remote.sw[Pause] == 1)
		{
			setGameModeAction(OFF_MODE);
		}
		else if (vtm_remote.sw[Left_up] == 1)
		{
			setGameModeAction(GAME_MODE);
			VTM_Fire();
			VTM_Lift();
			VTM_Gimbal_Ctrl(delta_t);
			VTM_Chassis_Ctrl();
		}
		else
		{
			VTM_Fire(); // 保留开火
			setGameModeAction(TEST_MODE);
			// 保留云台控制
			VTM_Gimbal_Ctrl(delta_t);
			chassis_solver.chassis_speed_x = 0.0f;
			chassis_solver.chassis_speed_y = 0.0f;
			chassis_solver.chassis_speed_w = 0.0f;
		}
		break;
	default:
		setGameModeAction(OFF_MODE);
		VTM_Init();
		break;
	}
}
