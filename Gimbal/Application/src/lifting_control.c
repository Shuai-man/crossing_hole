#include "lifting_control.h"

LiftingController lifting_controller;

// todo 上升过程如果TOF距离突然变小，认为有障碍物，需要停止上升
// 可以增加2006的里程辅助控制

// todo 车头在后方时不允许下降

void LiftPidInit(void)
{
	lifting_controller.high_state = GIMBAL_HIGH_STATE;
	lifting_controller.low_state = GIMBAL_LOW_STATE;
	lifting_controller.lift_state = remote_controller.gimbal_position;

	PID_Init(&lifting_controller.lift_pos_pid, LIFTING_SPEED_MAX, 0, 0, 25.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
	PID_Init(&lifting_controller.lift_speed_pid, C610_MAX_CURRENT, 3.0f, 0, 0.015f, 0.0, 0.0f, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate);

	lifting_controller.error = false;
	lifting_controller.is_up = false;
	lifting_controller.last_distance = 0;	
	lifting_controller.recovery_ref = 0;
}

// 检查状态是否需要切换
void Lifting_State_Check(void)
{
	if (lifting_controller.lift_state != remote_controller.gimbal_position)
	{
		lifting_controller.lift_state = remote_controller.gimbal_position;
		lifting_controller.finished = 0;
		lifting_controller.error = false;
		lifting_controller.is_up = false;
		lifting_controller.last_distance = 0;
		lifting_controller.recovery_ref = 0;
		//云台回正
		gimbal_controller.target_pitch_angle = 5.0f;
		gimbal_controller.return_flag = 1;
	}
}

void Lifting_Error_Check(float target_height)
{
	// 首次运行初始化
	if (lifting_controller.last_distance == 0)
	{
		lifting_controller.last_distance = Tof_ReceiveData.distance;
		return;
	}

	// 方向未确认：等待距离开始朝目标方向增长
	if (!lifting_controller.is_up)
	{
		if (Tof_ReceiveData.distance > lifting_controller.last_distance )
		{
			lifting_controller.is_up = true;
		}
		lifting_controller.last_distance = Tof_ReceiveData.distance;
		return; // 确认前不检测阻塞
	}

	//Tof离线
	if(global_debugger.tof_debugger.state != ON)
	{
		lifting_controller.error = true;
		return;
	}

	float pos_err = fabs(Tof_ReceiveData.distance - target_height);

	if (pos_err > OBSTACLE_ERR_THRESHOLD)
	{
		if (lifting_controller.error)
		{
			// 已阻塞：检查距离是否回到阻挡前位置
			if (Tof_ReceiveData.distance >= lifting_controller.recovery_ref - OBSTACLE_DELTA_THRESHOLD)
			{
				lifting_controller.error = false;
				lifting_controller.recovery_ref = 0;
			}
		}
		else
		{
			// 正常上升：检查是否被遮挡
			if (Tof_ReceiveData.distance + OBSTACLE_DELTA_THRESHOLD < lifting_controller.last_distance)
			{
				lifting_controller.error = true;
				lifting_controller.recovery_ref = lifting_controller.last_distance;
			}
		}
	}
	else
	{
		// 接近目标：稳态区域，不检测
		lifting_controller.error = false;
		lifting_controller.recovery_ref = 0;
	}

	// last_distance 每周期更新，不冻结
	lifting_controller.last_distance = Tof_ReceiveData.distance;
}

void Lifting_Finish_Check(void)
{
	if (fabs(lifting_controller.lift_pos_pid.Err) > POSE_THRESHOLD || fabs(lifting_controller.send_current) > CURRENT_THRESHOLD)
	{
		lifting_controller.finish_cnt = 0;
		return;
	}
	lifting_controller.finish_cnt++;
	if (lifting_controller.finish_cnt >= FINISH_CNT)
	{
		lifting_controller.finished = 1;
		lifting_controller.finish_cnt = FINISH_CNT;
	}
}

void Lifting_Clear(void)
{
	PID_Clear(&lifting_controller.lift_pos_pid);
	PID_Clear(&lifting_controller.lift_speed_pid);
	lifting_controller.set_pos = lifting_controller.lift_info.angle;
	lifting_controller.set_speed = 0;
	lifting_controller.send_current = 0;
}

void Lifting_Control(void)
{
	Lifting_State_Check();

	if ((lifting_controller.lift_state == UP || lifting_controller.lift_state == DOWN) && lifting_controller.finished == 0)
	{
		float target_height = (lifting_controller.lift_state == UP) ? lifting_controller.high_state : lifting_controller.low_state;

		if (lifting_controller.lift_state == UP)
		{
			Lifting_Error_Check(target_height);
		}

		if (!lifting_controller.error)
		{
			lifting_controller.lift_speed_pid.Ref = LIFT_DIR * PID_Calculate(&lifting_controller.lift_pos_pid, Tof_ReceiveData.distance, target_height);
			lifting_controller.send_current = PID_Calculate(&lifting_controller.lift_speed_pid, lifting_controller.lift_info.speed, lifting_controller.lift_speed_pid.Ref);
			Lifting_Finish_Check();
		}
		else
		{
			Lifting_Clear();
		}
	}
	else
	{ // power down or finished
		Lifting_Clear();
	}
}
