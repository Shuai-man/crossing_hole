#include "lifting_control.h"

// todo 加入编码器辅助判断位置

LiftingController lifting_controller;

void LiftPidInit(void)
{
	lifting_controller.high_state = GIMBAL_HIGH_STATE;
	lifting_controller.low_state = GIMBAL_LOW_STATE;

	PID_Init(&lifting_controller.lift_pos_pid, LIFTING_SPEED_MAX, 0, 0, 25.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
	PID_Init(&lifting_controller.lift_speed_pid, C610_MAX_CURRENT, 3.0f, 0, 0.015f, 0.0, 0.0f, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate);

	lifting_controller.phase = LIFT_PHASE_IDLE;
	lifting_controller.error = false;
	lifting_controller.finish_cnt = 0;
	/*---------遮挡检测---------*/
	lifting_controller.settle_ticks = 0;
	lifting_controller.last_distance = 0;
	lifting_controller.recovery_ref = 0;
	/*---------堵转检测---------*/
	lifting_controller.stall_counter = 0;
	lifting_controller.angle_reference = 0;
	lifting_controller.tof_reference = 0;
}

/* ---------- 内部辅助函数 ---------- */

static void clear_pid(void)
{
	PID_Clear(&lifting_controller.lift_pos_pid);
	PID_Clear(&lifting_controller.lift_speed_pid);
	lifting_controller.set_pos = lifting_controller.lift_info.angle;
	lifting_controller.set_speed = 0;
	lifting_controller.send_current = 0;
}

static void run_pid(float target_height)
{
	lifting_controller.lift_speed_pid.Ref = LIFT_DIR * PID_Calculate(&lifting_controller.lift_pos_pid,
																																	 Tof_ReceiveData.distance, target_height);
	lifting_controller.send_current = PID_Calculate(&lifting_controller.lift_speed_pid,
																									lifting_controller.lift_info.speed, lifting_controller.lift_speed_pid.Ref);
}

static bool check_finish(void)
{
	if (fabs(lifting_controller.lift_pos_pid.Err) > POSE_THRESHOLD || fabs(lifting_controller.send_current) > CURRENT_THRESHOLD)
	{
		lifting_controller.finish_cnt = 0;
		return false;
	}
	lifting_controller.finish_cnt++;
	return (lifting_controller.finish_cnt >= FINISH_CNT_LIMIT);
}

/* 上升阻塞检测：比较当前帧与上一帧的TOF距离突降 */
static bool detect_blocked(float target_height)
{
	/* 远离目标时才启用阻塞检测 */
	if (fabs(Tof_ReceiveData.distance - target_height) <= OBSTACLE_ERR_THRESHOLD)
	{
		return false;
	}

	/* 距离突降超过阈值 → 阻塞 */
	if (lifting_controller.last_distance > 0 &&
			Tof_ReceiveData.distance + OBSTACLE_DELTA_THRESHOLD < lifting_controller.last_distance)
	{
		return true;
	}
	lifting_controller.last_distance = Tof_ReceiveData.distance;
	return false;
}

static bool tof_online(void)
{
	return (global_debugger.tof_debugger.state == ON);
}

/* 堵转/空转统合检测 */
static bool detect_stall(void)
{
	float target;
	if (lifting_controller.phase == LIFT_PHASE_ASCENDING)
		target = lifting_controller.high_state;
	else
		target = lifting_controller.low_state;

	/* 接近目标时豁免（防止到位保持电流被误判） */
	if (fabs(Tof_ReceiveData.distance - target) <= OBSTACLE_ERR_THRESHOLD)
	{
		lifting_controller.stall_counter = 0;
		lifting_controller.angle_reference = lifting_controller.lift_info.angle;
		lifting_controller.tof_reference = Tof_ReceiveData.distance;
		return false;
	}

	float angle_delta = fabs(lifting_controller.lift_info.angle - lifting_controller.angle_reference);
	float tof_delta = fabs(Tof_ReceiveData.distance - lifting_controller.tof_reference);

	/* TOF有变化 → 机构在移动 → 复位 */
	if (tof_delta >= SPIN_TOF_RANGE)
	{
		lifting_controller.stall_counter = 0;
		lifting_controller.angle_reference = lifting_controller.lift_info.angle;
		lifting_controller.tof_reference = Tof_ReceiveData.distance;
		return false;
	}

	/* --- 堵转检测：电流大 + 角度不动 --- */
	if (angle_delta < STALL_ANGLE_DELTA_THRESHOLD)
	{
		if (fabs(lifting_controller.send_current) >= STALL_CURRENT_THRESHOLD)
		{
			lifting_controller.stall_counter++;
			if (lifting_controller.stall_counter >= STALL_CHECK_CYCLES)
			{
				lifting_controller.stall_counter = 0;
				return true;
			}
		}
		else
		{
			lifting_controller.stall_counter = 0;
		}
		return false;
	}

	/* --- 空转检测：角度大幅变化 + TOF不动 --- */
	if (angle_delta >= SPIN_ANGLE_RANGE)
	{
		lifting_controller.stall_counter++;
		if (lifting_controller.stall_counter >= SPIN_CHECK_CYCLES)
		{
			lifting_controller.stall_counter = 0;
			return true;
		}
		/* fall through — 不return false，防止更新参考点 */
	}

	/* 中间区（角度在STALL~SPIN之间）：不复位counter，等待角度继续积累 */
	return false;
}

/* ---------- 阶段转换 ---------- */

static void enter_phase(LiftPhase new_phase)
{
	lifting_controller.phase = new_phase;
	lifting_controller.error = false;
	lifting_controller.last_distance = 0;
	lifting_controller.recovery_ref = 0;
	lifting_controller.finish_cnt = 0;
	lifting_controller.settle_ticks = 0;
	lifting_controller.stall_counter = 0;
	lifting_controller.angle_reference = lifting_controller.lift_info.angle;
	lifting_controller.tof_reference = Tof_ReceiveData.distance;

	/* 任何升降动作都先要求云台回正 */
	gimbal_controller.target_pitch_angle = 5.0f;
	gimbal_controller.return_flag = 1;
}

/* ---------- 各阶段执行 ---------- */

static void run_ascending(void)
{
	float target = lifting_controller.high_state;

	/* 稳定期：下降→上升切换后等待惯性消退 */
	if (lifting_controller.settle_ticks > 0)
	{
		lifting_controller.settle_ticks--;
		run_pid(target);
		lifting_controller.last_distance = Tof_ReceiveData.distance; // 保护期间不进行阻塞检测
		if (check_finish())
		{
			enter_phase(LIFT_PHASE_COMPLETE);
			clear_pid();
		}
		return;
	}

	/* TOF离线 或 检测到阻塞 → 进入阻塞状态 */
	if (!tof_online() || detect_blocked(target))
	{
		lifting_controller.error = true;
		lifting_controller.recovery_ref = lifting_controller.last_distance;
		lifting_controller.phase = LIFT_PHASE_BLOCKED;
		clear_pid();
		return;
	}

	/* 电机堵转检测 */
	if (detect_stall())
	{
		lifting_controller.error = true;
		lifting_controller.phase = LIFT_PHASE_BLOCKED;
		clear_pid();
		return;
	}

	run_pid(target);
	if (check_finish())
	{
		enter_phase(LIFT_PHASE_COMPLETE);
		clear_pid();
	}
}

static void run_descending(void)
{
	/* 下降前等待云台回正 */
	if (gimbal_controller.return_flag != 0)
	{
		clear_pid();
		return;
	}

	/* 下降过程堵转检测 */
	if (detect_stall())
	{
		lifting_controller.error = true;
		lifting_controller.phase = LIFT_PHASE_BLOCKED;
		clear_pid();
		return;
	}

	float target = lifting_controller.low_state;
	run_pid(target);
	if (check_finish())
	{
		enter_phase(LIFT_PHASE_COMPLETE);
		clear_pid();
	}
}

static void run_blocked(void)
{
	/* TOF离线 → 保持阻塞 */
	if (!tof_online())
	{
		clear_pid();
		return;
	}

	/* 当前距离回到阻塞前位置附近 → 恢复上升 */
	if (lifting_controller.recovery_ref > 0 &&
			Tof_ReceiveData.distance >= (float)lifting_controller.recovery_ref - OBSTACLE_DELTA_THRESHOLD)
	{
		lifting_controller.phase = LIFT_PHASE_ASCENDING;
		lifting_controller.error = false;
		lifting_controller.recovery_ref = 0;
		/* 恢复后再次给予稳定期 */
		lifting_controller.settle_ticks = SETTLE_TICKS;
		return;
	}

	clear_pid();
}

/* ---------- 遥控器需求变化 → 阶段转换决策 ---------- */

static void handle_demand_change(uint8_t demand)
{
	switch (demand)
	{
	case UP:
	{
		/* 从下降切换至上升 → 需要稳定期处理惯性 */
		bool from_descend = (lifting_controller.phase == LIFT_PHASE_DESCENDING);
		enter_phase(LIFT_PHASE_ASCENDING);
		if (from_descend)
		{
			lifting_controller.settle_ticks = SETTLE_TICKS;
		}
		break;
	}
	case DOWN:
		enter_phase(LIFT_PHASE_DESCENDING);
		break;
	case POWER_DOWN:
	default:
		enter_phase(LIFT_PHASE_IDLE);
		clear_pid();
		break;
	}
}

/* ---------- 公开接口 ---------- */

void Lifting_Control(void)
{
	static uint8_t last_demand = POWER_DOWN;
	uint8_t demand = remote_controller.gimbal_position;

	/* 检测遥控器需求变化 */
	if (demand != last_demand)
	{
		last_demand = demand;
		handle_demand_change(demand);
		/* 转换后立即跳过本帧执行，下一帧进入对应 run_* 函数 */
		return;
	}

	/* 按当前阶段执行 */
	switch (lifting_controller.phase)
	{
	case LIFT_PHASE_ASCENDING:
		run_ascending();
		break;
	case LIFT_PHASE_DESCENDING:
		run_descending();
		break;
	case LIFT_PHASE_BLOCKED:
		run_blocked();
		break;
	case LIFT_PHASE_COMPLETE:
	case LIFT_PHASE_IDLE:
	default:
		clear_pid();
		break;
	}
}
