#include "FrictionWheel.h"

FrictionWheel_t friction_wheels; // 两个摩擦轮

void FrictionWheel_Init()
{
	// 超调会超弹速
	PID_Init(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL],
					 C620_MAX_SEND_CURRENT, 7000.0f, 0.0, 2.5, 0.1, 0.02, 1000, 1000, 0, 0, 1, ChangingIntegrationRate);
	PID_Init(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL],
					 C620_MAX_SEND_CURRENT, 7000.0f, 0.0, 2.5, 0.1, 0.02, 1000, 1000, 0, 0, 1, ChangingIntegrationRate);
	// 实测弹速会超0.5左右，但是基本不低于设定弹速
	friction_wheels.friction_speed = 23.78f; // 初始弹速设置
}

// 自动切弹速
void setFrictionSpeed(void)
{ // 如果没更新，速度会设为0

	// 电池没电时，弹速会掉，需要稍微提高设定转速
	// 长时间连发，弹速会掉，这时不能提高转速，否则恢复后会超
	// 漏弹也有速度回传，不能用来检测，否则弹速会超
	// 弹速低于20视为漏弹
	// 不要设置在25以内，一般测好的速度不会超，连发后弹速偶尔漂到25
	// 拨盘转速会影响弹速
	// 超速处理
	friction_wheels.shoot_speed = chassis_pack_get_1.shoot_speed;

	if (friction_wheels.shoot_speed != 0 && friction_wheels.last_shoot_Speed == 0) // 上升沿检测弹速更新
	{
		if (friction_wheels.shoot_speed > 25.0f)
		{
			friction_wheels.over_times += 1;
			friction_wheels.over_speed = friction_wheels.shoot_speed;
			friction_wheels.friction_speed -= (friction_wheels.over_speed - 25.0f);
		}
		// 掉速处理
		if (chassis_pack_get_1.shoot_speed < 23.0f && chassis_pack_get_1.shoot_speed > 20.0f) // 排除漏弹
		{
			friction_wheels.lower_speed_times += 1;
		}
		else
		{ // 偶尔掉速，不处理
			friction_wheels.lower_speed_times = 0;
		}
		if (friction_wheels.lower_speed_times >= 3)
		{
			// 多次低于，调整弹速
			friction_wheels.lower_speed = friction_wheels.shoot_speed;
			friction_wheels.friction_speed += (23.5f - friction_wheels.lower_speed);
			friction_wheels.lower_speed_times = 0;
		}
	}

	friction_wheels.last_shoot_Speed = friction_wheels.shoot_speed;

	if (friction_wheels.friction_speed > 24.5f)
		friction_wheels.friction_speed = 24.3f; // 超过这个值一定超弹速
	if (friction_wheels.friction_speed < 22.0f)
		friction_wheels.friction_speed = 22.0f;

	friction_wheels.set_speed = friction_wheels.friction_speed * SPEED_RATIO; // 不再限制弹速
}

void FrictionWheel_Set(float speed) // 度/s
{
	friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] =
			PID_Calculate(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL],
										friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed, speed);

	friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] =
			PID_Calculate(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL],
										friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed, -speed);
}
