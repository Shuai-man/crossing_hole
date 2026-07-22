#include "mecanum.h"
#include "mecanum.h"
#include "ChassisController.h"

#include <math.h>

void mecanum_pid_init()
{
    // 转向PID初始化
    PID_Init(&infantry.turn_pos_pid, 20, 0, 0.0, 0.3, 0, 0.0, 0, 0, 0.000, 0.000, 1, NONE);

    // 轮子控制PID
    PID_Init(&infantry.wheels_pid[LEFT_UP_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 10.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[RIGHT_UP_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 10.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[LEFT_DOWN_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 10.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[RIGHT_DOWN_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 10.0f, 0, 0, 0, 0, 0, 0, 1, NONE);
}

// 正运动学
void mecanum_pos_kinematics()
{
    // 计算正运动学，并转到m/s为单位
    float speed1 = infantry.sensors_info.wheels_decode[LEFT_UP_MECANUM_WHEEL].speed;
    float speed2 = infantry.sensors_info.wheels_decode[RIGHT_UP_MECANUM_WHEEL].speed;
    float speed3 = infantry.sensors_info.wheels_decode[LEFT_DOWN_MECANUM_WHEEL].speed;
    float speed4 = infantry.sensors_info.wheels_decode[RIGHT_DOWN_MECANUM_WHEEL].speed;

    // 以向前(y)，向右(x)，逆时针(yaw)
    infantry.y_v = (speed1 - speed2 + speed3 - speed4) / 4 * MECANUM_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    infantry.x_v = (speed1 + speed2 - speed3 - speed4) / 4 * MECANUM_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    infantry.yaw_v = (-speed1 - speed2 - speed3 - speed4) / 4 / (MECANUM_WIDTH + MECANUM_LENGTH) * MECANUM_DEGEREE_S_TO_MS; // rad/s
}

// 逆运动学
void mecanum_inv_kinematics()
{
    // 逆运动学解算,统一到度/s的单位
    infantry.wheels_set_v[LEFT_UP_MECANUM_WHEEL] = (infantry.set_x_v * SQRT_2 + infantry.set_y_v * SQRT_2 - infantry.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[RIGHT_UP_MECANUM_WHEEL] = (infantry.set_x_v * SQRT_2 - infantry.set_y_v * SQRT_2 - infantry.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[LEFT_DOWN_MECANUM_WHEEL] = (-infantry.set_x_v * SQRT_2 + infantry.set_y_v * SQRT_2 - infantry.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    infantry.wheels_set_v[RIGHT_DOWN_MECANUM_WHEEL] = (-infantry.set_x_v * SQRT_2 - infantry.set_y_v * SQRT_2 - infantry.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
}

// 底盘跟随策略
void mecanum_follow_control()
{
    float follow_angle_feedback;
    float abs_error_angle;
    const float follow_deadband = 0.2f;
    const float follow_soft_zone = 1.0f;

    // 正运动学解算
    mecanum_pos_kinematics();

    // 速度分解
    infantry.set_x_v = infantry.target_x_v * infantry.cos_dir - infantry.target_y_v * infantry.sin_dir;
    infantry.set_y_v = infantry.target_y_v * infantry.cos_dir + infantry.target_x_v * infantry.sin_dir;

    // 转向计算：云台目标角速度前馈 + 底盘/云台夹角反馈
    follow_angle_feedback = PID_Calculate(&infantry.turn_pos_pid, infantry.error_angle, 0);
    abs_error_angle = fabsf(infantry.error_angle);
    if (abs_error_angle < follow_deadband)
    {
        follow_angle_feedback = 0.0f;
    }
    else if (abs_error_angle < follow_soft_zone)
    {
        follow_angle_feedback *=
            (abs_error_angle - follow_deadband) / (follow_soft_zone - follow_deadband);
    }

    infantry.target_pid_yaw_v = GIMBAL_MOTOR_SIGN * follow_angle_feedback;
    infantry.set_yaw_v = LIMIT_MAX_MIN(infantry.target_pid_yaw_v + infantry.follow_yaw_v,
                                       infantry.speed_yaw_max,
                                       -infantry.speed_yaw_max);
    

    // 逆运动学解算
    mecanum_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }
}

// 底盘跟随策略,加上小陀螺
void mecanum_rotate_control()
{
    // 正运动学解算
    mecanum_pos_kinematics();

    // 速度分解
    infantry.set_x_v = infantry.target_x_v * infantry.cos_dir - infantry.target_y_v * infantry.sin_dir;
    infantry.set_y_v = infantry.target_y_v * infantry.cos_dir + infantry.target_x_v * infantry.sin_dir;
    infantry.set_yaw_v = infantry.target_yaw_v;

    // 逆运动学解算
    mecanum_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }
}

// 仅底盘运动策略，底盘将按照规定正方向运动，通常用于测试或者小陀螺状态
void mecanum_chassis_control()
{
    // 正运动学解算
    mecanum_pos_kinematics();

    infantry.set_x_v = infantry.target_x_v * infantry.cos_dir - infantry.target_y_v * infantry.sin_dir;
    infantry.set_y_v = infantry.target_y_v * infantry.cos_dir + infantry.target_x_v * infantry.sin_dir;
    infantry.set_yaw_v = 0;

    // 逆运动学解算
    mecanum_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.wheels_set_v[i]);
    }
}
