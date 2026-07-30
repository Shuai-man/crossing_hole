#include "ChassisController.h"

#include <float.h>
#include <math.h>

#include "NingCap.h"
#include "PowerControlTask.h"
#include "Gimbalreceive.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "can_config.h"
#include "debug.h"
#include "mecanum.h"
#include "robot_config.h"
#include "remote_control.h"
#include "tools.h"

Infantry infantry;

static uint8_t float_is_finite(float value)
{
    return (uint8_t)((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX));
}

// todo 有了功控其实很难超功率，所以应该把speedmax改成设定目标速度，通过调度超电，尽可能达到设定速度
// 根据目标速度，更改设定功率
void InfantryInit(Infantry *infantry)
{
    // 功率控制初始化
    PowerLimitInit(&infantry->power_limiter, 4, M3508, infantry->power_limit_method);
// 最大速度初始化
    MaxSpeed_Init(&infantry->speed_x_fit, 45.0f, 0.7f, 120.0f, 2.0f);
    MaxSpeed_Init(&infantry->speed_y_fit, 45.0f, 0.7f, 120.0f, 2.0f);
    MaxSpeed_Init(&infantry->speed_yaw_fit, 45.0f, 5.0f, 120.0f, 12.0f);
}

/**
 * @brief  转角限制到±180度
 * @param  输入转角
 * @retval 输出转角
 */
float limit_angle(float in)
{
    float angle = fmodf(in, 360.0f);
    if (angle < -180.0f)
        angle = angle + 2 * 180.0f;
    if (angle > 180.0f)
        angle = angle - 2 * 180.0f;
    return angle;
}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */
uint8_t dir = 0;
float angle_z_err_get(float target_ang, float zeros_angle)
{
    // 输入为电机机械角度
    float AngErr_front, AngErr_back, AngErr_left, AngErr_right, minAngle, angleBias = 0.0f;

    // 不同电机的计算不一样，但要保证最终的输出error_angle定义一致
    if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45) // 增加45度计算夹角
    {
        angleBias = 45.0f;
    }

    AngErr_front = limit_angle((zeros_angle - target_ang) + angleBias);
    AngErr_back = limit_angle(AngErr_front + 180.0f);
    AngErr_left = limit_angle(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
    AngErr_right = limit_angle(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);

    // 判断跟随
    if (infantry.chassis_follow_type == TWO_SIDES_FOLLOW)
    {
        if (fabs(AngErr_front) > fabs(AngErr_back))
        {
            infantry.chassis_direction = CHASSIS_BACK;
            return AngErr_back;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_FRONT;
            return AngErr_front;
        }
    }
    else if (infantry.chassis_follow_type == TWO_SIDES_LEFT_RIGHT)
    {
        if (fabs(AngErr_left) > fabs(AngErr_right))
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
            return AngErr_right;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_LEFT;
            return AngErr_left;
        }
    }
    else if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW || infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45)
    {
        minAngle = MIN(fabs(AngErr_front), MIN(fabs(AngErr_back), MIN(fabs(AngErr_left), fabs(AngErr_right))));
        if (fabs(fabs(AngErr_front) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_FRONT;
            dir = 1;
            return AngErr_front;
        }
        else if (fabs(fabs(AngErr_back) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_BACK;
            dir = 2;
            return AngErr_back;
        }
        else if (fabs(fabs(AngErr_left) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_LEFT;
            dir = 3;
            return AngErr_left;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
            dir = 4;
            return AngErr_right;
        }
    }
    return 0;
}

// 获取控制方向
void getDir(void)
{
    float AngleErr_front = limit_angle(GIMBAL_MOTOR_SIGN * (infantry.yaw_angle - GIMBAL_FOLLOW_ZERO));
    arm_sin_cos_f32(AngleErr_front, &infantry.sin_dir, &infantry.cos_dir);
}

void get_sensors_info(Sensors *sensors_info)
{
    // 麦轮和全向轮无舵向电机
    if (infantry.chassis_type == MECANUM_WHEEL)
    {
        for (int i = 0; i < 4; i++)
        {
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }
    // 用于底盘跟随的error_angle
    infantry.error_angle = angle_z_err_get(infantry.yaw_angle, GIMBAL_FOLLOW_ZERO);
    // 用于速度解算的angle
    getDir();
}

void MaxSpeed_Init(ChassisMaxSpeedFit *max_speed_fit, float power_low, float speed_low, float power_high, float speed_high)
{
    max_speed_fit->power_low = power_low;
    max_speed_fit->speed_low = speed_low;
    max_speed_fit->power_high = power_high;
    max_speed_fit->speed_high = speed_high;
    max_speed_fit->k = (speed_high - speed_low) / (power_high - power_low);
    max_speed_fit->b = speed_low - max_speed_fit->k * power_low;
}

float ChassisMaxSpeed(ChassisMaxSpeedFit *max_speed_fit, float set_power)
{
    max_speed_fit->speed_max = max_speed_fit->k * set_power + max_speed_fit->b;
    return max_speed_fit->speed_max;
}

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry)
{
    const uint8_t super_cap_available =
        (uint8_t)(global_debugger.super_power_debugger.state == ON &&
                  NingCapHasEnergy() != 0U);
    const uint8_t super_cap_active =
        (uint8_t)(super_cap_available != 0U &&
                  remote_controller.super_power_state ==
                      POWER_TO_SuperPower);

    infantry->set_power = GetChassisPowerLimit(super_cap_active);

    // 无超电时，小陀螺，平移，缓冲能量消耗速度=恢复速度
    //  节能模式35w时，速度也不会出现负数
    //  地胶地形
    if (infantry->chassis_type == MECANUM_WHEEL)
    {
        infantry->speed_x_max = ChassisMaxSpeed(&infantry->speed_x_fit, infantry->set_power);
        infantry->speed_y_max = ChassisMaxSpeed(&infantry->speed_y_fit, infantry->set_power);
        infantry->speed_yaw_max = ChassisMaxSpeed(&infantry->speed_yaw_fit, infantry->set_power);

        if (gimbal_receiver_pack1.chassis_mode_action == CV_ROTATE)
        {
            {
                float yaw_ratio;
                float translate_scale;

                yaw_ratio = fabsf(infantry->target_yaw_v_percent);
                yaw_ratio = LIMIT_MAX_MIN(yaw_ratio, 1.0f, 0.0f);

                translate_scale = 1.0f - CHASSIS_ROTATE_YAW_WEIGHT * yaw_ratio;
                translate_scale = LIMIT_MAX_MIN(translate_scale,
                                                1.0f,
                                                CHASSIS_ROTATE_TRANSLATE_MIN_SCALE);

                infantry->speed_x_max *= translate_scale;
                infantry->speed_y_max *= translate_scale;
            }
        }
    }
}

// 速度百分比由云台侧处理，底盘侧只做比例换算
void wheels_accel(Infantry *infantry)
{
    infantry->target_x_v = infantry->target_x_v_percent * infantry->speed_x_max;
    infantry->target_y_v = infantry->target_y_v_percent * infantry->speed_y_max;
    infantry->target_yaw_v = infantry->target_yaw_v_percent * infantry->speed_yaw_max;
}

void wheels_power_limit(Infantry *infantry)
{
    // 功率控制
    // limiter赋值
    float w_error = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        infantry->power_limiter.motor_w[i] = infantry->sensors_info.wheels_decode_raw[i].speed * ANGLE_TO_RAD_COEF;

        // 设置为电机反馈可作拟合
        infantry->power_limiter.I_collect[i] = infantry->sensors_info.wheels_decode_raw[i].torque_current;

        // 设置为发送电流可做削减
        infantry->power_limiter.motor_I[i] = infantry->excute_info.wheels_set_current[i] / C620_CURRENT_SEND_TRANS;

        // 设置实际轮子转速与设定转速差
        w_error = fabs(infantry->wheels_set_v[i] - infantry->sensors_info.wheels_decode[i].speed) * ANGLE_TO_RAD_COEF;
        infantry->power_limiter.motor_w_error[i] = fabs(w_error);
        infantry->power_limiter.motor_online[i] =
            (uint8_t)(global_debugger.wheels_comm_debugger[i].state == ON);
    }

    // 限制功率

    PowerLimit(&infantry->power_limiter, infantry->set_power);

    /*
     * 影子 RLS 只识别 R/B 并输出调试量，不修改当前功控参数。
     * 超电掉线时由已有 state 判断停止更新。
     */
    PowerModelRLSUpdate(
        &infantry->power_limiter,
        cap_controller.chassis_power,
        cap_controller.power_measurement_sequence,
        (uint8_t)(global_debugger.super_power_debugger.state == ON &&
                  cap_controller.power_data_valid != 0U));

    // 作功率削减
    for (int i = 0; i < 4; i++)
    {
        if (float_is_finite(infantry->excute_info.wheels_set_current[i]) == 0U ||
            infantry->power_limiter.send_torque_lower_scale[i] <= 0.0f)
        {
            infantry->excute_info.wheels_set_current[i] = 0.0f;
        }
        else
        {
            infantry->excute_info.wheels_set_current[i] *=
                infantry->power_limiter.send_torque_lower_scale[i];
        }
        infantry->wheels_send_current[i] = infantry->excute_info.wheels_set_current[i];
    }
}

void chassis_powerdown_control(Infantry *infantry)
{
    infantry->set_x_v = 0;
    infantry->set_y_v = 0;
    infantry->set_yaw_v = 0;
    infantry->target_x_v = 0;
    infantry->target_y_v = 0;
    infantry->target_yaw_v = 0;
    infantry->follow_yaw_v = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        infantry->excute_info.steers_set_current[i] = 0;
        infantry->excute_info.wheels_set_current[i] = 0;
    }
}

void chassis_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_follow_control();
        break;
    default:
        break;
    }
}

void chassis_not_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_chassis_control();
        break;
    default:
        break;
    }
}

void chassis_rotate_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_rotate_control();
        break;
    default:
        break;
    }
}

void main_control(Infantry *infantry)
{
    switch (remote_controller.control_mode_action)
    {
    case FOLLOW_GIMBAL:
        chassis_follow_control(infantry);
        break;
    case NOT_FOLLOW_GIMBAL:
        chassis_not_follow_control(infantry);
        break;
    case CV_ROTATE:
        chassis_rotate_control(infantry);
        break;

    default:
        chassis_powerdown_control(infantry);
        break;
    }
}

void execute_control(ExcuteTorque *torque) // 控制信息发送
{
    if (infantry.chassis_type == MECANUM_WHEEL)
    {
        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
}
