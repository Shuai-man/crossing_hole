#include "Gimbal.h"

GimbalController gimbal_controller;
float yaw_v = 0;
float yaw = 0;
float pitch = 0;

// todo 过洞时，云台最低角度与机身平行：读取电机角

void GimbalMotorInit(void)
{
    DM_Motor_Init(&gimbal_controller.DM_Pitch_Motor, P_MAX, 10, V_MAX);
    //	DM_Motor_Init(&gimbal_controller.DM_Yaw_Motor,P_MAX,T_MAX,V_MAX);
    DM_Motor_Init(&gimbal_controller.DM_Yaw_Motor, P_MAX, 10, V_MAX); // 为了方便调参，放大最大值;pid输出限幅要对应修改
}

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit(void)
{

    // Pitch
    PID_Init(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.0f, 20.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 40.0f, 100.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);

    // Yaw
    PID_Init(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 10.0f, 0, 0.0f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.yaw_speed_pid, 9000, 1600, 0.0, 120.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // 跟踪微分器
    TD_Init(&gimbal_controller.pos_yaw_td, 9000, 0.01);
    TD_Init(&gimbal_controller.speed_yaw_td, 90000, 0.01);

    // YAW前馈
    float ff_c_yaw[3] = {0, 1, 0}; // 使用目标角度的微分项，给速度环提供前馈目标速度；给角度输入给角度环和没加的效果是一样的
    // float ff_c_yaw_v[3] = {0, 0, 0};
    Feedforward_Init(&gimbal_controller.yaw_angle_forward, 7000, ff_c_yaw, 0.05, 1, 1);
    // Feedforward_Init(&gimbal_controller.yaw_speed_forward, 7000, ff_c_yaw_v, 0.05, 1, 1);
}

void GimbalPidChange(void)
{

    // Pitch
    PID_Change(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.0f, 12.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Change(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 10.0f, 400.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);

    // Yaw
    PID_Change(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 20.0f, 0, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Change(&gimbal_controller.yaw_speed_pid, 7000, 1600, 0.0, 90.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
}

void Auto_GimbalPidChange(void)
{
    // 打车需要锯齿波跟踪  /|/|/|

    // Pitch
    PID_Change(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.0f, 12.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Change(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 10.0f, 400.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);

    // Yaw
    PID_Change(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 20.0f, 0, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Change(&gimbal_controller.yaw_speed_pid, 7000, 1600, 0.0, 90.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
}

void Small_Buff_Change(void)
{

    // Pitch
    PID_Change(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.0f, 12.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Change(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 10.0f, 400.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);

    // Yaw
    PID_Change(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 20.0f, 0, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Change(&gimbal_controller.yaw_speed_pid, 7000, 1600, 0.0, 90.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
float Gimbal_Pitch_Calculate(float set_point)
{
    PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.target_pitch_angle);
    gimbal_controller.pitch_out = PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.pitch_angle_pid.Output);
    return gimbal_controller.pitch_out;
}

float Gimbal_Yaw_Calculate(float set_point)
{
    // 电机加速度不够，即使给了前馈，还是跟不上，所以现象和大pid控制差不多
    // 可以把角度环的输出减小，留一部分给前馈，可以防止超调
    // 超调其实是因为电机加速度不够，无法修正导致，直接减小输出即可

    // 前馈
    // 角度环前馈提供瞬时响应，速度环前馈抑制突变，作为阻尼项
    PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, set_point);
    Feedforward_Calculate(&gimbal_controller.yaw_angle_forward, set_point);
    gimbal_controller.yaw_speed_pid.Ref = gimbal_controller.yaw_angle_pid.Output + gimbal_controller.yaw_angle_forward.Output;
    gimbal_controller.yaw_out = PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.yaw_speed_pid.Ref);

    return gimbal_controller.yaw_out;
}
// 限制角度在[-180,180]范围内
float limit_angle(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (in < -180.0f)
            in = in + 2 * 180.0f;
        if (in > 180.0f)
            in = in - 2 * 180.0f;
    }
    return in;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);

    Feedforward_Clear(&gimbal_controller.pitch_speed_forward);
    Feedforward_Clear(&gimbal_controller.pitch_angle_forward);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.pitch_out = 0;
    gimbal_controller.comp_pitch_current = 0;

    // yaw
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    PID_Clear(&gimbal_controller.yaw_speed_pid);
    PID_Clear(&gimbal_controller.yaw_current_pid);

    Feedforward_Clear(&gimbal_controller.yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.yaw_angle_forward);

    TD_Clear(&gimbal_controller.pos_yaw_td, gimbal_controller.gyro_yaw_angle);

    gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.yaw_angle_forward.Ref = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.yaw_out = 0;
}

/**
 * @brief 限制设置的pitch角度大小
 */
void limitPitchAngle()
{
    float arr_angle = 0;
    if (remote_controller.gimbal_position == DOWN)
    {
        arr_angle = GIMBAL_PITCH_MOTOR_SIGN * (gimbal_controller.DM_Pitch_Motor.P_angle - GIMBAL_PITCH_ZERO);
        gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, GIMBAL_ANGLE_MAX - arr_angle, 8.0f - arr_angle);
    }
    else
    {
        gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, GIMBAL_ANGLE_MAX, GIMBAL_ANGLE_MIN);
    }
}

void Gimbal_Get_Dir(float target_angle, float zero_angle)
{
    float AngErr_front, AngErr_back;
    AngErr_front = limit_angle(target_angle - zero_angle);
    AngErr_back = limit_angle(target_angle - zero_angle + 180.0f);
    if (fabsf(AngErr_front) < fabsf(AngErr_back))
    {
        gimbal_controller.gimbal_direction = GIMBAL_FRONT;
    }
    else
    {
        gimbal_controller.gimbal_direction = GIMBAL_BACK;
    }
}

// 云台YAW回正
void Gimbal_Return(void)
{
    float angle = (gimbal_controller.DM_Yaw_Motor.P_angle - GIMBAL_ANGLE_ZERO);
    if (fabs(angle) > 0.5f )
    {
        float AngErr_front, AngErr_back = 0;
        AngErr_front = limit_angle(angle);
        AngErr_back = limit_angle(angle + 360.0f);
        if (fabsf(AngErr_front) < fabsf(AngErr_back))
        {
            gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + AngErr_front;
        }
        else
        {
            gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + AngErr_back;
        }
    }
}

/**
 * @brief 更新pitch角速度，以及角度(注意需要标定零点)
 */

void updateGyro()
{
    float speed = 0;
    // 对Pitch和roll进行交换（由于IMU安装问题）
    // pitch角度
    gimbal_controller.gyro_pitch_angle = GIMBAL_PITCH_GYRO_SIGN * INS.Roll;
    // pitch角速度
    speed = GIMBAL_PITCH_GYRO_SIGN * INS.Gyro[1] / PI * 180.0f;
    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.3);

    // yaw角度
    gimbal_controller.gyro_yaw_angle = GIMBAL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    // yaw角速度
    speed = GIMBAL_YAW_GYRO_SIGN * INS.Gyro[2] / PI * 180.0f;
    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.3);
}

/**
 * @brief 由于重力补偿的作用，云台需要施加一个非线性力抵消重力影响，该力需要根据实际来进行测定
 */
float GimbalPitchComp()
{
    // //记得每调一台车都需要重新更新参数
    // const static float pitch_comp[5] = {0.1399, -0.9144, -10.2, 9.038, -3337};
    // float x[4];

    // //解析静止时的非线性函数，只能大致补偿，然后靠PID的I使最终无静差
    // //低于一定角度或高于一定角度，根据测量结果，输出应大致不变
    // x[3] = LIMIT_MAX_MIN(gimbal_controller.gyro_pitch_angle, 8, -12);
    // x[2] = x[3] * x[3];
    // x[1] = x[2] * x[3];
    // x[0] = x[1] * x[3];

    // float sum = pitch_comp[4];
    // for (int i = 0; i < 4; i++)
    // {
    //     sum += x[i] * pitch_comp[i];
    // }
    // iir(&gimbal_controller.comp_pitch_current, sum * GIMBAL_PITCH_COMP_COEF, 0.7);
    // return gimbal_controller.comp_pitch_current;
    iir(&gimbal_controller.comp_pitch_current, GIMBAL_PITCH_COMP * arm_cos_f32(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) * GIMBAL_PITCH_COMP_COEF, 0.7);
    return gimbal_controller.comp_pitch_current;
}

/**
 * @brief 云台摩擦力模型，只使用库伦摩擦力，因粘性摩擦力在辨识中表现不明显，故忽略
 * @param[in] void
 */
float GimbalFrictionModel()
{
    // 根据转速判断符号
    if (fabsf(gimbal_controller.gyro_yaw_speed) < BORDER_FRICTION_SPEED)
    {
        // 部分补偿
        return gimbal_controller.gyro_yaw_speed / BORDER_FRICTION_SPEED * FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF;
    }
    // 全补偿
    return FRICTION_CURRENT_COMP * FRICTION_FORWARD_COEF * sign(gimbal_controller.gyro_yaw_speed);
}
