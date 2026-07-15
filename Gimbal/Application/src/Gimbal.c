#include "Gimbal.h"
#include "bsp_dwt.h"

GimbalController gimbal_controller;

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
    PID_Init(&gimbal_controller.pitch_angle_pid, 5000.0f, 0, 0.0f, 800.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
#if GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    /* Pitch 系统辨识专用速度环参数，适用于重力、B/C 和 J 三个阶段。 */
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000.0f, 4000.0f, 0.0f, 60.0f, 0.0f, 0.15f, 0, 0, 0.0018f, 0, 1, Integral_Limit);
#else
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000.0f, 4000.0f, 0.0f, 50.0f, 0.0f, 0, 0, 0, 0.0018f, 0, 1, Integral_Limit);
#endif
    // td结构体: r:加速度因子, h0:滤波系数，单位s
    TD_Init(&gimbal_controller.pos_pitch_td, 1000, 0.005);

    // Yaw
    PID_Init(&gimbal_controller.yaw_angle_pid, 5000.0, 0, 0, 2000.0f, 0, 0.0f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.yaw_speed_pid, 5000, 1600, 0.0, 100.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
    // td结构体: r:越大，突变越大，离原始信号越接近； h0:滤波系数，单位s，h0越大延迟越大
    // r增大，可以增加加速度项，从而加大前馈的输出值
    TD_Init(&gimbal_controller.pos_yaw_td, 2000, 0.005);

    // 底盘转向前馈
    float ff_c_follow[3] = {0, 0.01, 0};
    Feedforward_Init(&gimbal_controller.follow_gimbal_forward, 1.0f, ff_c_follow, 0.05, 1, 1);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
// todo 把重力补偿加上
float Gimbal_Pitch_Calculate(float set_point)
{
#if GIMBAL_SYSID ==GIMBAL_PITCH_SYSID
    PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.pitch_speed_pid.Ref);
    gimbal_controller.pitch_out = gimbal_controller.pitch_speed_pid.Output;
    return gimbal_controller.pitch_out;
#else
    float friction_ratio;

    TD_Calculate(&gimbal_controller.pos_pitch_td, set_point);
    PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.pos_pitch_td.x);
    PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.pos_pitch_td.dx);
    gimbal_controller.gravity_comp = GIMBAL_PITCH_SIN * sin(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) +
                                     GIMBAL_PITCH_COS * cos(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF);

    /*
     * C 是运动时的库仑摩擦，不是静摩擦。使用轨迹参考速度决定方向，并在低速区
     * 线性淡入，避免实际速度在零点附近抖动时前馈来回跳变。静止时该项为 0，
     * 剩余静摩擦和模型误差由闭环（尤其积分项）处理。
     */
    friction_ratio = LIMIT_MAX_MIN(gimbal_controller.pos_pitch_td.dx / BORDER_FRICTION_SPEED,
                                   1.0f, -1.0f);
    gimbal_controller.ff_tff_pitch = GIMBAL_PITCH_J * gimbal_controller.pos_pitch_td.ddx + // 惯量 × 加速度，主要输出项
                                     GIMBAL_PITCH_B * gimbal_controller.pos_pitch_td.dx + // 粘性阻尼 × 速度
                                     GIMBAL_PITCH_C * friction_ratio;                       // 运动库仑摩擦

    gimbal_controller.pitch_out = gimbal_controller.ff_tff_pitch + gimbal_controller.gravity_comp + gimbal_controller.pitch_angle_pid.Output + gimbal_controller.pitch_speed_pid.Output;
    return gimbal_controller.pitch_out;
#endif
}

/*MPC 轨迹:  θ_target(t), ω_target(t), α_target(t)
  │
  ├── 前馈路径（物理模型，开环）:
  │   I_ff = (J·α_target + B·ω_target + C·sign(ω_target) + G(θ_target)) / K_t
  │
  ├── 反馈路径（PID，闭环）:
  │   I_fb = (Kp·e_pos + Kd·e_vel + Ki·∫e_pos) / K_t
  │
  └── 总电流指令:
      I_cmd = I_ff + I_fb  →  电流环（FOC / 电机驱动）  →  电机出力
*/
float Gimbal_Yaw_Calculate(float set_point)
{
    // 由前馈电流负责控制，pid只负责闭环修正位置
    // 高速情况下纯pid控制有明显滞后问题，必须加前馈
#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.yaw_speed_pid.Ref);
    gimbal_controller.yaw_out = gimbal_controller.yaw_speed_pid.Output;
    return gimbal_controller.yaw_out;
#else
    float friction_ratio;

    // 输出滤波的角度，角速度，角加速度
    TD_Calculate(&gimbal_controller.pos_yaw_td, set_point);
    /*
     * C 表示运动时的库仑摩擦。使用轨迹参考速度决定方向，并在低速区
     * 线性淡入；参考速度为0时摩擦前馈也为0，避免 sign(0)=+1 造成
     * 固定的正向力矩和位置稳态偏差。
     */
    friction_ratio = LIMIT_MAX_MIN(gimbal_controller.pos_yaw_td.dx /
                                   BORDER_FRICTION_SPEED,
                                   1.0f, -1.0f);
    // 计算前馈力矩
    gimbal_controller.ff_tff =
        GIMBAL_YAW_J * gimbal_controller.pos_yaw_td.ddx +     // 惯量 × 加速度，主要输出项
        GIMBAL_YAW_B * gimbal_controller.pos_yaw_td.dx +      // 阻尼 × 速度
        GIMBAL_YAW_C * friction_ratio;                         // 低速平滑的运动库仑摩擦
    PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, gimbal_controller.pos_yaw_td.x);
    PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.pos_yaw_td.dx); // 速度环类似阻尼项，因为前馈会拉着电机加速，所以实际速度比设定速度快，一开始速度环输出会是负的，如果影响大，可以适当减小速度环的p
    // 总输出 = 前馈 + 角度环输出 + 速度环输出
    gimbal_controller.yaw_out = gimbal_controller.ff_tff + gimbal_controller.yaw_angle_pid.Output + gimbal_controller.yaw_speed_pid.Output;
    return gimbal_controller.yaw_out;
#endif
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

    TD_Clear(&gimbal_controller.pos_pitch_td, gimbal_controller.gyro_pitch_angle);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.pitch_out = 0;
    gimbal_controller.gravity_comp = 0;
    gimbal_controller.ff_tff_pitch = 0;

    // yaw
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    PID_Clear(&gimbal_controller.yaw_speed_pid);

    TD_Clear(&gimbal_controller.pos_yaw_td, gimbal_controller.gyro_yaw_angle);
    gimbal_controller.ff_tff = 0;

    gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle;
    gimbal_controller.yaw_out = 0;
}

/**
 * @brief 限制设置的pitch角度大小
 */
void limitPitchAngle()
{
    gimbal_controller.chassis_err_angle = GIMBAL_PITCH_MOTOR_SIGN * (GIMBAL_PITCH_ZERO - gimbal_controller.DM_Pitch_Motor.P_angle);
    gimbal_controller.chassis_pitch_angle = gimbal_controller.gyro_pitch_angle + gimbal_controller.chassis_err_angle;
    if (remote_controller.gimbal_position == DOWN)
    {
        gimbal_controller.pitch_max_angle = GIMBAL_ANGLE_MAX + gimbal_controller.chassis_pitch_angle;
        gimbal_controller.pitch_min_angle = gimbal_controller.chassis_pitch_angle;
    }
    else
    {
        gimbal_controller.pitch_max_angle = GIMBAL_ANGLE_MAX;
        gimbal_controller.pitch_min_angle = GIMBAL_ANGLE_MIN;
    }
    gimbal_controller.target_pitch_angle = LIMIT_MAX_MIN(gimbal_controller.target_pitch_angle, gimbal_controller.pitch_max_angle, gimbal_controller.pitch_min_angle);
}

void Gimbal_ErrorAngle(void)
{
    // limit_angle可以归一化180度，不需要再判断最小回正角度
    gimbal_controller.err_angle = limit_angle(gimbal_controller.DM_Yaw_Motor.P_angle - GIMBAL_ANGLE_ZERO);
    gimbal_controller.err_angle_180 = limit_angle(gimbal_controller.err_angle + 180.0f);

    // 判断方向
    if (fabsf(gimbal_controller.err_angle) < fabsf(gimbal_controller.err_angle_180))
    {
        gimbal_controller.gimbal_direction = GIMBAL_FRONT;
    }
    else
    {
        gimbal_controller.gimbal_direction = GIMBAL_BACK;
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
    gimbal_controller.gyro_pitch_angle = GIMBAL_PITCH_GYRO_SIGN * INS.Pitch;
    // pitch角速度
    speed = GIMBAL_PITCH_GYRO_SIGN * INS.Gyro[0] / PI * 180.0f;
    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.2);

    // yaw角度
    gimbal_controller.gyro_yaw_angle = GIMBAL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    // yaw角速度
    speed = GIMBAL_YAW_GYRO_SIGN * INS.Gyro[2] / PI * 180.0f;
    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.6); // 还是超调可以试试加大滤波
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
