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
    PID_Init(&gimbal_controller.pitch_angle_pid, 5000.0f, 0, 0.0f, 600.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000.0f, 4000.0f, 0.0f, 60.0f, 0.0f, 0, 0, 0, 0.0018f, 0, 1, Integral_Limit);
    // td结构体: r:加速度因子, h0:滤波系数，单位s
    TD_Init(&gimbal_controller.pos_pitch_td, 800, 0.005);

    // Yaw
    PID_Init(&gimbal_controller.yaw_angle_pid, 5000.0, 0, 0, 1000.0f, 0, 0.0f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.yaw_speed_pid, 5000, 1600, 0.0, 100.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
    // td结构体: r:越大，突变越大，离原始信号越接近； h0:滤波系数，单位s，h0越大延迟越大
    // r增大，可以增加加速度项，从而加大前馈的输出值
    TD_Init(&gimbal_controller.pos_yaw_td, 700, 0.005);

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
#if GIMBAL_PITCH_SYSID
    gimbal_controller.pitch_out = set_point * gimbal_controller.pitch_angle_pid.Kp;
    return gimbal_controller.pitch_out;

#else
    TD_Calculate(&gimbal_controller.pos_pitch_td, set_point);
    PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.pos_pitch_td.x);
    PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.pos_pitch_td.dx);
    gimbal_controller.gravity_comp = GIMBAL_PITCH_A * sin(gimbal_controller.pos_pitch_td.x * ANGLE_TO_RAD_COEF) +
                                     GIMBAL_PITCH_B * cos(gimbal_controller.pos_pitch_td.x * ANGLE_TO_RAD_COEF) +
                                     GIMBAL_PITCH_C * sign(gimbal_controller.pos_pitch_td.dx);
    gimbal_controller.pitch_out = gimbal_controller.pitch_angle_pid.Output + gimbal_controller.pitch_speed_pid.Output + gimbal_controller.gravity_comp;
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
#if GIMBAL_SYSID
    PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.yaw_speed_pid.Ref);
    gimbal_controller.yaw_out = gimbal_controller.yaw_speed_pid.Output;
    return gimbal_controller.yaw_out;
#else
    // 输出滤波的角度，角速度，角加速度
    TD_Calculate(&gimbal_controller.pos_yaw_td, set_point);
    // 计算前馈力矩
    gimbal_controller.ff_tff =
        GIMBAL_YAW_J * gimbal_controller.pos_yaw_td.ddx +     // 惯量 × 加速度，主要输出项
        GIMBAL_YAW_B * gimbal_controller.pos_yaw_td.dx +      // 阻尼 × 速度
        GIMBAL_YAW_C * sign(gimbal_controller.pos_yaw_td.dx); // 库伦摩擦 × 速度方向   // pid闭环
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
 * @brief 由于重力补偿的作用，云台需要施加一个非线性力抵消重力影响，该力需要根据实际来进行测定
 */
volatile float pitch_comp[5] = {-0.024386f, -0.328869f, -0.138744f, 30.807712f, 1750.2f};

float GimbalPitchComp()
{
    // 记得每调一台车都需要重新更新参数
    return 0;
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

/*-----------------系统辨识初始化------------------*/
/*开启方式：
 *进入Debug，进入键鼠模式（就是只动云台）
 *把gimbal_sysid_done设置为0，开始运行系统辨识
 *输出结果为JBC三个参数，分别为转动惯量，阻尼系数，库伦摩擦系数
 *先测BC，再测J
 *测试时准备一根2m长的烧录线，防止被扯断
 */
void Gimbal_SystemID_Init(void)
{
    TD_Init(&gimbal_controller.yaw_sysid.td_omega, 10000.0f, 0.005f);
    gimbal_controller.yaw_sysid.sysid_timer = 0.0f;

    gimbal_controller.yaw_sysid.gimbal_sysid_done = 1;

#if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    RLS_Init(&gimbal_controller.yaw_sysid.rls_yaw, 2, 1, 0.99f);
#elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    RLS_Init(&gimbal_controller.yaw_sysid.rls_yaw, 1, 1, 0.99f);
#endif
}

void Gimbal_SystemID_Run(void)
{
#if GIMBAL_SYSID

    if (gimbal_controller.yaw_sysid.gimbal_sysid_done)
        return;

    float dt = gimbal_controller.delta_t;
    if (dt > 0.01f)
        dt = 0.002f;
    gimbal_controller.yaw_sysid.sysid_timer += dt;

    // ---------- Step 1: 稳态速度测试 -> 辨识 B, C ----------
#if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC

    const static float vel_pts[] = {-200.0f, -150.0f, -100.0f, -50.0f,
                                    50.0f, 100.0f, 150.0f, 200.0f};
    const static uint8_t NUM_PTS = sizeof(vel_pts) / sizeof(vel_pts[0]);
    const static float SETTLE_TIME = 0.4f;
    const static float REVS_PER_POINT = 1.0f;
    const static float DEG_PER_REV = 360.0f;

    static uint8_t step_idx = 0;
    static float step_timer = 0.0f;
    static float total_angle = 0.0f;
    /* 整段数据累加器 */
    static float torque_sum = 0.0f;
    static float omega_sum = 0.0f;
    static uint32_t sample_count = 0;

    step_timer += dt;
    gimbal_controller.yaw_speed_pid.Ref = vel_pts[step_idx];

    if (step_timer > SETTLE_TIME)
    {
        float omega_raw = gimbal_controller.gyro_yaw_speed;
        float torque = GIMBAL_YAW_MOTOR_SIGN * gimbal_controller.DM_Yaw_Motor.t_ff_Receive;

        total_angle += fabsf(omega_raw) * dt;
        torque_sum += torque;
        omega_sum += omega_raw;
        sample_count++;
    }

    /* 转够圈数 -> 求平均 -> 喂 RLS -> 跳转 */
    if (total_angle >= DEG_PER_REV * REVS_PER_POINT)
    {
        if (sample_count > 0)
        {
            float torque_avg = torque_sum / (float)sample_count;
            float omega_avg = omega_sum / (float)sample_count;
            float sign_w = (omega_avg > 0.01f) ? 1.0f : ((omega_avg < -0.01f) ? -1.0f : 0.0f);
            gimbal_controller.yaw_sysid.rls_yaw.H_data[0] = omega_avg;
            gimbal_controller.yaw_sysid.rls_yaw.H_data[1] = sign_w;
            gimbal_controller.yaw_sysid.rls_yaw.y_data[0] = torque_avg;
            RLS_Update(&gimbal_controller.yaw_sysid.rls_yaw);
        }
        step_timer = 0.0f;
        total_angle = 0.0f;
        step_idx++;
        torque_sum = 0.0f;
        omega_sum = 0.0f;
        sample_count = 0;
        TD_Clear(&gimbal_controller.yaw_sysid.td_omega, gimbal_controller.gyro_yaw_speed);
        if (step_idx >= NUM_PTS)
        {
            gimbal_controller.yaw_sysid.gimbal_sysid_done = 1;
            gimbal_controller.yaw_speed_pid.Ref = 0;
            gimbal_controller.yaw_sysid.B = gimbal_controller.yaw_sysid.rls_yaw.x_data[0];
            gimbal_controller.yaw_sysid.C = gimbal_controller.yaw_sysid.rls_yaw.x_data[1];
        }
    }

    // ---------- Step 2: 恒加速测试 -> 辨识 J ----------

#elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J

    static const float ACCEL = 80.0f;
    static const float MAX_SPEED = 250.0f;
    static float ramp_speed = 0.0f;

    /* 整段累加器 */
    static float torque_sum = 0.0f;
    static float omega_sum = 0.0f;
    static float alpha_sum = 0.0f;
    static uint32_t sample_count = 0;

    ramp_speed += ACCEL * dt;
    if (ramp_speed > MAX_SPEED)
        ramp_speed = MAX_SPEED;

    gimbal_controller.yaw_speed_pid.Ref = ramp_speed;

    /* 跳过起始瞬态后开始积累 */
    if (gimbal_controller.yaw_sysid.sysid_timer > 0.1f)
    {
        float omega_raw = gimbal_controller.gyro_yaw_speed;
        float omega_smooth = TD_Calculate(&gimbal_controller.yaw_sysid.td_omega, omega_raw);
        float alpha_smooth = gimbal_controller.yaw_sysid.td_omega.dx;
        float torque = GIMBAL_YAW_MOTOR_SIGN * gimbal_controller.DM_Yaw_Motor.t_ff_Receive;

        torque_sum += torque;
        omega_sum += omega_smooth;
        alpha_sum += alpha_smooth;
        sample_count++;
    }

    /* 斜坡结束，用实测均值算 J */
    if (ramp_speed >= MAX_SPEED)
    {
        if (sample_count > 0)
        {
            float torque_avg = torque_sum / (float)sample_count;
            float omega_avg = omega_sum / (float)sample_count;
            float alpha_avg = alpha_sum / (float)sample_count;

            gimbal_controller.yaw_sysid.J = (torque_avg - GIMBAL_YAW_B * omega_avg - GIMBAL_YAW_C) / alpha_avg;
        }
        gimbal_controller.yaw_sysid.gimbal_sysid_done = 1;
        gimbal_controller.yaw_speed_pid.Ref = 0;
    }
#endif

#endif
}
