#include "Gimbal.h"
#include "bsp_dwt.h"

GimbalController gimbal_controller;

void GimbalMotorInit(void)
{
    DM_Motor_Init(&gimbal_controller.DM_Pitch_Motor, P_MAX, 10, V_MAX);
    //	DM_Motor_Init(&gimbal_controller.DM_Yaw_Motor,P_MAX,T_MAX,V_MAX);
    DM_Motor_Init(&gimbal_controller.DM_Yaw_Motor, P_MAX, 10, V_MAX); // 为了方便调参，放大最大值;pid输出限幅要对应修改
}

// 系统辨识初始化

static float sysid_timer = 0.0f;
static float sysid_speed_cmd = 0.0f;
float J, B, C = 0;

// 系统辨识初始化（移除扫频相关）
void Gimbal_SystemID_Init(void)
{
    TD_Init(&gimbal_controller.td_sysid_omega, 10.0f, 0.005f); // r 设小一点，专门用来压噪声
    // RLS 初始化（不变）
    RLS_Init(&gimbal_controller.rls_yaw, 3, 1, 0.99f);

    // 重置计时器
    sysid_timer = 0.0f;
    sysid_speed_cmd = 0.0f;

    gimbal_controller.gimbal_sysid_done = 1; // debug设为0启动，启动后云台会自动旋转，直到辨识完成
    gimbal_controller.gimbal_sysid_last_speed = 0.0f;
}

// 系统辨识单步运行（500Hz 调用）
void Gimbal_SystemID_Run(void)
{
#if GIMBAL_SYSID

    if (gimbal_controller.gimbal_sysid_done)
        return;

    // ===== 1. 生成梯形波速度指令（正反转交替） =====
    float dt = gimbal_controller.delta_t;
    sysid_timer += dt;

// 梯形波参数（单位：秒）
#define T_FORWARD 5.0f  // 正转持续时间
#define T_STOP1 1.0f    // 正转后停顿
#define T_REVERSE 5.0f  // 反转持续时间
#define T_STOP2 1.0f    // 反转后停顿
#define SPEED_AMP 20.0f // 最大速度 (°/s)，绝对安全

    float cycle_time = T_FORWARD + T_STOP1 + T_REVERSE + T_STOP2;
    float t_phase = fmodf(sysid_timer, cycle_time);

    if (t_phase < T_FORWARD)
        sysid_speed_cmd = SPEED_AMP;
    else if (t_phase < T_FORWARD + T_STOP1)
        sysid_speed_cmd = 0.0f;
    else if (t_phase < T_FORWARD + T_STOP1 + T_REVERSE)
        sysid_speed_cmd = -SPEED_AMP;
    else
        sysid_speed_cmd = 0.0f;

    // （可选）加入软启动，限制加速度防止冲击
    static float smooth_speed = 0.0f;
    float max_accel = 20.0f; // rad/s²，限制加加速度
    if (smooth_speed < sysid_speed_cmd)
        smooth_speed += max_accel * dt;
    else if (smooth_speed > sysid_speed_cmd)
        smooth_speed -= max_accel * dt;
    else
        smooth_speed = sysid_speed_cmd;
    // 限幅到目标值附近，避免积分误差
    if (fabsf(smooth_speed - sysid_speed_cmd) < 0.01f)
        smooth_speed = sysid_speed_cmd;

    // 最终指令，注入速度环
    gimbal_controller.yaw_speed_pid.Ref = smooth_speed;

    // ===== 2. 采集测量数据（不变） =====
    float omega_raw = gimbal_controller.gyro_yaw_speed;

    // 使用 TD 对速度进行滤波，并提取平滑后的速度和加速度
    float omega_processed = TD_Calculate(&gimbal_controller.td_sysid_omega, omega_raw);

    // TD 内部自带的 dx 就是平滑速度，ddx 就是平滑加速度（注意查看你的 TD 结构体是否有 ddx 输出）
    // 如果你的 TD_Calculate 只返回 x，请确认 td->dx 和 td->ddx 是否更新
    float alpha_smooth = gimbal_controller.td_sysid_omega.dx;
    float omega_smooth = gimbal_controller.td_sysid_omega.x;
    float torque = GIMBAL_YAW_MOTOR_SIGN * gimbal_controller.DM_Yaw_Motor.t_ff_Receive; // 反馈力矩

    // 为了时间对齐，使用历史缓冲区（但此处省略，你可以按之前建议加入）
    // 如果加，这里用对齐后的扭矩，否则直接用 torque

    // ===== 3. 更新 RLS（增加过零保护） =====
    // 只有速度绝对值大于阈值（正在匀速运动）才更新
    // 并且在过零点附近（速度太小）跳过，防止 sign 抖动
    if (fabsf(omega_smooth) > 0.05f && fabsf(torque) > 0.001f)
    {
        // 填充 H 矩阵
        gimbal_controller.rls_yaw.H_data[0] = alpha_smooth;
        gimbal_controller.rls_yaw.H_data[1] = omega_smooth;
        gimbal_controller.rls_yaw.H_data[2] = (omega_smooth > 0.01f) ? 1.0f : ((omega_smooth < -0.01f) ? -1.0f : 0.0f);
        gimbal_controller.rls_yaw.y_data[0] = torque;

        // 更新 RLS
        RLS_Update(&gimbal_controller.rls_yaw);
    }

    // ===== 4. 判定辨识完成 =====
    // 可以设定运行时间，比如跑 30 秒后自动结束
    if (sysid_timer > 30.0f)
    {
        gimbal_controller.gimbal_sysid_done = 1;
        gimbal_controller.yaw_speed_pid.Ref = 0;
        // 提取辨识参数
        J = gimbal_controller.rls_yaw.x_data[0];
        B = gimbal_controller.rls_yaw.x_data[1];
        C = gimbal_controller.rls_yaw.x_data[2];
    }

#endif
}

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit(void)
{

    // Pitch
    PID_Init(&gimbal_controller.pitch_angle_pid, 5000.0f, 0, 0.0f, 1000.0f, 500, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 60.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);
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

void GimbalPidChange(void)
{

    // Pitch
    PID_Change(&gimbal_controller.pitch_angle_pid, 500.0f, 0, 0.0f, 20.0f, 0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Change(&gimbal_controller.pitch_speed_pid, 5000, 4000, 0.0f, 40.0f, 100.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit);

    // Yaw
    PID_Change(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 20.0f, 0, 0.1f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Change(&gimbal_controller.yaw_speed_pid, 9000, 1600, 0.0, 120.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);
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
// todo 把重力补偿加上
float Gimbal_Pitch_Calculate(float set_point)
{
    TD_Calculate(&gimbal_controller.pos_pitch_td, set_point);
    PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.pos_pitch_td.x);
    PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, gimbal_controller.pos_pitch_td.dx);
    gimbal_controller.pitch_out = gimbal_controller.pitch_angle_pid.Output + gimbal_controller.pitch_speed_pid.Output;
    return gimbal_controller.pitch_out;
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
#if GIMBAL_SYSID
    PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, gimbal_controller.yaw_speed_pid.Ref);
    gimbal_controller.yaw_out = gimbal_controller.yaw_speed_pid.Output;
    return gimbal_controller.yaw_out;
#else
    // 由前馈电流负责控制，pid只负责闭环修正位置
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
    gimbal_controller.comp_pitch_current = 0;

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
    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.3);

    // yaw角度
    gimbal_controller.gyro_yaw_angle = GIMBAL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    // yaw角速度
    speed = GIMBAL_YAW_GYRO_SIGN * INS.Gyro[2] / PI * 180.0f;
    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.6);//还是超调可以试试加大滤波
}

/**
 * @brief 由于重力补偿的作用，云台需要施加一个非线性力抵消重力影响，该力需要根据实际来进行测定
 */
volatile float pitch_comp[5] = {-0.024386f, -0.328869f, -0.138744f, 30.807712f, 1750.2f};

float GimbalPitchComp()
{
    // 记得每调一台车都需要重新更新参数
    float x[4];

    // 解析静止时的非线性函数，只能大致补偿，然后靠PID的I使最终无静差
    // 低于一定角度或高于一定角度，根据测量结果，输出应大致不变
    x[3] = LIMIT_MAX_MIN(gimbal_controller.gyro_pitch_angle, 8, -12);
    x[2] = x[3] * x[3];
    x[1] = x[2] * x[3];
    x[0] = x[1] * x[3];

    float sum = pitch_comp[4];
    for (int i = 0; i < 4; i++)
    {
        sum += x[i] * pitch_comp[i];
    }
    iir(&gimbal_controller.comp_pitch_current, sum * GIMBAL_PITCH_COMP_COEF, 0.7f);
    return gimbal_controller.comp_pitch_current;
    // iir(&gimbal_controller.comp_pitch_current, GIMBAL_PITCH_COMP * arm_cos_f32(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) * GIMBAL_PITCH_COMP_COEF, 0.7);
    // return gimbal_controller.comp_pitch_current;
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
