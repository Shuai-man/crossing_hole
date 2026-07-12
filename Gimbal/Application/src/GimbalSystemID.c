/**
 ******************************************************************************
 * @file    GimbalSystemID.c
 * @brief   云台系统辨识模块实现
 *          包含 Yaw / Pitch 轴的辨识测试逻辑以及公共工具
 *
 * 用法：
 *   1. 在合适位置调用 GimbalSystemID_Init(&gimbal_controller)
 *   2. 在循环中调用 GimbalSystemID_Run()
 *   3. 通过 GIMBAL_SYSID / GIMBAL_SYSID_STEP 编译宏切换轴和步骤
 ******************************************************************************
 */
#include "Gimbal.h" 
#include "GimbalSystemID.h"
#include "TD.h"
#include "tools.h"
#include "arm_math.h"
#include "ins.h"        /* ANGLE_TO_RAD_COEF */

/* ========== Pitch 模式选择（与旧 Gimbal.c 中一致） ========== */
#define PITCH_SYSID_MODE_B 0
#define PITCH_SYSID_MODE_J 1
#ifndef PITCH_SYSID_MODE
#define PITCH_SYSID_MODE PITCH_SYSID_MODE_B
#endif

/* Pitch 安全限位 / 配置常量（原 Gimbal.c 函数体内 #define） */
#define PITCH_SAFE_MIN_DEG    -16.5f
#define PITCH_SAFE_MAX_DEG     30.0f
#define PITCH_REVERSE_MARGIN    1.0f
#define PITCH_SCAN_CYCLES       3
#define PITCH_VEL_UP_DPS      120.0f
#define PITCH_VEL_DOWN_DPS      2.0f

/* ========== 模块级全局状态 ========== */
static GimbalController *ctrl = NULL;

/* ---------- Yaw 轴测试状态 ---------- */
static StepSequencer    yaw_bc_seq;
static SampleAccumulator yaw_j_torque_acc;
static SampleAccumulator yaw_j_omega_acc;
static SampleAccumulator yaw_j_alpha_acc;
static float            yaw_j_ramp_speed;

/* ---------- Pitch 轴测试状态 ---------- */
static BoundaryScanner  pitch_scanner;

/* ---------- 信号发生器实例 ---------- */
static StepFunction     speed_step;
static SquareWave       speed_square;   /* 注: 当前未在 Run 中使用，留作扩展 */

/* ==================================================================
 *  1. 采样累加器
 * ================================================================== */
void SAcc_Reset(SampleAccumulator *acc)
{
    acc->sum   = 0.0f;
    acc->count = 0;
}

void SAcc_Add(SampleAccumulator *acc, float val)
{
    acc->sum   += val;
    acc->count ++;
}

float SAcc_Mean(const SampleAccumulator *acc)
{
    if (acc->count == 0)
        return 0.0f;
    return acc->sum / (float)acc->count;
}

/* ==================================================================
 *  2. 多速度点步进序列
 * ================================================================== */
void StepSeq_Init(StepSequencer *seq, const float *pts, uint8_t n,
                  float settle, float revs, float deg_per_rev)
{
    seq->vel_pts         = pts;
    seq->num_pts         = n;
    seq->settle_time     = settle;
    seq->hold_revolutions = revs;
    seq->deg_per_rev     = deg_per_rev;

    seq->current_idx   = 0;
    seq->step_elapsed  = 0.0f;
    seq->step_angle    = 0.0f;
    seq->sample_count  = 0;
    seq->torque_sum    = 0.0f;
    seq->omega_sum     = 0.0f;
}

bool StepSeq_Run(StepSequencer *seq, float dt,
                 float omega, float torque,
                 bool *done, float *out_omega, float *out_torque)
{
    *done = false;

    if (seq->current_idx >= seq->num_pts)
    {
        *done = true;
        return false;
    }

    seq->step_elapsed += dt;

    /* 稳定期过后开始采样 */
    if (seq->step_elapsed > seq->settle_time)
    {
        seq->step_angle  += fabsf(omega) * dt;
        seq->torque_sum  += torque;
        seq->omega_sum   += omega;
        seq->sample_count++;
    }

    /* 转够圈数 → 输出平均数据，推进到下一个速度点 */
    if (seq->step_angle >= seq->deg_per_rev * seq->hold_revolutions)
    {
        if (seq->sample_count > 0)
        {
            *out_omega  = seq->omega_sum  / (float)seq->sample_count;
            *out_torque = seq->torque_sum / (float)seq->sample_count;
        }
        else
        {
            *out_omega  = 0.0f;
            *out_torque = 0.0f;
        }

        /* 重置当前步状态 */
        seq->step_elapsed = 0.0f;
        seq->step_angle   = 0.0f;
        seq->sample_count = 0;
        seq->torque_sum   = 0.0f;
        seq->omega_sum    = 0.0f;
        seq->current_idx++;

        if (seq->current_idx >= seq->num_pts)
            *done = true;

        return true;    /* 调用方可喂 RLS */
    }

    return false;
}

/* ==================================================================
 *  3. 角度边界往返扫描器
 * ================================================================== */
void BScan_Init_ConstVel(BoundaryScanner *bs,
                         float angle_min, float angle_max, float margin,
                         float vel_forward, float vel_backward,
                         uint8_t max_half_cycles)
{
    bs->angle_min       = angle_min;
    bs->angle_max       = angle_max;
    bs->margin          = margin;
    bs->max_half_cycles = max_half_cycles;

    bs->mode          = SCAN_CONST_VEL;
    bs->vel_forward   = vel_forward;
    bs->vel_backward  = vel_backward;
    bs->accel         = 0.0f;
    bs->max_speed     = 0.0f;

    bs->half_cycle_count = 0;
    bs->current_vel_ref  = vel_forward;
    bs->ramp_speed       = 0.0f;
}

void BScan_Init_AccelRamp(BoundaryScanner *bs,
                          float angle_min, float angle_max, float margin,
                          float accel, float max_speed,
                          uint8_t max_half_cycles)
{
    bs->angle_min       = angle_min;
    bs->angle_max       = angle_max;
    bs->margin          = margin;
    bs->max_half_cycles = max_half_cycles;

    bs->mode          = SCAN_ACCEL_RAMP;
    bs->vel_forward   = max_speed;
    bs->vel_backward  = max_speed;
    bs->accel         = accel;
    bs->max_speed     = max_speed;

    bs->half_cycle_count = 0;
    bs->current_vel_ref  = 0.0f;
    bs->ramp_speed       = 0.0f;
}

bool BScan_Run(BoundaryScanner *bs, float theta_deg, float dt,
               float *out_vel_ref)
{
    if (bs->mode == SCAN_CONST_VEL)
    {
        /* 到达上边界且正向 → 换向 */
        if (theta_deg > (bs->angle_max - bs->margin) && bs->current_vel_ref > 0)
        {
            bs->current_vel_ref = -bs->vel_backward;
            bs->half_cycle_count++;
        }
        /* 到达下边界且反向 → 换向 */
        else if (theta_deg < (bs->angle_min + bs->margin) && bs->current_vel_ref < 0)
        {
            bs->current_vel_ref = bs->vel_forward;
            bs->half_cycle_count++;
        }
    }
    else /* SCAN_ACCEL_RAMP */
    {
        /* 到达上边界 → 标记结束 (与原始逻辑一致) */
        if (theta_deg > (bs->angle_max - bs->margin) && bs->ramp_speed > 0)
        {
            bs->half_cycle_count = bs->max_half_cycles;
        }

        /* 斜坡加速 */
        bs->ramp_speed += bs->accel * dt;
        bs->ramp_speed  = LIMIT_MAX_MIN(bs->ramp_speed, bs->max_speed, -bs->max_speed);
        bs->current_vel_ref = bs->ramp_speed;
    }

    *out_vel_ref = bs->current_vel_ref;
    return (bs->half_cycle_count >= bs->max_half_cycles);
}

bool BScan_IsDone(const BoundaryScanner *bs)
{
    return (bs->half_cycle_count >= bs->max_half_cycles);
}

/* ==================================================================
 *  4. Yaw 轴 — 稳态速度测试 (辨识 B, C)
 * ================================================================== */
static void Yaw_StepBC_Run(void)
{
    static const float vel_pts[] = {
        -200.0f, -150.0f, -100.0f, -50.0f,
         50.0f,  100.0f,  150.0f,  200.0f
    };
    static bool seq_inited = false;

    if (!seq_inited)
    {
        StepSeq_Init(&yaw_bc_seq, vel_pts, 8, 0.4f, 1.0f, 360.0f);
        seq_inited = true;
    }

    /* 设置当前目标速度 */
    float target = (yaw_bc_seq.current_idx < 8)
                   ? yaw_bc_seq.vel_pts[yaw_bc_seq.current_idx]
                   : 0.0f;
    ctrl->yaw_speed_pid.Ref = target;

    /* 读取实时数据 */
    float dt = ctrl->delta_t;
    if (dt > 0.01f) dt = 0.002f;

    float omega = ctrl->gyro_yaw_speed;
    float torque = GIMBAL_YAW_MOTOR_SIGN * ctrl->DM_Yaw_Motor.t_ff_Receive;

    /* 运行步进序列 */
    bool  done;
    float out_omega, out_torque;
    bool  point_ready = StepSeq_Run(&yaw_bc_seq, dt, omega, torque,
                                     &done, &out_omega, &out_torque);

    if (point_ready)
    {
        float sign_w = (out_omega > 0.01f) ? 1.0f
                      : ((out_omega < -0.01f) ? -1.0f : 0.0f);

        ctrl->yaw_sysid.rls_sysid.H_data[0] = out_omega;
        ctrl->yaw_sysid.rls_sysid.H_data[1] = sign_w;
        ctrl->yaw_sysid.rls_sysid.y_data[0] = out_torque;
        RLS_Update(&ctrl->yaw_sysid.rls_sysid);

        TD_Clear(&ctrl->yaw_sysid.td_omega, ctrl->gyro_yaw_speed);
    }

    if (done)
    {
        ctrl->yaw_sysid.sysid_done = 1;
        ctrl->yaw_speed_pid.Ref = 0.0f;
        ctrl->yaw_sysid.B = ctrl->yaw_sysid.rls_sysid.x_data[0];
        ctrl->yaw_sysid.C = ctrl->yaw_sysid.rls_sysid.x_data[1];
    }
}

/* ==================================================================
 *  5. Yaw 轴 — 恒加速度测试 (辨识 J)
 * ================================================================== */
static void Yaw_StepJ_Run(void)
{
    static const float ACCEL     = 80.0f;
    static const float MAX_SPEED = 250.0f;

    float dt = ctrl->delta_t;
    if (dt > 0.01f) dt = 0.002f;

    /* 跳过起始瞬态 */
    if (ctrl->yaw_sysid.sysid_timer <= 0.1f)
        return;

    /* 斜坡加速 */
    yaw_j_ramp_speed += ACCEL * dt;
    if (yaw_j_ramp_speed > MAX_SPEED)
        yaw_j_ramp_speed = MAX_SPEED;

    ctrl->yaw_speed_pid.Ref = yaw_j_ramp_speed;

    /* 采集数据 */
    float omega_raw = ctrl->gyro_yaw_speed;
    float omega_smooth = TD_Calculate(&ctrl->yaw_sysid.td_omega, omega_raw);
    float alpha_smooth = ctrl->yaw_sysid.td_omega.dx;
    float torque = GIMBAL_YAW_MOTOR_SIGN * ctrl->DM_Yaw_Motor.t_ff_Receive;

    SAcc_Add(&yaw_j_torque_acc, torque);
    SAcc_Add(&yaw_j_omega_acc,  omega_smooth);
    SAcc_Add(&yaw_j_alpha_acc,  alpha_smooth);

    /* 斜坡结束 → 计算 J */
    if (yaw_j_ramp_speed >= MAX_SPEED)
    {
        float torque_avg = SAcc_Mean(&yaw_j_torque_acc);
        float omega_avg  = SAcc_Mean(&yaw_j_omega_acc);
        float alpha_avg  = SAcc_Mean(&yaw_j_alpha_acc);

        ctrl->yaw_sysid.J = (torque_avg - GIMBAL_YAW_B * omega_avg - GIMBAL_YAW_C)
                            / alpha_avg;

        ctrl->yaw_sysid.sysid_done = 1;
        ctrl->yaw_speed_pid.Ref = 0.0f;

        SAcc_Reset(&yaw_j_torque_acc);
        SAcc_Reset(&yaw_j_omega_acc);
        SAcc_Reset(&yaw_j_alpha_acc);
    }
}

/* ==================================================================
 *  6. Pitch 轴 — 边界往返扫描 (恒定速度 / 加速度斜坡)
 * ================================================================== */
static void Pitch_Run(void)
{
    float dt = ctrl->delta_t;
    if (dt > 0.01f) dt = 0.002f;

    float theta_deg   = ctrl->gyro_pitch_angle;
    float omega_raw   = ctrl->gyro_pitch_speed;
    float torque_raw  = GIMBAL_PITCH_MOTOR_SIGN * ctrl->DM_Pitch_Motor.t_ff_Receive;

    /* 角度 → 弧度 */
    float theta_rad   = theta_deg * ANGLE_TO_RAD_COEF;

    /* 重力补偿和库仑摩擦 */
    float G      = GIMBAL_PITCH_A * sin(theta_rad) + GIMBAL_PITCH_B * cos(theta_rad);
    float C_sign = (omega_raw > 0.01f)  ?  GIMBAL_PITCH_C
                 : ((omega_raw < -0.01f) ? -GIMBAL_PITCH_C : 0.0f);

    /* 运行边界扫描，获取速度参考值 */
    float vel_ref;
    BScan_Run(&pitch_scanner, theta_deg, dt, &vel_ref);
    ctrl->pitch_speed_pid.Ref = vel_ref;

#if PITCH_SYSID_MODE == PITCH_SYSID_MODE_B
    float T_comp = torque_raw - G - C_sign;

    if (fabsf(omega_raw) > 0.5f)
    {
        ctrl->pitch_sysid.rls_sysid.H_data[0] = omega_raw;
        ctrl->pitch_sysid.rls_sysid.y_data[0] = T_comp;
        RLS_Update(&ctrl->pitch_sysid.rls_sysid);
    }
#elif PITCH_SYSID_MODE == PITCH_SYSID_MODE_J
    float omega_smooth = TD_Calculate(&ctrl->pitch_sysid.td_omega, omega_raw);
    float alpha_smooth = ctrl->pitch_sysid.td_omega.dx;

    float T_comp = torque_raw - G - C_sign - GIMBAL_PITCH_CB * omega_smooth;

    if (fabsf(omega_raw) > 5.0f)
    {
        ctrl->pitch_sysid.rls_sysid.H_data[0] = alpha_smooth;
        ctrl->pitch_sysid.rls_sysid.y_data[0] = T_comp;
        RLS_Update(&ctrl->pitch_sysid.rls_sysid);
    }
#endif

    /* 完成 → 保存辨识结果 */
    if (BScan_IsDone(&pitch_scanner))
    {
        ctrl->pitch_sysid.sysid_done = 1;
        ctrl->pitch_speed_pid.Ref = 0.0f;

#if PITCH_SYSID_MODE == PITCH_SYSID_MODE_B
        ctrl->pitch_sysid.B = ctrl->pitch_sysid.rls_sysid.x_data[0];
#elif PITCH_SYSID_MODE == PITCH_SYSID_MODE_J
        ctrl->pitch_sysid.J = ctrl->pitch_sysid.rls_sysid.x_data[0];
#endif
    }

    /* 紧急保护 */
    if (theta_deg < PITCH_SAFE_MIN_DEG || theta_deg > PITCH_SAFE_MAX_DEG)
    {
        ctrl->pitch_speed_pid.Ref = 0.0f;
        ctrl->pitch_sysid.sysid_done = 1;
    }
}

/* ==================================================================
 *  7. 顶层接口
 * ================================================================== */
void GimbalSystemID_Init(GimbalController *controller)
{
    ctrl = controller;

#if GIMBAL_SYSID
    /* 通用初始化 */
    TD_Init(&ctrl->yaw_sysid.td_omega,   10000.0f, 0.005f);
    TD_Init(&ctrl->pitch_sysid.td_omega, 10000.0f, 0.005f);

    ctrl->yaw_sysid.sysid_timer   = 0.0f;
    ctrl->yaw_sysid.sysid_done    = 1;
    ctrl->pitch_sysid.sysid_timer = 0.0f;
    ctrl->pitch_sysid.sysid_done  = 1;

    /* 清空模块级状态 */
    SAcc_Reset(&yaw_j_torque_acc);
    SAcc_Reset(&yaw_j_omega_acc);
    SAcc_Reset(&yaw_j_alpha_acc);
    yaw_j_ramp_speed  = 0.0f;

#if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    RLS_Init(&ctrl->yaw_sysid.rls_sysid,   2, 1, 0.99f);
    RLS_Init(&ctrl->pitch_sysid.rls_sysid, 1, 1, 0.99f);
    SquareWaveInit(&speed_square, 120.0f, 2.0f, 0.0f, 100.0f);

    /* 初始化 Pitch 边界扫描器 (恒定速度模式) */
    BScan_Init_ConstVel(&pitch_scanner,
                        -16.5f, 30.0f, 1.0f,
                        120.0f, 2.0f, 6);

#elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    RLS_Init(&ctrl->yaw_sysid.rls_sysid,   1, 1, 0.99f);
    RLS_Init(&ctrl->pitch_sysid.rls_sysid, 1, 1, 0.99f);
    StepInit(&speed_step, 0.0f, 120.0f, 0.0f);

    /* 初始化 Pitch 边界扫描器 (加速度斜坡模式) */
    BScan_Init_AccelRamp(&pitch_scanner,
                         -16.5f, 30.0f, 1.0f,
                         80.0f, 120.0f, 6);
#endif
#endif /* GIMBAL_SYSID */
}

void GimbalSystemID_Run(void)
{
    float dt = ctrl->delta_t;
    if (dt > 0.01f)
        dt = 0.002f;

#if GIMBAL_SYSID == GIMBAL_YAW_SYSID
    if (ctrl->yaw_sysid.sysid_done)
        return;
    ctrl->yaw_sysid.sysid_timer += dt;

#if GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_BC
    Yaw_StepBC_Run();
#elif GIMBAL_SYSID_STEP == GIMBAL_SYSID_STEP_J
    Yaw_StepJ_Run();
#endif

#elif GIMBAL_SYSID == GIMBAL_PITCH_SYSID
    if (ctrl->pitch_sysid.sysid_done)
        return;
    ctrl->pitch_sysid.sysid_timer += dt;
    Pitch_Run();
#endif
}
