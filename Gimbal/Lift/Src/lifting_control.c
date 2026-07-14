#include "lifting_control.h"

#include <math.h>
#include "Gimbal.h"
#include "debug.h"
#include "encoder.h"
#include "lifting_config.h"
#include "lifting_encoder_protection.h"
#include "lifting_tof_protection.h"
#include "remote_control.h"
#include "tof.h"

LiftingController lifting_controller;

static void init_active_protection(void)
{
#if ROBOT_SELECT == OLD
    LiftTofProtection_Init();
#else
    LiftEncoderProtection_Init();
#endif
}

static void begin_active_protection(void)
{
#if ROBOT_SELECT == OLD
    LiftTofProtection_BeginMotion();
#else
    LiftEncoderProtection_BeginMotion();
#endif
}

void LiftPidInit(void)
{
#if ROBOT_SELECT == NEW
    lifting_controller.high_state = LIFT_ENCODER_HIGH_POINT_COUNT;
    lifting_controller.low_state = LIFT_ENCODER_LOW_POINT_COUNT;
    lifting_controller.sensor_dir = LIFT_ENCODER_SENSOR_TO_MOTOR_DIR;
    PID_Init(&lifting_controller.lift_pos_pid, LIFT_SPEED_REF_MAX_DEG_S,
        0, 0, LIFT_ENCODER_POSITION_KP, 0, 0, 0, 0, 0, 0, 1, NONE);
#else
    lifting_controller.high_state = LIFT_TOF_HIGH_POINT_MM;
    lifting_controller.low_state = LIFT_TOF_LOW_POINT_MM;
    lifting_controller.sensor_dir = LIFT_TOF_SENSOR_TO_MOTOR_DIR;
    PID_Init(&lifting_controller.lift_pos_pid, LIFT_SPEED_REF_MAX_DEG_S,
        0, 0, LIFT_TOF_POSITION_KP, 0, 0, 0, 0, 0, 0, 1, NONE);
#endif

    /* 速度环输入始终是M2006输出轴deg/s，不随位置传感器量程缩放。 */
    PID_Init(&lifting_controller.lift_speed_pid, C610_MAX_CURRENT,
        LIFT_SPEED_KP, 0, LIFT_SPEED_KD, 0.0f, 0.0f, 50, 100, 0, 0, 1,
        Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate);

    lifting_controller.phase = LIFT_PHASE_IDLE;
    lifting_controller.finish_cnt = 0U;
    lifting_controller.settle_ticks = 0U;
    lifting_controller.send_current = 0.0f;
    init_active_protection();
}

static float read_position_sensor(void)
{
#if ROBOT_SELECT == NEW
    return (float)encoder.value;
#else
    /* OLD只允许通过毛刺检查的最后可信TOF进入位置PID。 */
    return lifting_controller.protection.trusted_tof;
#endif
}

static bool feedback_online(void)
{
#if ROBOT_SELECT == NEW
    return (global_debugger.encoder_debugger.state == ON &&
        global_debugger.lift_debugger.state == ON);
#else
    return (global_debugger.tof_debugger.state == ON &&
        global_debugger.lift_debugger.state == ON);
#endif
}

static void stop_output(void)
{
    PID_Clear(&lifting_controller.lift_pos_pid);
    PID_Clear(&lifting_controller.lift_speed_pid);
    lifting_controller.send_current = 0.0f;
}

/* 串级PID：位置误差生成速度目标，速度误差生成C610电流。 */
static void run_cascade_pid(float target_position)
{
    lifting_controller.lift_speed_pid.Ref = lifting_controller.sensor_dir *
        PID_Calculate(&lifting_controller.lift_pos_pid,
            read_position_sensor(), target_position);
    lifting_controller.send_current = PID_Calculate(
        &lifting_controller.lift_speed_pid,
        lifting_controller.lift_info.speed,
        lifting_controller.lift_speed_pid.Ref);
}

static bool position_stably_reached(void)
{
    float position_threshold;
    uint16_t finish_ticks;

#if ROBOT_SELECT == NEW
    position_threshold = LIFT_ENCODER_FINISH_POSITION_COUNT;
    finish_ticks = LIFT_ENCODER_FINISH_TICKS;
#else
    position_threshold = LIFT_TOF_FINISH_POSITION_MM;
    finish_ticks = LIFT_TOF_FINISH_TICKS;
#endif

    if (fabsf(lifting_controller.lift_pos_pid.Err) > position_threshold ||
        fabsf(lifting_controller.lift_info.speed) > LIFT_FINISH_SPEED_DEG_S)
    {
        lifting_controller.finish_cnt = 0U;
        return false;
    }

    lifting_controller.finish_cnt++;
    return (lifting_controller.finish_cnt >= finish_ticks);
}

static void enter_phase(LiftPhase new_phase)
{
    lifting_controller.phase = new_phase;
    lifting_controller.finish_cnt = 0U;
    lifting_controller.settle_ticks = 0U;
    begin_active_protection();

    /* 任何升降动作都要求云台先回正；下降阶段会等待return_flag清零。 */
    gimbal_controller.target_pitch_angle = 5.0f;
    gimbal_controller.return_flag = 1;
}

static void stop_with_fault(LiftFault fault)
{
    lifting_controller.protection.common.resume_phase = lifting_controller.phase;
    lifting_controller.protection.common.fault = fault;
    lifting_controller.phase = LIFT_PHASE_BLOCKED;
    stop_output();
}

/*
 * 单次运动计算顺序固定为：反馈在线 -> PID前传感器过滤 -> PID -> 机械保护。
 * protection_enabled仅用于OLD方向切换时暂停TOF判断；快速碰顶保护不会暂停。
 */
static bool run_motion_control(float target, bool protection_enabled)
{
    LiftFault fault;

    if (!feedback_online())
    {
        stop_with_fault(LIFT_FAULT_FEEDBACK_OFFLINE);
        return false;
    }

#if ROBOT_SELECT == OLD
    fault = LiftTofProtection_BeforePid(protection_enabled);
    if (fault != LIFT_FAULT_NONE)
    {
        stop_with_fault(fault);
        return false;
    }
#endif

    run_cascade_pid(target);

#if ROBOT_SELECT == OLD
    fault = LiftTofProtection_AfterPid();
#else
    fault = LiftEncoderProtection_AfterPid(protection_enabled);
#endif
    if (fault != LIFT_FAULT_NONE)
    {
        stop_with_fault(fault);
        return false;
    }

    return true;
}

static void run_ascending(void)
{
    if (lifting_controller.settle_ticks > 0U)
    {
        lifting_controller.settle_ticks--;
        if (!run_motion_control(lifting_controller.high_state, false))
        {
            return;
        }
    }
    else if (!run_motion_control(lifting_controller.high_state, true))
    {
        return;
    }

    if (position_stably_reached())
    {
        enter_phase(LIFT_PHASE_COMPLETE);
        stop_output();
    }
}

static void run_descending(void)
{
    if (gimbal_controller.return_flag != 0)
    {
        stop_output();
        return;
    }

    if (!run_motion_control(lifting_controller.low_state, true))
    {
        return;
    }

    if (position_stably_reached())
    {
        enter_phase(LIFT_PHASE_COMPLETE);
        stop_output();
    }
}

static void run_blocked(void)
{
    stop_output();

#if ROBOT_SELECT == OLD
    /* 只有场地造成的TOF遮挡可自动恢复；堵转、空转和离线保持锁定。 */
    if (lifting_controller.protection.common.fault != LIFT_FAULT_TOF_OCCLUDED ||
        !feedback_online())
    {
        lifting_controller.protection.recovery_frames = 0U;
        return;
    }
    if (LiftTofProtection_OcclusionRecovered())
    {
        LiftPhase resume_phase = lifting_controller.protection.common.resume_phase;
        enter_phase(resume_phase);
    }
#else
    /* NEW仅允许反馈离线在编码器和M2006都恢复后自动续跑。 */
    if (lifting_controller.protection.common.fault != LIFT_FAULT_FEEDBACK_OFFLINE ||
        !feedback_online())
    {
        return;
    }
    enter_phase(lifting_controller.protection.common.resume_phase);
#endif
}

static void handle_demand_change(uint8_t demand)
{
    switch (demand)
    {
    case UP:
    {
#if ROBOT_SELECT == OLD
        bool from_descend = (lifting_controller.phase == LIFT_PHASE_DESCENDING);
#endif
        enter_phase(LIFT_PHASE_ASCENDING);
#if ROBOT_SELECT == OLD
        if (from_descend)
        {
            lifting_controller.settle_ticks = LIFT_TOF_DIRECTION_SETTLE_TICKS;
        }
#endif
        break;
    }
    case DOWN:
        enter_phase(LIFT_PHASE_DESCENDING);
        break;
    case POWER_DOWN:
    default:
        enter_phase(LIFT_PHASE_IDLE);
        stop_output();
        break;
    }
}

void Lifting_Control(void)
{
    static uint8_t last_demand = POWER_DOWN;
    uint8_t demand = remote_controller.gimbal_position;

    if (demand != last_demand)
    {
        last_demand = demand;
        handle_demand_change(demand);
        return;
    }

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
        stop_output();
        break;
    }
}
