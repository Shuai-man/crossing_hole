#include "lifting_tof_protection.h"

#if ROBOT_SELECT == OLD

#include <math.h>
#include "lifting_config.h"
#include "lifting_control.h"
#include "tof.h"

static float motor_direction_for_phase(LiftPhase phase)
{
    return (phase == LIFT_PHASE_ASCENDING) ? -1.0f : 1.0f;
}

static void reset_fast_stall_window(bool clear_confirmation)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->fast_window_ticks = 0U;
    protection->fast_effort_ticks = 0U;
    protection->fast_window_motor_angle = lifting_controller.lift_info.angle;
    if (clear_confirmation)
    {
        protection->fast_stall_windows = 0U;
    }
}

static void reset_spin_frame_window(bool clear_confirmation)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->spin_window_frames = 0U;
    protection->window_motor_angle = lifting_controller.lift_info.angle;
    protection->window_tof = protection->trusted_tof;
    if (clear_confirmation)
    {
        protection->spin_windows = 0U;
    }
}

static bool read_new_tof(float *distance)
{
    LiftProtection *protection = &lifting_controller.protection;
    uint32_t update_counter = Tof_UpdateCounter;

    if (update_counter == protection->last_tof_update)
    {
        return false;
    }

    protection->last_tof_update = update_counter;
    *distance = (float)Tof_ReceiveData.distance;
    return true;
}

static void update_odometry_ratio(float motor_delta, float tof_delta)
{
    LiftProtection *protection = &lifting_controller.protection;
    float ratio;
    float abs_ratio;

    if (fabsf(motor_delta) < LIFT_TOF_ODOM_MIN_MOTOR_DEG)
    {
        return;
    }

    ratio = tof_delta / motor_delta;
    abs_ratio = fabsf(ratio);
    if (abs_ratio < LIFT_TOF_ODOM_RATIO_MIN || abs_ratio > LIFT_TOF_ODOM_RATIO_MAX)
    {
        return;
    }

    if (!protection->ratio_valid)
    {
        protection->tof_per_motor_degree = ratio;
        protection->ratio_valid = true;
    }
    else
    {
        protection->tof_per_motor_degree += LIFT_TOF_ODOM_RATIO_FILTER *
            (ratio - protection->tof_per_motor_degree);
    }
}

/* 每收到指定数量的可信TOF新帧才评估一次，彻底摆脱1kHz窗口与TOF帧率的耦合。 */
static LiftFault evaluate_spin_on_trusted_frame(bool detection_enabled)
{
    LiftProtection *protection = &lifting_controller.protection;
    bool tof_stable;

    if (!detection_enabled)
    {
        reset_spin_frame_window(true);
        return LIFT_FAULT_NONE;
    }

    protection->spin_window_frames++;
    if (protection->spin_window_frames < LIFT_TOF_SPIN_WINDOW_FRAMES)
    {
        return LIFT_FAULT_NONE;
    }

    protection->motor_delta = lifting_controller.lift_info.angle -
        protection->window_motor_angle;
    protection->tof_delta = protection->trusted_tof - protection->window_tof;
    protection->motor_progress = protection->motor_delta *
        motor_direction_for_phase(lifting_controller.phase);
    protection->tof_progress = protection->tof_delta *
        ((lifting_controller.phase == LIFT_PHASE_ASCENDING) ? 1.0f : -1.0f);
    tof_stable = (fabsf(protection->tof_delta) <= LIFT_TOF_STABLE_MM);

    if (tof_stable && protection->motor_progress >= LIFT_TOF_SPIN_MOTOR_MIN_DEG)
    {
        protection->spin_windows++;
    }
    else
    {
        protection->spin_windows = 0U;
    }

    if (protection->motor_progress > LIFT_TOF_ODOM_MIN_MOTOR_DEG &&
        protection->tof_progress > LIFT_TOF_STABLE_MM)
    {
        update_odometry_ratio(protection->motor_delta, protection->tof_delta);
    }

    if (protection->spin_windows >= LIFT_TOF_SPIN_CONFIRM_WINDOWS)
    {
        reset_spin_frame_window(false);
        return LIFT_FAULT_MOTOR_SPIN;
    }

    reset_spin_frame_window(false);
    return LIFT_FAULT_NONE;
}

void LiftTofProtection_Init(void)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->tof_per_motor_degree = 0.0f;
    protection->ratio_valid = false;
    protection->suspect_tof_count = 0U;
    LiftTofProtection_BeginMotion();
}

void LiftTofProtection_BeginMotion(void)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->common.fault = LIFT_FAULT_NONE;
    protection->common.resume_phase = lifting_controller.phase;
    protection->startup_ticks = LIFT_TOF_PROTECT_STARTUP_TICKS;
    protection->occlusion_frames = 0U;
    protection->recovery_frames = 0U;
    protection->last_tof_update = Tof_UpdateCounter;
    protection->trusted_motor_angle = lifting_controller.lift_info.angle;
    protection->trusted_tof = (float)Tof_ReceiveData.distance;
    protection->recovery_motor_angle = protection->trusted_motor_angle;
    protection->recovery_tof = protection->trusted_tof;
    protection->motor_delta = 0.0f;
    protection->tof_delta = 0.0f;
    protection->motor_progress = 0.0f;
    protection->tof_progress = 0.0f;
    protection->odometry_residual = 0.0f;

    protection->fast_startup_ticks = LIFT_TOF_FAST_STALL_STARTUP_TICKS;
    protection->fast_motor_progress = 0.0f;
    reset_fast_stall_window(true);
    reset_spin_frame_window(true);
}

LiftFault LiftTofProtection_BeforePid(bool detection_enabled)
{
    LiftProtection *protection = &lifting_controller.protection;
    float current_tof;
    bool armed;

    if (protection->startup_ticks > 0U)
    {
        protection->startup_ticks--;
    }
    armed = detection_enabled && (protection->startup_ticks == 0U) &&
        (fabsf(lifting_controller.lift_pos_pid.Err) > LIFT_TOF_PROTECT_TARGET_GUARD_MM);

    if (!read_new_tof(&current_tof))
    {
        if (!armed)
        {
            protection->occlusion_frames = 0U;
            reset_spin_frame_window(true);
        }
        return LIFT_FAULT_NONE;
    }

    protection->motor_delta = lifting_controller.lift_info.angle -
        protection->trusted_motor_angle;
    protection->tof_delta = current_tof - protection->trusted_tof;
    if (protection->ratio_valid)
    {
        float expected_tof_delta = protection->motor_delta *
            protection->tof_per_motor_degree;
        protection->odometry_residual = protection->tof_delta - expected_tof_delta;
    }
    else
    {
        protection->odometry_residual = protection->tof_delta;
    }

    if (armed && lifting_controller.phase == LIFT_PHASE_ASCENDING &&
        protection->tof_delta <= -LIFT_TOF_OCCLUSION_DROP_MM)
    {
        bool mileage_mismatch;

        if (protection->ratio_valid)
        {
            mileage_mismatch =
                (fabsf(protection->odometry_residual) >= LIFT_TOF_ODOM_RESIDUAL_MM);
        }
        else
        {
            float motor_progress = protection->motor_delta *
                motor_direction_for_phase(lifting_controller.phase);
            mileage_mismatch = (motor_progress >= -LIFT_TOF_ODOM_MIN_MOTOR_DEG);
        }

        if (mileage_mismatch)
        {
            if (protection->occlusion_frames < LIFT_TOF_OCCLUSION_CONFIRM_FRAMES)
            {
                protection->occlusion_frames++;
            }
            if (protection->suspect_tof_count < UINT32_MAX)
            {
                protection->suspect_tof_count++;
            }

            /* 单帧只隔离；连续异常才保存恢复基准并确认遮挡。 */
            if (protection->occlusion_frames < LIFT_TOF_OCCLUSION_CONFIRM_FRAMES)
            {
                return LIFT_FAULT_NONE;
            }

            protection->recovery_motor_angle = protection->trusted_motor_angle;
            protection->recovery_tof = protection->trusted_tof;
            return LIFT_FAULT_TOF_OCCLUDED;
        }
    }

    protection->occlusion_frames = 0U;
    protection->trusted_motor_angle = lifting_controller.lift_info.angle;
    protection->trusted_tof = current_tof;
    return evaluate_spin_on_trusted_frame(armed);
}

LiftFault LiftTofProtection_AfterPid(void)
{
    LiftProtection *protection = &lifting_controller.protection;
    bool finish_candidate;
    bool high_effort;

    if (protection->fast_startup_ticks > 0U)
    {
        protection->fast_startup_ticks--;
        reset_fast_stall_window(true);
        return LIFT_FAULT_NONE;
    }

    finish_candidate = (fabsf(lifting_controller.lift_pos_pid.Err) <=
        LIFT_TOF_FINISH_POSITION_MM) &&
        (fabsf(lifting_controller.lift_info.speed) <= LIFT_FINISH_SPEED_DEG_S);
    if (finish_candidate)
    {
        reset_fast_stall_window(true);
        return LIFT_FAULT_NONE;
    }

    protection->fast_window_ticks++;
    if (fabsf(lifting_controller.send_current) >= LIFT_TOF_FAST_STALL_CURRENT_MIN ||
        fabsf(lifting_controller.lift_info.torque_current) >= LIFT_TOF_FAST_STALL_CURRENT_MIN)
    {
        protection->fast_effort_ticks++;
    }
    if (protection->fast_window_ticks < LIFT_TOF_FAST_STALL_WINDOW_TICKS)
    {
        return LIFT_FAULT_NONE;
    }

    protection->fast_motor_progress =
        (lifting_controller.lift_info.angle - protection->fast_window_motor_angle) *
        motor_direction_for_phase(lifting_controller.phase);
    high_effort = ((uint32_t)protection->fast_effort_ticks * 100U >=
        (uint32_t)protection->fast_window_ticks * LIFT_TOF_FAST_STALL_EFFORT_PERCENT);

    if (high_effort &&
        protection->fast_motor_progress <= LIFT_TOF_FAST_STALL_MOTOR_MAX_DEG)
    {
        protection->fast_stall_windows++;
    }
    else
    {
        protection->fast_stall_windows = 0U;
    }

    if (protection->fast_stall_windows >= LIFT_TOF_FAST_STALL_CONFIRM_WINDOWS)
    {
        reset_fast_stall_window(false);
        return LIFT_FAULT_MOTOR_STALL;
    }

    reset_fast_stall_window(false);
    return LIFT_FAULT_NONE;
}

bool LiftTofProtection_OcclusionRecovered(void)
{
    LiftProtection *protection = &lifting_controller.protection;
    float current_tof;
    float expected_tof;

    if (!read_new_tof(&current_tof))
    {
        return false;
    }

    expected_tof = protection->recovery_tof;
    if (protection->ratio_valid)
    {
        expected_tof +=
            (lifting_controller.lift_info.angle - protection->recovery_motor_angle) *
            protection->tof_per_motor_degree;
    }
    protection->odometry_residual = current_tof - expected_tof;

    if (fabsf(protection->odometry_residual) <= LIFT_TOF_RECOVERY_TOLERANCE_MM)
    {
        if (protection->recovery_frames < LIFT_TOF_RECOVERY_FRAMES)
        {
            protection->recovery_frames++;
        }
    }
    else
    {
        protection->recovery_frames = 0U;
    }

    return (protection->recovery_frames >= LIFT_TOF_RECOVERY_FRAMES);
}

#endif
