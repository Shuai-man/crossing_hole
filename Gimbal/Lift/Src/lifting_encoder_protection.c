#include "lifting_encoder_protection.h"

#if ROBOT_SELECT == NEW

#include <math.h>
#include "encoder.h"
#include "lifting_config.h"
#include "lifting_control.h"

static int8_t motor_direction_from_speed(float speed_ref)
{
    if (speed_ref > 0.0f)
    {
        return 1;
    }
    if (speed_ref < 0.0f)
    {
        return -1;
    }
    return 0;
}

static float encoder_direction_for_phase(LiftPhase phase)
{
    return (phase == LIFT_PHASE_ASCENDING) ? -1.0f : 1.0f;
}

static void reset_window(bool clear_confirmation)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->window_ticks = 0U;
    protection->effort_ticks = 0U;
    protection->encoder_samples = 0U;
    protection->last_encoder_update = Encoder_UpdateCounter;
    protection->window_motor_angle = lifting_controller.lift_info.angle;
    protection->window_encoder = (float)encoder.value;
    if (clear_confirmation)
    {
        protection->stall_windows = 0U;
        protection->spin_windows = 0U;
    }
}

void LiftEncoderProtection_Init(void)
{
    LiftEncoderProtection_BeginMotion();
}

void LiftEncoderProtection_BeginMotion(void)
{
    LiftProtection *protection = &lifting_controller.protection;

    protection->common.fault = LIFT_FAULT_NONE;
    protection->common.resume_phase = lifting_controller.phase;
    protection->startup_ticks = LIFT_ENCODER_PROTECT_STARTUP_TICKS;
    protection->motor_direction = 0;
    protection->motor_delta = 0.0f;
    protection->encoder_delta = 0.0f;
    protection->motor_progress = 0.0f;
    protection->sensor_progress = 0.0f;
    reset_window(true);
}

LiftFault LiftEncoderProtection_AfterPid(bool detection_enabled)
{
    LiftProtection *protection = &lifting_controller.protection;
    uint32_t encoder_updates;
    uint32_t sample_room;
    int8_t motor_direction;
    bool finish_candidate;
    bool encoder_not_advancing;
    bool high_effort;
    bool enough_samples;
    LiftFault fault = LIFT_FAULT_NONE;

    if (protection->startup_ticks > 0U)
    {
        protection->startup_ticks--;
    }

    finish_candidate = (fabsf(lifting_controller.lift_pos_pid.Err) <=
        LIFT_ENCODER_FINISH_POSITION_COUNT) &&
        (fabsf(lifting_controller.lift_info.speed) <= LIFT_FINISH_SPEED_DEG_S);
    if (!detection_enabled || protection->startup_ticks > 0U || finish_candidate)
    {
        protection->motor_direction = 0;
        reset_window(true);
        return LIFT_FAULT_NONE;
    }

    motor_direction = motor_direction_from_speed(lifting_controller.lift_speed_pid.Ref);
    if (motor_direction == 0)
    {
        protection->motor_direction = 0;
        reset_window(true);
        return LIFT_FAULT_NONE;
    }
    if (protection->motor_direction != motor_direction)
    {
        protection->motor_direction = motor_direction;
        reset_window(true);
    }

    encoder_updates = Encoder_UpdateCounter - protection->last_encoder_update;
    protection->last_encoder_update = Encoder_UpdateCounter;
    sample_room = (uint32_t)UINT16_MAX - protection->encoder_samples;
    if (encoder_updates > sample_room)
    {
        protection->encoder_samples = UINT16_MAX;
    }
    else
    {
        protection->encoder_samples += (uint16_t)encoder_updates;
    }

    protection->window_ticks++;
    if (fabsf(lifting_controller.send_current) >= LIFT_ENCODER_PROTECT_CURRENT_MIN ||
        fabsf(lifting_controller.lift_info.torque_current) >= LIFT_ENCODER_PROTECT_CURRENT_MIN)
    {
        protection->effort_ticks++;
    }
    if (protection->window_ticks < LIFT_ENCODER_PROTECT_WINDOW_TICKS)
    {
        return LIFT_FAULT_NONE;
    }

    protection->motor_delta = lifting_controller.lift_info.angle -
        protection->window_motor_angle;
    protection->encoder_delta = (float)encoder.value - protection->window_encoder;
    protection->motor_progress = protection->motor_delta * (float)motor_direction;
    protection->sensor_progress = protection->encoder_delta *
        encoder_direction_for_phase(lifting_controller.phase);
    encoder_not_advancing =
        (protection->sensor_progress <= LIFT_ENCODER_STABLE_COUNT);
    high_effort = ((uint32_t)protection->effort_ticks * 100U >=
        (uint32_t)protection->window_ticks * LIFT_ENCODER_PROTECT_EFFORT_PERCENT);
    enough_samples =
        (protection->encoder_samples >= LIFT_ENCODER_PROTECT_MIN_SAMPLES);

    if (enough_samples && encoder_not_advancing &&
        protection->motor_progress >= LIFT_ENCODER_MOTOR_SPIN_DEG)
    {
        protection->spin_windows++;
        protection->stall_windows = 0U;
    }
    else if (enough_samples && high_effort && encoder_not_advancing)
    {
        protection->stall_windows++;
        protection->spin_windows = 0U;
    }
    else
    {
        protection->stall_windows = 0U;
        protection->spin_windows = 0U;
    }

    if (protection->spin_windows >= LIFT_ENCODER_PROTECT_CONFIRM_WINDOWS)
    {
        fault = LIFT_FAULT_MOTOR_SPIN;
    }
    else if (protection->stall_windows >= LIFT_ENCODER_PROTECT_CONFIRM_WINDOWS)
    {
        fault = LIFT_FAULT_MOTOR_STALL;
    }

    reset_window(false);
    return fault;
}

#endif
