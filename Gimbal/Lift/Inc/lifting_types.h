#ifndef LIFTING_TYPES_H
#define LIFTING_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "Robot_config.h"

typedef enum {
    LIFT_PHASE_IDLE = 0,   // 待机，无电流输出
    LIFT_PHASE_ASCENDING,  // 上升到high_state
    LIFT_PHASE_DESCENDING, // 下降到low_state
    LIFT_PHASE_BLOCKED,    // 故障锁定停机
    LIFT_PHASE_COMPLETE    // 已稳定到达目标
} LiftPhase;

typedef enum {
    LIFT_FAULT_NONE = 0,
    LIFT_FAULT_FEEDBACK_OFFLINE, // 位置传感器或M2006反馈离线
    LIFT_FAULT_MOTOR_STALL,      // 碰顶、卡死等堵转
    LIFT_FAULT_MOTOR_SPIN,       // 丝杆脱开、联轴器打滑等空转
    LIFT_FAULT_TOF_OCCLUDED      // OLD临时场地遮挡，可自动恢复
} LiftFault;

/* 两种机型都需要的故障状态；调试时先观察fault，再观察机型专用字段。 */
typedef struct {
    LiftFault fault;
    LiftPhase resume_phase;
} LiftProtectionCommon;

#if ROBOT_SELECT == OLD

typedef struct {
    LiftProtectionCommon common;

    /* TOF遮挡和按新帧驱动的空转检测。 */
    uint16_t startup_ticks;
    uint8_t occlusion_frames;
    uint8_t recovery_frames;
    uint8_t spin_window_frames;
    uint8_t spin_windows;
    uint32_t last_tof_update;
    uint32_t suspect_tof_count;

    /* 不依赖TOF的快速碰顶窗口。 */
    uint16_t fast_startup_ticks;
    uint16_t fast_window_ticks;
    uint16_t fast_effort_ticks;
    uint8_t fast_stall_windows;

    /* TOF帧窗、可信坐标和遮挡恢复基准。 */
    float window_motor_angle;
    float window_tof;
    float trusted_motor_angle;
    float trusted_tof;
    float recovery_motor_angle;
    float recovery_tof;
    float fast_window_motor_angle;

    /* 调试和机构比例标定量。 */
    float motor_delta;
    float tof_delta;
    float motor_progress;
    float tof_progress;
    float fast_motor_progress;
    float odometry_residual;
    float tof_per_motor_degree;
    bool ratio_valid;
} LiftProtection;

#else

typedef struct {
    LiftProtectionCommon common;

    uint16_t startup_ticks;
    uint16_t window_ticks;
    uint16_t effort_ticks;
    uint16_t encoder_samples;
    uint8_t stall_windows;
    uint8_t spin_windows;
    int8_t motor_direction;
    uint32_t last_encoder_update;

    float window_motor_angle;
    float window_encoder;
    float motor_delta;
    float encoder_delta;
    float motor_progress;
    float sensor_progress;
} LiftProtection;

#endif

#endif
