#ifndef LIFTING_CONFIG_H
#define LIFTING_CONFIG_H

#include "M2006.h"

/*
 * 升降控制任务固定以1kHz运行，因此下列所有_TICKS参数均可直接理解为毫秒。
 * 修改任务周期时必须同步换算这些参数。
 */

/* ---------- 位置目标与串级PID ---------- */

#define LIFT_TOF_HIGH_POINT_MM             170.0f // 老云台最高点TOF距离
#define LIFT_TOF_LOW_POINT_MM               30.0f // 老云台最低点TOF距离
#define LIFT_TOF_SENSOR_TO_MOTOR_DIR        -1.0f // TOF位置环输出到电机速度的方向

#define LIFT_ENCODER_HIGH_POINT_COUNT      500.0f // 新云台最高点绝对编码器值
#define LIFT_ENCODER_LOW_POINT_COUNT     10586.0f // 新云台最低点绝对编码器实车标定值
#define LIFT_ENCODER_SENSOR_TO_MOTOR_DIR     1.0f // 若实车朝目标反向运动则改为-1.0f

#define LIFT_SPEED_REF_MAX_DEG_S          2700.0f // 位置环允许输出的最大速度目标
#define LIFT_TOF_POSITION_KP                25.0f // TOF位置输入单位为mm
#define LIFT_ENCODER_POSITION_KP             0.25f // 编码器量程约为TOF的100倍
#define LIFT_SPEED_KP                        3.0f // 两种机型速度输入均为M2006 deg/s
#define LIFT_SPEED_KD                        0.015f

/* ---------- 到位判定 ---------- */

#define LIFT_TOF_FINISH_POSITION_MM           7.0f // 老云台允许的TOF误差
#define LIFT_ENCODER_FINISH_POSITION_COUNT  700.0f // 新云台允许的编码器误差
#define LIFT_FINISH_SPEED_DEG_S                5.0f // 实际输出轴速度门槛
#define LIFT_TOF_FINISH_TICKS                600U  // 老云台稳定600ms后确认到位
#define LIFT_ENCODER_FINISH_TICKS            200U  // 新云台稳定200ms后确认到位

/* 仅OLD：下降中直接切换上升时，暂停TOF判断；快速堵转仍保持工作。 */
#define LIFT_TOF_DIRECTION_SETTLE_TICKS      200U

/* ---------- OLD：不依赖TOF的快速碰顶保护 ---------- */

#define LIFT_TOF_FAST_STALL_STARTUP_TICKS       50U   // 启动仅等待50ms建立驱动
#define LIFT_TOF_FAST_STALL_WINDOW_TICKS       100U   // 每100ms统计电流和M2006里程
#define LIFT_TOF_FAST_STALL_CONFIRM_WINDOWS      2U   // 连续两窗确认，最快约250ms停机
#define LIFT_TOF_FAST_STALL_EFFORT_PERCENT       60U   // 高负载周期至少占窗口60%
#define LIFT_TOF_FAST_STALL_CURRENT_MIN (0.35f * C610_MAX_CURRENT) // 指令或反馈电流达到3.15A
#define LIFT_TOF_FAST_STALL_MOTOR_MAX_DEG        30.0f // 100ms内前进不超过30deg

/* ---------- OLD：按真实TOF新帧驱动的遮挡、空转和里程保护 ---------- */

#define LIFT_TOF_PROTECT_TARGET_GUARD_MM          8.0f // 距目标8mm内暂停TOF故障判断
#define LIFT_TOF_PROTECT_STARTUP_TICKS           300U  // 动作开始300ms后启用TOF保护
#define LIFT_TOF_SPIN_WINDOW_FRAMES                2U   // 每2帧可信TOF评估一次空转
#define LIFT_TOF_SPIN_CONFIRM_WINDOWS              2U   // 连续2个异常帧窗确认空转
#define LIFT_TOF_SPIN_MOTOR_MIN_DEG               500.0f // 帧窗内电机至少前进500deg
#define LIFT_TOF_STABLE_MM                          4.0f // 帧窗内TOF变化不超过4mm
#define LIFT_TOF_ODOM_MIN_MOTOR_DEG                 5.0f // 里程比例学习所需的最小电机变化

#define LIFT_TOF_OCCLUSION_DROP_MM                  8.0f // 上升时单帧突然减小至少8mm
#define LIFT_TOF_ODOM_RESIDUAL_MM                   8.0f // 实测与里程预测残差门槛
#define LIFT_TOF_OCCLUSION_CONFIRM_FRAMES           2U   // 连续2帧疑似异常才确认遮挡
#define LIFT_TOF_RECOVERY_TOLERANCE_MM              8.0f // 遮挡恢复允许的预测误差
#define LIFT_TOF_RECOVERY_FRAMES                    3U   // 连续3帧可信后自动续跑

#define LIFT_TOF_ODOM_RATIO_MIN                     0.01f // 合理机构比例下限，单位mm/deg
#define LIFT_TOF_ODOM_RATIO_MAX                   100.0f // 合理机构比例上限
#define LIFT_TOF_ODOM_RATIO_FILTER                  0.15f // 比例低通更新系数

/* ---------- NEW：编码器与M2006联合保护 ---------- */

#define LIFT_ENCODER_PROTECT_STARTUP_TICKS          50U   // 启动等待50ms
#define LIFT_ENCODER_PROTECT_WINDOW_TICKS          100U   // 每100ms统计一次
#define LIFT_ENCODER_PROTECT_CONFIRM_WINDOWS         2U   // 连续2窗确认
#define LIFT_ENCODER_PROTECT_MIN_SAMPLES              2U   // 每窗至少2帧新编码器数据
#define LIFT_ENCODER_PROTECT_EFFORT_PERCENT           60U   // 高负载周期占比门槛
#define LIFT_ENCODER_PROTECT_CURRENT_MIN (0.35f * C610_MAX_CURRENT) // 3.15A
#define LIFT_ENCODER_MOTOR_SPIN_DEG                  30.0f // 编码器不动而电机前进30deg
#define LIFT_ENCODER_STABLE_COUNT                    50.0f // 100ms内前进不超过50count

#endif
