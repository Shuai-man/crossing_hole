#ifndef _LIFTING_CONTROL_H
#define _LIFTING_CONTROL_H

#include "remote_control.h"
#include "tof.h"
#include "pid.h"
#include "M2006.h"
#include <stdint.h>
#include "debug.h"

#define GIMBAL_HIGH_STATE 177 //此处修改最高处的设定值（单位:mm）
#define GIMBAL_LOW_STATE 30
#define LIFTING_SPEED_MAX 2700.0f
#define LIFT_DIR -1 //向上为正

#define POSE_THRESHOLD 7.0f //位置误差阈值
#define CURRENT_THRESHOLD 0.17f*C610_MAX_CURRENT //电流误差阈值
#define FINISH_CNT_LIMIT 600 //结束计数阈值

#define OBSTACLE_ERR_THRESHOLD   20.0f   // mm, 误差大于此才启用阻塞检测
#define OBSTACLE_DELTA_THRESHOLD 8.0f    // mm, 距离下降超过此值才触发阻塞
#define SETTLE_TICKS             200     // 下降→上升方向稳定周期数

#define STALL_CURRENT_THRESHOLD  0.5f*C610_MAX_CURRENT  // 堵转：电流>=此值 (占C610_MAX_CURRENT比例)
#define STALL_ANGLE_DELTA_THRESHOLD  3.0f               // 堵转：累计角度变化<=此值（输出轴度）
#define STALL_CHECK_CYCLES       150                      // 堵转确认帧数（含起步等待）

#define SPIN_ANGLE_RANGE     800.0f   // 空转：角度累计变化超过此值（输出轴度），后续换电机可以把阈值调小，正常不应该空转那么多圈
#define SPIN_TOF_RANGE       2.0f     // 空转：TOF累计变化小于此值（mm）
#define SPIN_CHECK_CYCLES    3        // 空转确认帧数

typedef enum {
    LIFT_PHASE_IDLE = 0,        // 待机
    LIFT_PHASE_ASCENDING,       // 上升中
    LIFT_PHASE_DESCENDING,      // 下降中
    LIFT_PHASE_BLOCKED,         // 上升阻塞
    LIFT_PHASE_COMPLETE         // 到达目标
} LiftPhase;

typedef struct LiftingController
{
	uint8_t high_state;
	uint8_t low_state;
	LiftPhase phase;          // 当前执行阶段
	uint8_t settle_ticks;     // 方向稳定计数器（仅上升使用）
	uint16_t finish_cnt;       // 到达判定计数
	uint16_t stall_counter;    // 堵转确认计数器（仅升降过程中使用）
	float angle_reference;     // 堵转检测的角度参考点（输出轴度）
	float tof_reference;       // TOF距离参考点（用于空转/堵转统合判断）

	uint16_t recovery_ref;    // 阻塞前记录的TOF距离，用作恢复判据
	uint16_t last_distance;   // 上一帧TOF距离（用于delta检测）
	bool error;               // 阻塞标志（用于外部查询）

  PID_t lift_pos_pid;
  PID_t lift_speed_pid; //丝杆速度环
  M2006_Recv lift_recv; //原始数据
  M2006_Info lift_info; //解码数据: 度/秒

  float set_pos;
  float set_speed;

  float send_current;     // 发送电流值

} LiftingController;

extern LiftingController lifting_controller;
void LiftPidInit(void);
void Lifting_Control(void);
#endif
