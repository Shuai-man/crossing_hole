#ifndef LIFTING_CONTROL_H
#define LIFTING_CONTROL_H

#include <stdint.h>
#include "lifting_types.h"
#include "pid.h"
#include "M2006.h"

/*
 * 升降主控制器。
 * phase和protection.common.fault描述状态；PID的Ref/Err和专用protection字段用于调试。
 */
typedef struct {
    float high_state; // 当前机型最高点目标：OLD单位mm，NEW单位count
    float low_state;  // 当前机型最低点目标：OLD单位mm，NEW单位count
    float sensor_dir; // 位置环输出到电机速度目标的方向

    LiftPhase phase;
    uint16_t settle_ticks;
    uint16_t finish_cnt;
    LiftProtection protection;

    PID_t lift_pos_pid;
    PID_t lift_speed_pid;
    M2006_Recv lift_recv;
    M2006_Info lift_info;
    float send_current; // PID输出并发送给C610的电流，单位A
} LiftingController;

extern LiftingController lifting_controller;

/* 云台任务启动时调用一次，初始化目标、PID和保护状态。 */
void LiftPidInit(void);

/* 1kHz周期入口：处理遥控需求、状态机、PID、到位和故障保护。 */
void Lifting_Control(void);

#endif
