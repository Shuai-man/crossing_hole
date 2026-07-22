#ifndef POWER_CONTROL_TASK_H
#define POWER_CONTROL_TASK_H

#include <stdint.h>

typedef struct
{
    float referee_power_limit;
    float referee_buffer_energy;
    float buffer_power_adjustment;
    float buffer_limited_power;
    float chassis_power_limit;
    uint8_t referee_online;
    uint8_t super_cap_online;
    uint8_t super_cap_active;
} ChassisPowerStatus;

extern ChassisPowerStatus chassis_power_status;

void PowerControlTask(void const *argument);

/**
 * @brief 返回考虑裁判缓冲能量和超电状态后的底盘功率上限。
 *
 * 超电未启用时，允许利用裁判缓冲能量短时提高功率；超电启用时，
 * 缓冲能量只做向下保护，避免正向奖励与超电功率重复叠加。
 */
float GetChassisPowerLimit(uint8_t super_cap_active);

#endif
