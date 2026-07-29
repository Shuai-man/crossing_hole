#include "PowerControlTask.h"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "NingCap.h"
#include "Referee.h"
#include "bsp_can.h"
#include "can_config.h"
#include "debug.h"

#define REFEREE_POWER_LIMIT_MAX 125.0f      /* 接受的裁判底盘功率上限最大值，单位 W */
#define REFEREE_POWER_LIMIT_MIN 35.0f       /* 接受的裁判底盘功率上限最小值，单位 W */
#define REFEREE_POWER_LIMIT_DEFAULT 60.0f   /* 尚未收到裁判数据时的默认功率，单位 W */
#define REFEREE_BUFFER_ENERGY_MAX 60.0f     /* 裁判底盘缓冲能量最大有效值，单位 J */
#define REFEREE_BUFFER_ENERGY_TARGET 30.0f  /* 缓冲能量闭环的目标值，单位 J */
#define REFEREE_BUFFER_POWER_KP 12.0f       /* 缓冲能量偏差转换成功率修正的比例系数 */
#define REFEREE_BUFFER_MAX_BONUS 20.0f      /* 无超电时缓冲能量最多额外提供的功率，单位 W */
#define REFEREE_BUFFER_MIN_POWER_COEF 0.75f /* 缓冲不足时最低功率占裁判上限的比例 */
#define REFEREE_OFFLINE_POWER_COEF 0.85f    /* 裁判掉线后使用最近有效上限的保守比例 */
#define BUFFER_ADJUST_RISE_ALPHA 0.01f      /* 功率上调滤波系数：小值使功率缓慢恢复 */
#define BUFFER_ADJUST_FALL_ALPHA 0.20f      /* 功率下调滤波系数：大值使功率快速收紧 */
#define SUPER_CAP_POWER_BONUS 100.0f         /* 超电正常且启用时允许增加的功率，单位 W */

ChassisPowerStatus chassis_power_status;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void ChassisPowerControllerInit(void)
{
    chassis_power_status.referee_power_limit = REFEREE_POWER_LIMIT_DEFAULT;
    chassis_power_status.referee_buffer_energy = 0.0f;
    chassis_power_status.buffer_power_adjustment =
        REFEREE_POWER_LIMIT_DEFAULT * (REFEREE_OFFLINE_POWER_COEF - 1.0f);
    chassis_power_status.buffer_limited_power =
        REFEREE_POWER_LIMIT_DEFAULT * REFEREE_OFFLINE_POWER_COEF;
    chassis_power_status.chassis_power_limit = chassis_power_status.buffer_limited_power;
    chassis_power_status.referee_online = 0U;
    chassis_power_status.super_cap_online = 0U;
    chassis_power_status.super_cap_active = 0U;
}

static void UpdateBufferPowerLoop(uint8_t referee_online)
{
    float target_adjustment;
    float minimum_adjustment;
    float filter_alpha;

    chassis_power_status.referee_online = referee_online;
    if (referee_online == 0U)
    {
        chassis_power_status.referee_buffer_energy = 0.0f;
        chassis_power_status.buffer_power_adjustment =
            chassis_power_status.referee_power_limit * (REFEREE_OFFLINE_POWER_COEF - 1.0f);
        chassis_power_status.buffer_limited_power =
            chassis_power_status.referee_power_limit * REFEREE_OFFLINE_POWER_COEF;
        return;
    }

    chassis_power_status.referee_power_limit =
        clamp_float((float)Robot_Status.chassis_power_limit, REFEREE_POWER_LIMIT_MIN, REFEREE_POWER_LIMIT_MAX);
    chassis_power_status.referee_buffer_energy =
        clamp_float((float)Power_Heat_Data.buffer_energy, 0.0f, REFEREE_BUFFER_ENERGY_MAX);

    target_adjustment =
        REFEREE_BUFFER_POWER_KP *
        (sqrtf(chassis_power_status.referee_buffer_energy) - sqrtf(REFEREE_BUFFER_ENERGY_TARGET));
    minimum_adjustment =
        chassis_power_status.referee_power_limit * (REFEREE_BUFFER_MIN_POWER_COEF - 1.0f);
    target_adjustment =
        clamp_float(target_adjustment, minimum_adjustment, REFEREE_BUFFER_MAX_BONUS);

    /*
     * 缓冲下降时快速收功率，缓冲恢复时缓慢放开，避免裁判数据阶跃
     * 导致底盘功率上限突然上升。
     */
    filter_alpha =
        target_adjustment < chassis_power_status.buffer_power_adjustment
            ? BUFFER_ADJUST_FALL_ALPHA
            : BUFFER_ADJUST_RISE_ALPHA;
    chassis_power_status.buffer_power_adjustment +=
        filter_alpha * (target_adjustment - chassis_power_status.buffer_power_adjustment);

    chassis_power_status.buffer_limited_power =
        clamp_float(chassis_power_status.referee_power_limit + chassis_power_status.buffer_power_adjustment,
                    chassis_power_status.referee_power_limit * REFEREE_BUFFER_MIN_POWER_COEF,
                    chassis_power_status.referee_power_limit + REFEREE_BUFFER_MAX_BONUS);
}

float GetChassisPowerLimit(uint8_t super_cap_active)
{
    float power_limit;

    chassis_power_status.super_cap_active = super_cap_active;
    if (super_cap_active != 0U)
    {
        /*
         * 超电提供正向额外功率；裁判缓冲只允许降低该上限，避免两个
         * 能量源同时给出正向奖励。
         */
        power_limit =
            chassis_power_status.referee_power_limit +
            SUPER_CAP_POWER_BONUS +
            (chassis_power_status.buffer_power_adjustment < 0.0f
                 ? chassis_power_status.buffer_power_adjustment
                 : 0.0f);
    }
    else
    {
        power_limit = chassis_power_status.buffer_limited_power;
    }

    chassis_power_status.chassis_power_limit =
        clamp_float(power_limit,
                    REFEREE_POWER_LIMIT_MIN * REFEREE_BUFFER_MIN_POWER_COEF,
                    REFEREE_POWER_LIMIT_MAX + SUPER_CAP_POWER_BONUS);
    return chassis_power_status.chassis_power_limit;
}

void PowerControlTask(void const *argument)
{
    portTickType last_wake_time;
    uint8_t send_divider = 0U;
    uint8_t referee_online;
    uint8_t super_cap_online;

    (void)argument;
    CapControllerInit();
    ChassisPowerControllerInit();

    last_wake_time = xTaskGetTickCount();
    while (1)
    {
        referee_online =
            (uint8_t)(global_debugger.referee_debugger.state == ON);
        super_cap_online =
            (uint8_t)(global_debugger.super_power_debugger.state == ON);

        UpdateBufferPowerLoop(referee_online);
        NingCapUpdateState(super_cap_online);
        chassis_power_status.super_cap_online = super_cap_online;

        if (send_divider == 0U)
        {
            /* 超电始终只接收裁判系统的基础功率上限。 */
            SendCapPack(&cap_send_data, chassis_power_status.referee_power_limit);
            CanSend(SUPER_POWER_CAN, (uint8_t *)&cap_send_data, SEND_TO_SUPER_POWER_CAN_ID, sizeof(cap_send_data));
        }

        send_divider = (uint8_t)((send_divider + 1U) % 4U);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
