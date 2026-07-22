#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H

#include <stdint.h>

#include "Motor_Typdef.h"

#define POWER_LIMIT_MOTOR_COUNT 4U /* 参与底盘功率分配的轮电机数量 */

/* P = R * I^2 + K * w * I + B * |w| + P0 */

#define M3508_R 0.124462309f /* M3508 电流平方损耗系数 */
#define M3508_K 0.3f /* M3508 转速与电流乘积系数 */
#define M3508_B 0.1004156735f /* M3508 随绝对转速变化的损耗系数 */
#define M3508_P0 2.4f/4.0f /* M3508 单电机固定损耗，单位 W */

typedef enum
{
    POWER_COEFF_CURRENT_SQUARED = 0,
    POWER_COEFF_ABS_SPEED,
    POWER_COEFF_SPEED_CURRENT,
    POWER_COEFF_STATIC_LOSS,
    POWER_COEFF_COUNT
} PowerModelCoefficient;

typedef enum
{
    NOT_ARRANGE = 0,
    NEG_ARRANGE,
    NEED_ARRANGE,
    NORMAL_ARRANGE,
    ARRANGE_ERROR
} Power_Arrange;

typedef enum
{
    TORQUE_REDUCE_METHOD = 0,
    SPEED_ERROR_METHOD
} PowerLimitMethod;

typedef struct
{
    /* 影子 RLS 仅用于识别参数，不会覆盖当前功控系数。 */
    float identified_R;
    float identified_B;
    float measured_power;
    float predicted_power;
    float residual;
    float current_squared_sum;
    float speed_current_sum;
    float absolute_speed_sum;
    float covariance[2][2];
    float window_current_squared_sum;
    float window_absolute_speed_sum;
    float window_loss_power_sum;
    uint32_t last_measurement_sequence;
    uint32_t update_count;
    uint32_t rejected_sample_count;
    uint8_t window_sample_count;
    uint8_t latest_sample_valid;
    uint8_t enabled;
} PowerModelRLS;

typedef struct
{
    /* 每次调用前由底盘控制器更新，单位分别为 rad/s 和 A。 */
    float motor_w[POWER_LIMIT_MOTOR_COUNT];
    float motor_I[POWER_LIMIT_MOTOR_COUNT];
    float I_collect[POWER_LIMIT_MOTOR_COUNT];
    float motor_w_error[POWER_LIMIT_MOTOR_COUNT];
    uint8_t motor_online[POWER_LIMIT_MOTOR_COUNT];

    /* 输出和诊断信息。 */
    float motor_P[POWER_LIMIT_MOTOR_COUNT];
    float allocated_power[POWER_LIMIT_MOTOR_COUNT];
    float send_torque_lower_scale[POWER_LIMIT_MOTOR_COUNT];
    Power_Arrange power_arrange_state[POWER_LIMIT_MOTOR_COUNT];
    float predict_send_power;
    float now_power_predict;
    float set_power;

    float power_k[POWER_COEFF_COUNT];
    PowerModelRLS rls;
    uint8_t motor_num;
    MOTOR_TYPE motor_type;
    PowerLimitMethod power_limit_method;

} PowerLimiter;

void PowerLimitInit(PowerLimiter *limiter,
                    uint8_t motor_num,
                    MOTOR_TYPE motor_type,
                    PowerLimitMethod method);

void PowerLimit(PowerLimiter *limiter, float set_power);

void PowerModelRLSUpdate(PowerLimiter *limiter,
                         float measured_chassis_power,
                         uint32_t measurement_sequence,
                         uint8_t measurement_valid);

#endif
