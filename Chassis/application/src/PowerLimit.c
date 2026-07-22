#include "PowerLimit.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define POWER_EPSILON 1.0e-6f            /* 浮点零值和分母保护阈值 */
#define POWER_ERROR_PROP_THRESHOLD 15.0f /* 低于该总转速误差时按预测功率比例分配 */
#define POWER_ERROR_FULL_THRESHOLD 20.0f /* 高于该总转速误差时按转速误差优先分配 */

#define POWER_RLS_WINDOW_SAMPLES 8U          /* 每 8 帧新超电数据更新一次 RLS */
#define POWER_RLS_FORGETTING_FACTOR 0.9995f  /* 遗忘因子，靠近 1 使参数缓慢收敛 */
#define POWER_RLS_INITIAL_COVARIANCE 10.0f   /* 初始参数不确定度 */
#define POWER_RLS_I2_SCALE 100.0f            /* 电流平方特征归一化尺度 */
#define POWER_RLS_SPEED_SCALE 100.0f         /* 绝对转速特征归一化尺度 */
#define POWER_RLS_MIN_MEASURED_POWER 5.0f    /* 低功率和接近零的回馈钳位区不拟合 */
#define POWER_RLS_MAX_RESIDUAL_MIN 20.0f     /* 稳定后单窗口允许的最小残差阈值 */
#define POWER_RLS_BRAKE_POWER_TOLERANCE 2.0f /* 负于此值的单电机制动功率不参与首轮拟合 */
#define POWER_RLS_R_MIN 0.0001f
#define POWER_RLS_R_MAX 2.0f
#define POWER_RLS_B_MIN 0.00001f
#define POWER_RLS_B_MAX 1.0f

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

static uint8_t float_is_finite(float value)
{
    return (uint8_t)((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX));
}

static void reset_power_model_rls(PowerLimiter *limiter)
{
    memset(&limiter->rls, 0, sizeof(limiter->rls));
    limiter->rls.identified_R =
        limiter->power_k[POWER_COEFF_CURRENT_SQUARED];
    limiter->rls.identified_B = limiter->power_k[POWER_COEFF_ABS_SPEED];
    limiter->rls.covariance[0][0] = POWER_RLS_INITIAL_COVARIANCE;
    limiter->rls.covariance[1][1] = POWER_RLS_INITIAL_COVARIANCE;
    limiter->rls.enabled = 1U;
}

static float predict_motor_power(const PowerLimiter *limiter,
                                 uint8_t motor_index,
                                 float current)
{
    const float speed = limiter->motor_w[motor_index];

    return limiter->power_k[POWER_COEFF_CURRENT_SQUARED] * current * current +
           limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed * current +
           limiter->power_k[POWER_COEFF_ABS_SPEED] * fabsf(speed) +
           limiter->power_k[POWER_COEFF_STATIC_LOSS];
}

static float solve_current_scale(const PowerLimiter *limiter,
                                 uint8_t motor_index,
                                 float allocated_power)
{
    const float command_current = limiter->motor_I[motor_index];
    const float speed = limiter->motor_w[motor_index];
    const float a = limiter->power_k[POWER_COEFF_CURRENT_SQUARED];
    const float b = limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed;
    const float c = limiter->power_k[POWER_COEFF_ABS_SPEED] * fabsf(speed) +
                    limiter->power_k[POWER_COEFF_STATIC_LOSS] -
                    allocated_power;
    float discriminant;
    float limited_current;
    float limited_power;
    float scale;

    if (fabsf(command_current) <= POWER_EPSILON)
    {
        return 1.0f;
    }

    if (fabsf(a) <= POWER_EPSILON)
    {
        if (fabsf(b) <= POWER_EPSILON)
        {
            return 0.0f;
        }
        limited_current = -c / b;
    }
    else
    {
        discriminant = b * b - 4.0f * a * c;
        if (!float_is_finite(discriminant))
        {
            return 0.0f;
        }

        if (discriminant < 0.0f)
        {
            /* 无实根时使用抛物线顶点；随后仍会限制到原指令区间。 */
            limited_current = -b / (2.0f * a);
        }
        else if (command_current >= 0.0f)
        {
            limited_current = (-b + sqrtf(discriminant)) / (2.0f * a);
        }
        else
        {
            limited_current = (-b - sqrtf(discriminant)) / (2.0f * a);
        }
    }

    if (!float_is_finite(limited_current))
    {
        return 0.0f;
    }

    if (command_current > 0.0f)
    {
        limited_current = clamp_float(limited_current, 0.0f, command_current);
    }
    else
    {
        limited_current = clamp_float(limited_current, command_current, 0.0f);
    }

    limited_power = predict_motor_power(limiter, motor_index, limited_current);
    if (!float_is_finite(limited_power))
    {
        return 0.0f;
    }

    if (limited_power > allocated_power + POWER_EPSILON)
    {
        float best_current = 0.0f;
        float best_power = predict_motor_power(limiter, motor_index, 0.0f);
        const float command_power =
            predict_motor_power(limiter, motor_index, command_current);

        /*
         * 命令区间内可能不存在满足分配功率的实根。例如两个根都在
         * 原命令同一侧。此时选择区间内预测功率最低的电流，而不是
         * 保留一个仍然超出分配值的原命令。
         */
        if (float_is_finite(command_power) && command_power < best_power)
        {
            best_current = command_current;
            best_power = command_power;
        }

        if (fabsf(a) > POWER_EPSILON)
        {
            const float vertex_current = -b / (2.0f * a);
            float clamped_vertex;
            float vertex_power;

            if (command_current > 0.0f)
            {
                clamped_vertex = clamp_float(vertex_current, 0.0f, command_current);
            }
            else
            {
                clamped_vertex = clamp_float(vertex_current, command_current, 0.0f);
            }
            vertex_power =
                predict_motor_power(limiter, motor_index, clamped_vertex);
            if (float_is_finite(vertex_power) && vertex_power < best_power)
            {
                best_current = clamped_vertex;
                best_power = vertex_power;
            }
        }

        limited_current = best_current;
    }

    scale = limited_current / command_current;
    return float_is_finite(scale) ? clamp_float(scale, 0.0f, 1.0f) : 0.0f;
}

static void update_feedback_power_prediction(PowerLimiter *limiter)
{
    float current_squared_sum = 0.0f;
    float absolute_speed_sum = 0.0f;
    float speed_current_sum = 0.0f;
    uint8_t online_motor_count = 0U;
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < limiter->motor_num; motor_index++)
    {
        const float current = limiter->I_collect[motor_index];
        const float speed = limiter->motor_w[motor_index];

        if (limiter->motor_online[motor_index] == 0U ||
            !float_is_finite(current) ||
            !float_is_finite(speed))
        {
            continue;
        }

        current_squared_sum += current * current;
        absolute_speed_sum += fabsf(speed);
        speed_current_sum += speed * current;
        online_motor_count++;
    }

    limiter->now_power_predict =
        limiter->power_k[POWER_COEFF_CURRENT_SQUARED] * current_squared_sum +
        limiter->power_k[POWER_COEFF_ABS_SPEED] * absolute_speed_sum +
        limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed_current_sum +
        limiter->power_k[POWER_COEFF_STATIC_LOSS] * online_motor_count;
}

void PowerLimitInit(PowerLimiter *limiter,
                    uint8_t motor_num,
                    MOTOR_TYPE motor_type,
                    PowerLimitMethod method)
{
    if (limiter == 0)
    {
        return;
    }

    memset(limiter, 0, sizeof(*limiter));
    limiter->motor_num =
        (uint8_t)clamp_float((float)motor_num, 1.0f, (float)POWER_LIMIT_MOTOR_COUNT);
    limiter->motor_type = motor_type;
    limiter->power_limit_method = method;

    limiter->power_k[POWER_COEFF_CURRENT_SQUARED] = M3508_R;
    limiter->power_k[POWER_COEFF_ABS_SPEED] = M3508_B;
    limiter->power_k[POWER_COEFF_SPEED_CURRENT] = M3508_K;
    limiter->power_k[POWER_COEFF_STATIC_LOSS] = M3508_P0;

    reset_power_model_rls(limiter);
}

void PowerModelRLSUpdate(PowerLimiter *limiter,
                         float measured_chassis_power,
                         uint32_t measurement_sequence,
                         uint8_t measurement_valid)
{
    /* 使用超电输出侧实测功率进行两参数影子识别。 */
    float current_squared_sum = 0.0f;
    float absolute_speed_sum = 0.0f;
    float speed_current_sum = 0.0f;
    float static_power;
    float predicted_power;
    float loss_power;
    float residual_limit;
    uint8_t online_motor_count = 0U;
    uint8_t motor_index;

    if (limiter == 0 || limiter->rls.enabled == 0U)
    {
        return;
    }

    if (measurement_valid == 0U)
    {
        limiter->rls.latest_sample_valid = 0U;
        return;
    }
    if (measurement_sequence == limiter->rls.last_measurement_sequence)
    {
        return;
    }
    limiter->rls.latest_sample_valid = 0U;
    limiter->rls.last_measurement_sequence = measurement_sequence;

    for (motor_index = 0U; motor_index < limiter->motor_num; motor_index++)
    {
        const float current = limiter->I_collect[motor_index];
        const float speed = limiter->motor_w[motor_index];
        float motor_effective_power;

        if (limiter->motor_online[motor_index] == 0U ||
            !float_is_finite(current) ||
            !float_is_finite(speed))
        {
            limiter->rls.rejected_sample_count++;
            return;
        }

        motor_effective_power =
            limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed * current;
        if (motor_effective_power < -POWER_RLS_BRAKE_POWER_TOLERANCE)
        {
            limiter->rls.rejected_sample_count++;
            return;
        }

        current_squared_sum += current * current;
        absolute_speed_sum += fabsf(speed);
        speed_current_sum += speed * current;
        online_motor_count++;
    }

    static_power =
        limiter->power_k[POWER_COEFF_STATIC_LOSS] * online_motor_count;
    predicted_power =
        limiter->rls.identified_R * current_squared_sum +
        limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed_current_sum +
        limiter->rls.identified_B * absolute_speed_sum + static_power;

    limiter->rls.measured_power = measured_chassis_power;
    limiter->rls.predicted_power = predicted_power;
    limiter->rls.residual = measured_chassis_power - predicted_power;
    limiter->rls.current_squared_sum = current_squared_sum;
    limiter->rls.speed_current_sum = speed_current_sum;
    limiter->rls.absolute_speed_sum = absolute_speed_sum;

    if (!float_is_finite(measured_chassis_power) ||
        measured_chassis_power < POWER_RLS_MIN_MEASURED_POWER)
    {
        limiter->rls.rejected_sample_count++;
        return;
    }

    loss_power =
        measured_chassis_power -
        limiter->power_k[POWER_COEFF_SPEED_CURRENT] * speed_current_sum -
        static_power;
    if (!float_is_finite(loss_power) || loss_power <= 0.0f ||
        (current_squared_sum <= POWER_EPSILON &&
         absolute_speed_sum <= POWER_EPSILON))
    {
        limiter->rls.rejected_sample_count++;
        return;
    }

    residual_limit = fmaxf(POWER_RLS_MAX_RESIDUAL_MIN,
                           measured_chassis_power * 0.5f);
    if (limiter->rls.update_count >= 10U &&
        fabsf(limiter->rls.residual) > residual_limit)
    {
        limiter->rls.rejected_sample_count++;
        return;
    }

    limiter->rls.latest_sample_valid = 1U;
    limiter->rls.window_current_squared_sum += current_squared_sum;
    limiter->rls.window_absolute_speed_sum += absolute_speed_sum;
    limiter->rls.window_loss_power_sum += loss_power;
    limiter->rls.window_sample_count++;

    if (limiter->rls.window_sample_count >= POWER_RLS_WINDOW_SAMPLES)
    {
        const float sample_count = (float)limiter->rls.window_sample_count;
        const float phi0 =
            limiter->rls.window_current_squared_sum /
            sample_count / POWER_RLS_I2_SCALE;
        const float phi1 =
            limiter->rls.window_absolute_speed_sum /
            sample_count / POWER_RLS_SPEED_SCALE;
        const float output = limiter->rls.window_loss_power_sum / sample_count;
        const float p_phi0 = limiter->rls.covariance[0][0] * phi0 +
                             limiter->rls.covariance[0][1] * phi1;
        const float p_phi1 = limiter->rls.covariance[1][0] * phi0 +
                             limiter->rls.covariance[1][1] * phi1;
        const float denominator = POWER_RLS_FORGETTING_FACTOR +
                                  phi0 * p_phi0 + phi1 * p_phi1;

        if (float_is_finite(denominator) && denominator > POWER_EPSILON)
        {
            const float gain0 = p_phi0 / denominator;
            const float gain1 = p_phi1 / denominator;
            const float theta0 = limiter->rls.identified_R * POWER_RLS_I2_SCALE;
            const float theta1 = limiter->rls.identified_B * POWER_RLS_SPEED_SCALE;
            const float error = output - theta0 * phi0 - theta1 * phi1;
            float new_theta0 = theta0 + gain0 * error;
            float new_theta1 = theta1 + gain1 * error;
            float new_p00 = (limiter->rls.covariance[0][0] -
                             gain0 * p_phi0) /
                            POWER_RLS_FORGETTING_FACTOR;
            float new_p01 = (limiter->rls.covariance[0][1] -
                             gain0 * p_phi1) /
                            POWER_RLS_FORGETTING_FACTOR;
            float new_p10 = (limiter->rls.covariance[1][0] -
                             gain1 * p_phi0) /
                            POWER_RLS_FORGETTING_FACTOR;
            float new_p11 = (limiter->rls.covariance[1][1] -
                             gain1 * p_phi1) /
                            POWER_RLS_FORGETTING_FACTOR;
            float symmetric_off_diagonal;

            if (float_is_finite(new_theta0) && float_is_finite(new_theta1) &&
                float_is_finite(new_p00) && float_is_finite(new_p01) &&
                float_is_finite(new_p10) && float_is_finite(new_p11))
            {
                limiter->rls.identified_R =
                    clamp_float(new_theta0 / POWER_RLS_I2_SCALE,
                                POWER_RLS_R_MIN,
                                POWER_RLS_R_MAX);
                limiter->rls.identified_B =
                    clamp_float(new_theta1 / POWER_RLS_SPEED_SCALE,
                                POWER_RLS_B_MIN,
                                POWER_RLS_B_MAX);

                symmetric_off_diagonal = 0.5f * (new_p01 + new_p10);
                limiter->rls.covariance[0][0] =
                    clamp_float(new_p00, POWER_EPSILON, FLT_MAX);
                limiter->rls.covariance[0][1] = symmetric_off_diagonal;
                limiter->rls.covariance[1][0] = symmetric_off_diagonal;
                limiter->rls.covariance[1][1] =
                    clamp_float(new_p11, POWER_EPSILON, FLT_MAX);
                limiter->rls.update_count++;
            }
            else
            {
                limiter->rls.rejected_sample_count++;
            }
        }
        else
        {
            limiter->rls.rejected_sample_count++;
        }

        limiter->rls.window_current_squared_sum = 0.0f;
        limiter->rls.window_absolute_speed_sum = 0.0f;
        limiter->rls.window_loss_power_sum = 0.0f;
        limiter->rls.window_sample_count = 0U;
    }
}

void PowerLimit(PowerLimiter *limiter, float set_power)
{
    float allocatable_power;
    float positive_power_sum = 0.0f;
    float speed_error_sum = 0.0f;
    float error_confidence = 0.0f;
    uint8_t motor_index;

    if (limiter == 0)
    {
        return;
    }

    if (!float_is_finite(set_power) || set_power < 0.0f)
    {
        set_power = 0.0f;
    }

    limiter->set_power = set_power;
    limiter->predict_send_power = 0.0f;
    allocatable_power = set_power;

    for (motor_index = 0U; motor_index < limiter->motor_num; motor_index++)
    {
        limiter->allocated_power[motor_index] = 0.0f;
        limiter->send_torque_lower_scale[motor_index] = 0.0f;
        limiter->power_arrange_state[motor_index] = NOT_ARRANGE;

        if (limiter->motor_online[motor_index] == 0U)
        {
            limiter->motor_P[motor_index] = 0.0f;
            continue;
        }

        limiter->motor_P[motor_index] =
            predict_motor_power(limiter, motor_index, limiter->motor_I[motor_index]);

        if (!float_is_finite(limiter->motor_P[motor_index]))
        {
            limiter->motor_P[motor_index] = 0.0f;
            limiter->power_arrange_state[motor_index] = ARRANGE_ERROR;
            continue;
        }

        limiter->predict_send_power += limiter->motor_P[motor_index];
        if (limiter->motor_P[motor_index] <= POWER_EPSILON)
        {
            allocatable_power -= limiter->motor_P[motor_index];
            limiter->power_arrange_state[motor_index] = NEG_ARRANGE;
        }
        else
        {
            positive_power_sum += limiter->motor_P[motor_index];
            speed_error_sum += fabsf(limiter->motor_w_error[motor_index]);
            limiter->power_arrange_state[motor_index] = NEED_ARRANGE;
        }
    }

    if (limiter->predict_send_power <= set_power)
    {
        for (motor_index = 0U; motor_index < limiter->motor_num; motor_index++)
        {
            if (limiter->motor_online[motor_index] != 0U &&
                limiter->power_arrange_state[motor_index] != ARRANGE_ERROR)
            {
                limiter->send_torque_lower_scale[motor_index] = 1.0f;
                limiter->allocated_power[motor_index] = limiter->motor_P[motor_index];
                limiter->power_arrange_state[motor_index] = NORMAL_ARRANGE;
            }
        }
        update_feedback_power_prediction(limiter);
        return;
    }

    allocatable_power = clamp_float(allocatable_power, 0.0f, FLT_MAX);
    if (limiter->power_limit_method == SPEED_ERROR_METHOD)
    {
        error_confidence =
            clamp_float((speed_error_sum - POWER_ERROR_PROP_THRESHOLD) /
                            (POWER_ERROR_FULL_THRESHOLD - POWER_ERROR_PROP_THRESHOLD),
                        0.0f,
                        1.0f);
    }

    for (motor_index = 0U; motor_index < limiter->motor_num; motor_index++)
    {
        float proportional_weight;
        float error_weight;
        float mixed_weight;

        if (limiter->motor_online[motor_index] == 0U)
        {
            continue;
        }

        if (limiter->power_arrange_state[motor_index] == NEG_ARRANGE)
        {
            limiter->allocated_power[motor_index] = limiter->motor_P[motor_index];
            limiter->send_torque_lower_scale[motor_index] = 1.0f;
            continue;
        }
        if (limiter->power_arrange_state[motor_index] != NEED_ARRANGE ||
            positive_power_sum <= POWER_EPSILON)
        {
            limiter->power_arrange_state[motor_index] = ARRANGE_ERROR;
            limiter->send_torque_lower_scale[motor_index] = 0.0f;
            continue;
        }

        proportional_weight =
            limiter->motor_P[motor_index] / positive_power_sum;
        error_weight = speed_error_sum > POWER_EPSILON
                           ? fabsf(limiter->motor_w_error[motor_index]) / speed_error_sum
                           : proportional_weight;
        mixed_weight = error_confidence * error_weight +
                       (1.0f - error_confidence) * proportional_weight;

        limiter->allocated_power[motor_index] =
            mixed_weight * allocatable_power;
        limiter->send_torque_lower_scale[motor_index] =
            solve_current_scale(limiter,
                                motor_index,
                                limiter->allocated_power[motor_index]);
    }

    update_feedback_power_prediction(limiter);
}
