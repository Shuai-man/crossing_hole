#include "PowerLimit.h"
#include "SuperPower.h"
#include "ChassisController.h"

#include "NingCap.h"

RLS power_rls;

void PowerLimitInit(PowerLimiter *limitter, int motor_num, MOTOR_TYPE motor_type, PowerLimitMethod method)
{
    limitter->motor_num = LIMIT_MAX_MIN(motor_num, 4, 1);
    limitter->motor_type = motor_type;
    limitter->power_limit_method = method;
		
		if (limitter->motor_type == MF9025)
    {
        limitter->power_k[I2] = MF9025_R;
        limitter->power_k[WI] = MF9025_K;
        limitter->power_k[W2] = MF9025_B;
        limitter->power_k[P0] = MF9025_P0;
    }
    else if (limitter->motor_type == M3508)
    {
        limitter->power_k[I2] = M3508_R;
        limitter->power_k[WI] = M3508_K;
        limitter->power_k[W2] = M3508_B;
        limitter->power_k[P0] = M3508_P0;
    }
}

void PowerLimit(PowerLimiter *limitter, float set_power)
{
    float i_2, w_i, w_i_collect, w_2, i_2_collect, sum_i_2 = 0.0f, sum_w_i = 0.0f, sum_w_i_collect = 0.0f, sum_w_2 = 0.0f, sum_i_2_collect = 0.0f;
    limitter->predict_send_power = 0.0f;

    for (int i = 0; i < limitter->motor_num; i++)
    {
        // 底盘3508电机掉线处理，如果3508掉了，不把该3508计入功率预测
        // 若3508电机运动过程中can突然掉线，电调反馈的W和I保持掉线前一刻的值不再更新。如果该掉线电机前一刻W或I很大,即前一刻功率预测很大，掉线后该3508功率预测一直保持该值。最后底盘4个电机转矩电流削减很大，速度龟爬
        if (offline_detector.wheel_3508_state[i] == WHEEL_3508_ON || offline_detector.wheel_3508_state[i] == NOT_INIT) // 7_14训练赛复活后超功率问题：刚复活没初始化状态要计入功率预测
        {
            i_2 = limitter->motor_I[i] * limitter->motor_I[i];
            w_i = limitter->motor_w[i] * limitter->motor_I[i];
            w_2 = sqrt(limitter->motor_w[i] * limitter->motor_w[i]);
						i_2_collect = limitter->I_collect[i] * limitter->I_collect[i];
						w_i_collect = limitter->motor_w[i] * limitter->I_collect[i];
					
            sum_i_2 += i_2;
            sum_w_i += w_i;
            sum_w_2 += w_2;
						sum_i_2_collect += i_2_collect;
						sum_w_i_collect += w_i_collect;
					
            limitter->motor_a[i] = i_2 * limitter->power_k[I2];
            limitter->motor_b[i] = w_i * limitter->power_k[WI];
            limitter->motor_c[i] = w_2 * limitter->power_k[W2];
            limitter->motor_P[i] = limitter->motor_a[i] + limitter->motor_b[i] + limitter->motor_c[i] + limitter->power_k[P0];

            // 计算总功率
            limitter->predict_send_power += limitter->motor_P[i];
        }
    }

    limitter->set_power = set_power;
    limitter->sum_i_2 = sum_i_2;
    limitter->sum_w_i = sum_w_i;
    limitter->sum_w_2 = sum_w_2;
		limitter->sum_i_2_collect = sum_i_2_collect;
		limitter->sum_w_i_collect = sum_w_i_collect;
		
    // 重置分配策略
    for (int i = 0; i < limitter->motor_num; i++)
    {
        limitter->power_arrange_state[i] = NOT_ARRANGE;
    }

    if (limitter->predict_send_power > set_power)	//如果predict没有超过set值，那么就不会进行功率限制
    {
        // 首先找到小于0的功率值，以及小于最小给定电机的功率值
        limitter->can_arrange_power = set_power;
        limitter->need_arrange_power = 0;
        limitter->need_arrange_w = 0;

        if (limitter->power_limit_method == TORQUE_REDUCE_METHOD)
        {
            // 首先找到小于0的功率值，以及小于最小给定电机的功率值
            for (int i = 0; i < limitter->motor_num; i++)
            {
                if (limitter->motor_P[i] <= 1e-4f) // 反向输出（速度方向与电流方向相反）情况，不作功率限制(或者电流很小的情况)
                {
                    limitter->can_arrange_power -= limitter->motor_P[i];
                    limitter->power_arrange_state[i] = NEG_ARRANGE;
                }
                else
                {
                    limitter->need_arrange_power += limitter->motor_P[i];
                    limitter->power_arrange_state[i] = NEED_ARRANGE;
                }
            }

            // 二次分配
            if (limitter->need_arrange_power >= limitter->can_arrange_power)
            {
                for (int i = 0; i < limitter->motor_num; i++)
                {
                    if (limitter->power_arrange_state[i] == NEED_ARRANGE)
                    {
                        limitter->motor_P[i] = limitter->motor_P[i] * limitter->can_arrange_power / limitter->need_arrange_power;
                    }
                }
            }
        }
        else if (limitter->power_limit_method == SPEED_ERROR_METHOD)
        {
            for (int i = 0; i < limitter->motor_num; i++)
            {
                if (limitter->motor_P[i] <= 1e-4f) // 反向输出（速度方向与电流方向相反）情况，不作功率限制(或者电流很小的情况)
                {
                    limitter->can_arrange_power -= limitter->motor_P[i];
                    limitter->power_arrange_state[i] = NEG_ARRANGE;
                }
                else
                {
                    limitter->need_arrange_power += fabs(limitter->motor_P[i]);
                    limitter->need_arrange_w += limitter->motor_w_error[i];
                    limitter->power_arrange_state[i] = NEED_ARRANGE;
                }
            }

            // 二次分配，功率分配策略1：利用每个电机最大功率进行分配
            if (limitter->need_arrange_power >= limitter->can_arrange_power)
            {
                for (int i = 0; i < limitter->motor_num; i++)
                {
                    if (limitter->power_arrange_state[i] == NEED_ARRANGE)
                    {
                        limitter->motor_P[i] = limitter->motor_w_error[i] * limitter->can_arrange_power / (limitter->need_arrange_w + 1e-6f);
                    }
                }
            }
        }
    }
    else
    {
        // 不进行削减
        for (int i = 0; i < limitter->motor_num; i++)
        {
            limitter->power_arrange_state[i] = NORMAL_ARRANGE;
        }
    }

    // 计算削减系数 最后输出的值
    float a, b, c;
    for (int i = 0; i < limitter->motor_num; i++)
    {
        if (limitter->power_arrange_state[i] == NEG_ARRANGE || limitter->power_arrange_state[i] == NORMAL_ARRANGE)
        {
					limitter->send_torque_lower_scale[i] = 1.0f;		//一般情况下不限制，先前的计算是为了功率不够的情况
        }
        else if (limitter->power_arrange_state[i] == NEED_ARRANGE)
        {
            a = limitter->motor_a[i];
            b = limitter->motor_b[i];
            c = limitter->motor_c[i] + limitter->power_k[P0] - limitter->motor_P[i];
						
//            if (c > 0) // 无正解，转速过高，仅可以选0作为解
//            {
//                limitter->send_torque_lower_scale[i] = 0.0f;
//                limitter->power_arrange_state[i] = ARRANGE_ERROR;
//            }
//            else if (fabs(a) < 1e-6f) // a为0保护
//            {
//                limitter->send_torque_lower_scale[i] = 0.0f;
//                limitter->power_arrange_state[i] = ARRANGE_ERROR;
//            }
//            else
//            {
//                limitter->send_torque_lower_scale[i] = (-b + Sqrt(b * b - 4 * c * a)) / 2 / a;
//                limitter->send_torque_lower_scale[i] = LIMIT_MAX_MIN(limitter->send_torque_lower_scale[i], 1.0f, 0.0f);
//            }
							if(b * b - 4 * a * c <= 1e-6f)
							{
								limitter->send_torque_lower_scale[i] = - b / (2 * a);
								limitter->send_torque_lower_scale[i] = LIMIT_MAX_MIN(limitter->send_torque_lower_scale[i], 1.0f, 0.0f);
							}
							else if(b * b - 4 * a * c > 1e-6f)
							{
								 limitter->send_torque_lower_scale[i] = (-b + Sqrt(b * b - 4 * c * a)) / 2 / a;
								 limitter->send_torque_lower_scale[i] = LIMIT_MAX_MIN(limitter->send_torque_lower_scale[i], 1.0f, 0.0f);
							}
							
        }
    }
		power_fitting(limitter);
    // 外部自行计算削减后的力矩
}

void power_fit_init(PowerLimiter *limitter)
{
		RLS_Init(&power_rls, 2, 1, 1);
		power_rls.x_data[I2] = limitter->power_k[I2];
		power_rls.x_data[W2] = limitter->power_k[W2];
		power_rls.P_data[0] = 0.0001;
		power_rls.P_data[1] = 0;
		power_rls.P_data[2] = 0;
		power_rls.P_data[3] = 0.0001;
}

void power_fitting(PowerLimiter *limitter)
{
		LossJudge(&global_debugger.super_power_debugger);
		if(global_debugger.super_power_debugger.loss_time < 50)
		{
			power_rls.y_data[0] = cap_recv_data.chassis_power/100.0f - limitter->power_k[WI]*limitter->sum_w_i_collect - limitter->power_k[P0] * 4;
			power_rls.H_data[0] = limitter->sum_i_2_collect;
			power_rls.H_data[1] = limitter->sum_w_2;
			RLS_Update(&power_rls);
			limitter->power_k[I2] = power_rls.x_data[I2];
			limitter->power_k[W2] = power_rls.x_data[W2];
			limitter->now_power_predict = limitter->power_k[WI]*limitter->sum_w_i_collect + limitter->power_k[I2]*limitter->sum_i_2_collect + limitter->power_k[W2]*limitter->sum_w_2 + limitter->power_k[P0] * 4;
		}
}


void setINAPower(PowerLimiter *limitter, float ina_power)
{
    limitter->actual_ina260_power = ina_power;
}
