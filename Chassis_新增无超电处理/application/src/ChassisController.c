#include "ChassisController.h"

Infantry infantry;

float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;

void InfantryInit(Infantry *infantry)
{
    // 功率控制初始化
    PowerLimitInit(&infantry->power_limiter, 4, M3508, infantry->power_limit_method);
		power_fit_init(&infantry->power_limiter);
    TD_Init(&infantry->x_v_td, 20000, 0.01); // 约0.5s上升时间
    TD_Init(&infantry->y_v_td, 20000, 0.01);
    TD_Init(&infantry->yaw_v_td, 20000, 0.01);
}

/**
 * @brief  转角限制到±180度
 * @param  输入转角
 * @retval 输出转角
 */
float limit_angle(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (fabs(in - 0) < 1e-4)
        {
            in = 0.0f;
            break;
        }
        else if (in < -0.0f)
        {
            in = in + 360.0f;
        }
        else if (in > 0.0f)
        {
            in = in - 360.0f;
        }
    }
    return in;
}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */
uint8_t dir = 0;
float err = 0;
float angle_z_err_get(float target_ang, float zeros_angle)
{
    //输入为电机机械角度
    float AngErr_front, AngErr_back, AngErr_left, AngErr_right, minAngle, angleBias = 0.0f;

    // 不同电机的计算不一样，但要保证最终的输出error_angle定义一致
    if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45) // 增加45度计算夹角
    {
        angleBias = 45.0f;
    }
    
    AngErr_front = limit_angle((zeros_angle - target_ang)+ angleBias);
    AngErr_back = limit_angle(AngErr_front + 180.0f);
    AngErr_left = limit_angle(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
    AngErr_right = limit_angle(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
    

    // 判断跟随
    if (infantry.chassis_follow_type == TWO_SIDES_FOLLOW)
    {
        if (fabs(AngErr_front) > fabs(AngErr_back))
        {
            infantry.chassis_direction = CHASSIS_BACK;
            return AngErr_back;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_FRONT;
            return AngErr_front;
        }
    }
    else if (infantry.chassis_follow_type == TWO_SIDES_LEFT_RIGHT)
    {
        if (fabs(AngErr_left) > fabs(AngErr_right))
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
            return AngErr_right;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_LEFT;
            return AngErr_left;
        }
    }
    else if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW || infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45)
    {
        minAngle = MIN(fabs(AngErr_front), MIN(fabs(AngErr_back), MIN(fabs(AngErr_left), fabs(AngErr_right))));
        if (fabs(fabs(AngErr_front) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_FRONT;
						dir = 1;
            return AngErr_front;
					
        }
        else if (fabs(fabs(AngErr_back) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_BACK;
						dir = 2;
            return AngErr_back;
        }
        else if (fabs(fabs(AngErr_left) - minAngle) < 1e-6f)
        {
            infantry.chassis_direction = CHASSIS_LEFT;
						dir = 3;
            return AngErr_left;
        }
        else
        {
            infantry.chassis_direction = CHASSIS_RIGHT;
						dir = 4;
            return AngErr_right;
        }
    }
    return 0;
}

// 获取控制方向
void getDir(void)
{
    float AngleErr_front = limit_angle(GIMBAL_MOTOR_SIGN *(infantry.yaw_angle  - GIMBAL_FOLLOW_ZERO)) ;
		arm_sin_cos_f32(AngleErr_front, &infantry.sin_dir, &infantry.cos_dir);
    
}

void get_sensors_info(Sensors *sensors_info)
{
    // 麦轮和全向轮无舵向电机
    if (infantry.chassis_type == MECANUM_WHEEL )
    {
        for (int i = 0; i < 4; i++)
        {
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }
		//用于底盘跟随的error_angle
    infantry.error_angle = angle_z_err_get(infantry.yaw_angle, GIMBAL_FOLLOW_ZERO);
		//用于速度解算的angle
    getDir();
}

void wheels_power_limit(Infantry *infantry)
{
    // 功率控制
    // limiter赋值
    float w_error = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        infantry->power_limiter.motor_w[i] = infantry->sensors_info.wheels_decode_raw[i].speed * ANGLE_TO_RAD_COEF;

        // 设置为电机反馈可作拟合
         infantry->power_limiter.I_collect[i] = infantry->sensors_info.wheels_decode_raw[i].torque_current;

        // 设置为发送电流可做削减
        infantry->power_limiter.motor_I[i] = infantry->excute_info.wheels_set_current[i] / C620_CURRENT_SEND_TRANS;

        // 设置实际轮子转速与设定转速差
        w_error = fabs(infantry->wheels_set_v[i] - infantry->sensors_info.wheels_decode[i].speed) * ANGLE_TO_RAD_COEF;
        infantry->power_limiter.motor_w_error[i] = fabs(w_error) ; 
    }

    // 限制功率

    PowerLimit(&infantry->power_limiter, infantry->set_power);	//计算出scale，影响set_current
//    PowerLimit(&infantry->power_limiter, 80);

    // 作功率削减
    for (int i = 0; i < 4; i++)
    {
        infantry->excute_info.wheels_set_current[i] *= infantry->power_limiter.send_torque_lower_scale[i];
        infantry->wheels_send_current[i] = infantry->excute_info.wheels_set_current[i];
    }
}




float test_power = 70; //不用或者没有超电就手动设置功率

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry)
{
	//没有裁判系统时，超电设定power为0，改为默认80w
	if(cap_controller.cap_vol_state!=CapVol_Low&&cap_controller.set_power>50.0f)
	{
		infantry->set_power = cap_controller.set_power ; //超电设定80w
	}
	else
	{
		infantry->set_power = test_power; //手动更改功率上限
	} 
    // 根据设置的功率计算出设定速度，注意保持speed_x_max 与 speed_y_max 与 speed_yaw_max * wheel_radius基本同值
    // 因为该参数需要关联到功率控制部分，所以要保证在跑满功率的前提下给大，但过大会导致部分机器人轮子打滑，所以需要控制

    
    if (infantry->chassis_type == MECANUM_WHEEL)
    {
       if(gimbal_receiver_pack1.chassis_mode_action == CV_ROTATE)
			{
				//这里的speed不要超过10
				infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.10f + 2.0f; 
				infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.10f + 2.0f;
				infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.5f + 1.5f;
			}else
			{
				infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.10f + 2.0f; 
				infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.10f + 2.0f;
				infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.3f + 2.0f;
			}
    }
    
}

// 加速策略	TD滤波，减缓变化
void wheels_accel(Infantry *infantry)
{  
		//给遥控器信号加个滤波，缓启动	底盘移动目标值
    infantry->target_x_v = TD_Calculate(&infantry->x_v_td, infantry->target_x_v_percent * infantry->speed_x_max);
    infantry->target_y_v = TD_Calculate(&infantry->y_v_td, infantry->target_y_v_percent * infantry->speed_y_max);
    infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, infantry->target_yaw_v_percent * infantry->speed_yaw_max);
 
}

void chassis_powerdown_control(Infantry *infantry)
{
    infantry->set_x_v = 0;
    infantry->set_y_v = 0;
    infantry->set_yaw_v = 0;
    infantry->target_x_v = 0;
    infantry->target_y_v = 0;
    infantry->target_yaw_v = 0;

    for (int i = 0; i < 4; i++)
    {
        infantry->excute_info.steers_set_current[i] = 0;
        infantry->excute_info.wheels_set_current[i] = 0;
    }

    TD_Clear(&infantry->x_v_td, 0);
    TD_Clear(&infantry->y_v_td, 0);
    TD_Clear(&infantry->yaw_v_td, 0);
}

void chassis_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_follow_control();
        break;
    default:
        break;
    }
}

void chassis_not_follow_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_chassis_control();
        break;
    default:
        break;
    }
}

void chassis_rotate_control(Infantry *infantry)
{
    switch (infantry->chassis_type)
    {
    case MECANUM_WHEEL:
        mecanum_rotate_control();
        break;
    default:
        break;
    }
}

void main_control(Infantry *infantry)
{
    switch (remote_controller.control_mode_action)
    {
    case FOLLOW_GIMBAL:
        chassis_follow_control(infantry);
        break;
    case NOT_FOLLOW_GIMBAL:
        chassis_not_follow_control(infantry);
        break;
    case CV_ROTATE:
        chassis_rotate_control(infantry);
        break;

    default:
        chassis_powerdown_control(infantry);
        break;
    }
}

void execute_control(ExcuteTorque *torque)//控制信息发送
{	
    if (infantry.chassis_type == MECANUM_WHEEL )
    {
        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
}
