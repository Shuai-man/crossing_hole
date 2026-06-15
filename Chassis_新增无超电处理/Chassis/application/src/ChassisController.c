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
float limit_pi(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (fabs(in - 180.0f) < 1e-4)
        {
            in = 180.0f;
            break;
        }
        else if (in < -180.0f)
        {
            in = in + 360.0f;
        }
        else if (in > 180.0f)
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
    float AngErr_front, AngErr_back, AngErr_left, AngErr_right, minAngle, angleBias = 0.0f;

    // 不同电机的计算不一样，但要保证最终的输出error_angle定义一致
    if (infantry.chassis_follow_type == FOUR_SIDES_FOLLOW_45) // 增加45度计算夹角
    {
        angleBias = 45.0f;
    }

    if (infantry.yaw_motor_type == YAW_GM6020)
    {
        AngErr_front = limit_pi(zeros_angle / 22.755555556f - target_ang / 22.755555556f + angleBias);
        AngErr_back = limit_pi(AngErr_front + 180.0f);
        AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
        AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
    }
    else if (infantry.yaw_motor_type == YAW_DM_MOTOR)
    {
        AngErr_front = limit_pi(zeros_angle / 90.0f - target_ang / 90.0f + angleBias);
        AngErr_back = limit_pi(AngErr_front + 180.0f);
        AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
        AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);
    }

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
void getDir()
{
    if (infantry.yaw_motor_type == YAW_GM6020)
    {
        // 计算与正对情况的夹角，22.755555556f = 8192 / 360.0f，结果转弧度
        float AngErr_front = limit_pi(GIMBAL_FOLLOW_ZERO / 22.755555556f - gimbal_receiver_pack1.yaw_motor_angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
        infantry.sin_dir = arm_sin_f32(AngErr_front);
        infantry.cos_dir = arm_cos_f32(AngErr_front);
			
				UI_FRONT_ERR = limit_pi((GIMBAL_FOLLOW_ZERO + UI_FRONT_BIAS) / 22.755555556f - gimbal_receiver_pack1.yaw_motor_angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
				UI_FRONT_SIN = arm_sin_f32(UI_FRONT_ERR);
				UI_FRONT_COS = arm_cos_f32(UI_FRONT_ERR);
    }
    else if (infantry.yaw_motor_type == YAW_DM_MOTOR)
    {

        float AngErr_front = limit_pi(GIMBAL_MOTOR_SIGN *(gimbal_receiver_pack1.yaw_motor_angle / 90.0f - GIMBAL_FOLLOW_ZERO / 90.0f )) * ANGLE_TO_RAD_COEF;
        infantry.sin_dir = arm_sin_f32(AngErr_front);
        infantry.cos_dir = arm_cos_f32(AngErr_front);
			
		UI_FRONT_ERR = limit_pi((GIMBAL_FOLLOW_ZERO + UI_FRONT_BIAS) / 90.0f - gimbal_receiver_pack1.yaw_motor_angle / 90.0f) * ANGLE_TO_RAD_COEF;
		UI_FRONT_SIN = arm_sin_f32(UI_FRONT_ERR);
		UI_FRONT_COS = arm_cos_f32(UI_FRONT_ERR);
    }
}

void get_sensors_info(Sensors *sensors_info)
{
    if (infantry.chassis_type == STEER_WHEEL)
    {
        for (int i = 0; i < 4; i++)
        {
            // 舵电机解码，速度没经过滤波
            GM6020_Decode(&sensors_info->steer_recv[i], &sensors_info->steer_decode[i]);

            // 轮电机解码
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }
    // 麦轮和全向轮无舵向电机
    else if (infantry.chassis_type == MECANUM_WHEEL || infantry.chassis_type == OMNI_WHEEL)
    {
        for (int i = 0; i < 4; i++)
        {
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
            M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
        }
    }
		//用于底盘跟随的error_angle
    infantry.error_angle = angle_z_err_get(gimbal_receiver_pack1.yaw_motor_angle, GIMBAL_FOLLOW_ZERO) * ANGLE_TO_RAD_COEF;
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




float test_power = 80; //不用或者没有超电就手动设置功率

// 设置机器人功率以及控制其速度
void set_robot_speed(Infantry *infantry)
{
	//没有裁判系统时，超电设定power为0，改为默认80w
	if(cap_controller.set_power)
	{
		infantry->set_power = cap_controller.set_power ; //超电设定80w
	}
	else
	{
		infantry->set_power = test_power; //手动更改功率上限
	} 
    // 根据设置的功率计算出设定速度，注意保持speed_x_max 与 speed_y_max 与 speed_yaw_max * wheel_radius基本同值
    // 因为该参数需要关联到功率控制部分，所以要保证在跑满功率的前提下给大，但过大会导致部分机器人轮子打滑，所以需要控制
    if (infantry->chassis_type == STEER_WHEEL)
    {
        infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
        infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
        infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
    }
    else if (infantry->chassis_type == MECANUM_WHEEL)
    {
        if(gimbal_receiver_pack1.control_mode_action == CV_ROTATE)
		{
			//小陀螺平移，要注意speed_error问题
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 1.0f + 0.0f; 
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 1.0f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 1.5f + 0.0f;
		}else{
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 1.50f + 0.0f; 
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 1.50f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 1.5f + 0.0f;
		}
    }
    else if (infantry->chassis_type == OMNI_WHEEL)
    {
#if ROBOT == QI_TIAN_DA_SHENG

		if(gimbal_receiver_pack1.control_mode_action == CV_ROTATE)
		{
			//bocchi58: 小陀螺平移，要保证speed_x_max与speed_y_max与speed_yaw_max*wheel_radius基本同值
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f; // 经测试，不同功率时，平地、上坡以及飞坡均能吃满set_power
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
		}else{
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f; // 经测试，不同功率时，平地、上坡以及飞坡均能吃满set_power
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 0.0f + 0.0f;
		}
#elif ROBOT == NIUNIU
		if(gimbal_receiver_pack1.control_mode_action == CV_ROTATE)
		{
			//bocchi58: 小陀螺平移，要保证speed_x_max与speed_y_max与speed_yaw_max*wheel_radius基本同值
			infantry->speed_x_max = (infantry->set_power - 50.0f) * 1.3f + 0.0f; // 经测试，不同功率时，平地、上坡以及飞坡均能吃满set_power
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 1.3f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 1.3f + 0.0f;
		}else{
			infantry->speed_x_max = (infantry->set_power - 50.0f) *1.5f + 0.0f; // 经测试，不同功率时，平地、上坡以及飞坡均能吃满set_power
			infantry->speed_y_max = (infantry->set_power - 50.0f) * 1.5f + 0.0f;
			infantry->speed_yaw_max = (infantry->set_power - 50.0f) * 1.5f + 0.0f;
		}
#endif
    }
}

// 加速策略	TD滤波，减缓变化
void wheels_accel(Infantry *infantry)
{  
		//给遥控器信号加个滤波，缓启动	底盘移动目标值
    infantry->target_x_v = TD_Calculate(&infantry->x_v_td, infantry->target_x_v_percent * infantry->speed_x_max);
    infantry->target_y_v = TD_Calculate(&infantry->y_v_td, infantry->target_y_v_percent * infantry->speed_y_max);

    if (remote_controller.control_mode_action == NOT_FOLLOW_GIMBAL || remote_controller.control_mode_action == CV_ROTATE) // 检录陀螺要变向
    {
        infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, infantry->target_yaw_v_percent * infantry->speed_yaw_max)/1.5f;
    }
    else
    {
        infantry->target_yaw_v = TD_Calculate(&infantry->yaw_v_td, infantry->speed_yaw_max);
    }
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
    case OMNI_WHEEL:
        omni_follow_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
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
    case OMNI_WHEEL:
        omni_chassis_control();
        break;
    case STEER_WHEEL:
        steer_chassis_control();
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
    case STEER_WHEEL:
        steer_chassis_control();
        break;
    case OMNI_WHEEL:
        omni_rotate_control();
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
    if (infantry.chassis_type == STEER_WHEEL)
    {
        // 底盘6020电机掉线处理，如果6020掉了，给对应的3508发0。
        for (int i = 0; i < 4; i++)
        {
            if (offline_detector.steer_6020_state[i] == STEER_6020_OFF)
                infantry.excute_info.wheels_set_current[i] = 0.0f;
        }
        // 舵
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_1 - 0x204, (int16_t)torque->steers_set_current[0], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_2 - 0x204, (int16_t)torque->steers_set_current[1], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_3 - 0x204, (int16_t)torque->steers_set_current[2], GM6020_CUR_MODE);
        GM6020_SendPack(torque->steers_send_data, GM6020_STD_CUR_ID_1_4, DJI_6020_MOTORS_4 - 0x204, (int16_t)torque->steers_set_current[3], GM6020_CUR_MODE);
        CanSend(DJI_STEERS_CAN, torque->steers_send_data, GM6020_STD_CUR_ID_1_4, 8);

        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
    else if (infantry.chassis_type == MECANUM_WHEEL || infantry.chassis_type == OMNI_WHEEL)
    {
        // 轮
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_1 - 0x200, torque->wheels_set_current[0], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_2 - 0x200, torque->wheels_set_current[1], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_3 - 0x200, torque->wheels_set_current[2], SEND_CURRENT);
        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_1_4, DJI_3508_MOTORS_4 - 0x200, torque->wheels_set_current[3], SEND_CURRENT);
        CanSend(DJI_WHEELS_CAN, torque->wheels_send_data, C620_STD_ID_1_4, 8);
    }
}
