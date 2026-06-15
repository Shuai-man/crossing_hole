#include "ChassisController.h"
#include "PowerLimit.h"
void steer_pid_init()
{
    // 舵向初始编码值设定
    infantry.steer_init_encoder[STEER1] = 3740;
    infantry.steer_init_encoder[STEER2] = 5764;
    infantry.steer_init_encoder[STEER3] = 5090;
    infantry.steer_init_encoder[STEER4] = 3048;

    // 轮毂电机安装方向
    infantry.steer_wheel_install_direction[STEER1] = 1;
    infantry.steer_wheel_install_direction[STEER2] = -1;
    infantry.steer_wheel_install_direction[STEER3] = -1;
    infantry.steer_wheel_install_direction[STEER4] = 1;
    // 轮子控制PID
    PID_Init(&infantry.wheels_pid[STEER1], C620_MAX_SEND_CURRENT, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[STEER2], C620_MAX_SEND_CURRENT, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[STEER3], C620_MAX_SEND_CURRENT, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.wheels_pid[STEER4], C620_MAX_SEND_CURRENT, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, NONE);

    // 舵向控制PID
    PID_Init(&infantry.steers_angle_pid[STEER1], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER2], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER3], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&infantry.steers_angle_pid[STEER4], 1000, 0, 0, 30, 0, 0, 0, 0, 0, 0, 1, NONE);

    PID_Init(&infantry.steers_speed_pid[STEER1], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER2], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER3], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);
    PID_Init(&infantry.steers_speed_pid[STEER4], 30000, 20000, 0, 17, 7, 0, 0, 0, 0, 0, 1, Integral_Limit);

    // 6020前馈初始化
    infantry.Steer_6020_FF_Coefficient[0] = 10.0f;
    infantry.Steer_6020_FF_Coefficient[1] = 0.0f;
    infantry.Steer_6020_FF_Coefficient[2] = 0.0f;
    Feedforward_Init(&infantry.Steer_6020_FF, 7500, infantry.Steer_6020_FF_Coefficient, 0.004, 0, 0); // 15000

    // 底盘正方形跟随 输出旋转角速度rad/s  输入弧度制角度
    PID_Init(&infantry.turn_pid, 3.0, 0, 0, -15.0f, 0, 0, 0, 0, 0.001, 0.009, 1, DerivativeFilter | OutputFilter);
}

/**********************************************************************************************************
 *函 数 名: add_vector
 *功能说明: 两个向量相加,返回向量为-180到180角度，输入输出均为degree
 *形    参: vector1 vector2
 *返 回 值: result = vector1 + vector2(向量加法)
 **********************************************************************************************************/
Vector add_vector(Vector *v1, Vector *v2)
{
    Vector result;
    float v1_angle = DEG2R_RATIO * v1->angle;
    float v2_angle = DEG2R_RATIO * v2->angle;
    float x = v1->module * arm_cos_f32(v1_angle) + v2->module * arm_cos_f32(v2_angle);
    float y = v1->module * arm_sin_f32(v1_angle) + v2->module * arm_sin_f32(v2_angle);
    arm_sqrt_f32(x * x + y * y, &(result.module));
    result.angle = atan2f(y, x) * R2DEG_RATIO;
    return result;
}

/*舵向电机控制运动优化*/
float steer_moving_optimization(uint8_t steer_num)
{
    /*顺逆时针转动角度计算*/
    float min_angle;
    float flipped_angle_set;
    float invert_flag = 1.0; // 速度向量反转标志
    float angle_now = (infantry.sensors_info.steer_recv[steer_num].angle - infantry.steer_init_encoder[steer_num]) / 8192.0f * 360.0f;
    if (angle_now < 0)
        angle_now += 360; // 把转动方向统一到相对初始角度正方向
    float angle_set = fmodf(infantry.steer_vector[steer_num].angle, 360);
    if (angle_set < 0)
        angle_set += 360;                                              // 把转动方向统一到相对初始角度正方向
    float angle_clockwise = fmodf((angle_set - angle_now + 360), 360); // 转动方向统一到相对现在位置正方向
    float angle_counter_clockwise = 360 - angle_clockwise;
    /*比较顺逆时针转动角度大小*/
    if (angle_clockwise < angle_counter_clockwise)
    {
        min_angle = angle_clockwise;
    }
    else
    {
        min_angle = -angle_counter_clockwise;
    }
    /*若最小旋转角大于90°，则翻转轮组的向量坐标重新计算*/
    if (abs(min_angle) > 90)
    {
        float flipped_angle_now = angle_now + 180;
        if (flipped_angle_now >= 360)
        {
            flipped_angle_now -= 360;
        } // 把翻转后的旋转方向统一到相对初始角正方向
        if (angle_clockwise > angle_counter_clockwise)
        {
            flipped_angle_set = fmodf((angle_set - flipped_angle_now + 360), 360);
        }
        else
        {
            flipped_angle_set = -fmodf((flipped_angle_now - angle_set + 360), 360);
        }
        invert_flag = -1.0; // 改变轮子转向
        min_angle = flipped_angle_set;
    }
    infantry.steer_vector[steer_num].module *= invert_flag;
    return min_angle;
}

/**********************************************************************************************************
 *函 数 名: steer_control
 *功能说明: 舵轮底盘控制计算,跟随、陀螺、平动集成在一个函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void steer_chassis_control(void)
{
    /*如要底盘跟随,计算旋转速度*/
    if (remote_controller.control_mode_action == FOLLOW_GIMBAL)
    {
        infantry.target_yaw_v = GIMBAL_MOTOR_SIGN * (PID_Calculate(&infantry.turn_pid, infantry.error_angle, 0.0f)); // 单位rad/s
    }

    // 拨杆回正后（云台板传下来的v_x从1变为0），TD算完infantry.target_x_v会有e-14量级的微小值，不是0.0f，导致atan2函数不是0而是某个值，故target_x_y小于e-5后直接赋0.0f
    if (abs(infantry.target_x_v) < 1e-5f)
        infantry.target_x_v = 0.0f;
    if (abs(infantry.target_y_v) < 1e-5f)
        infantry.target_y_v = 0.0f;
    if (abs(infantry.target_yaw_v) < 1e-5f)
        infantry.target_yaw_v = 0.0f;

//    if (infantry.target_x_v != 0 || infantry.target_y_v != 0 || infantry.target_yaw_v != 0 || gimbal_receiver_pack1.through_hole_flag)
//    {
        /*计算平动向量的速度模值m/s、方向degree*/
        if (infantry.chassis_direction == CHASSIS_BACK)
            infantry.robot_vector.angle = 180.0f + R2DEG_RATIO * atan2f(infantry.target_x_v, infantry.target_y_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;
        else
            infantry.robot_vector.angle = R2DEG_RATIO * atan2f(infantry.target_x_v, infantry.target_y_v) + GIMBAL_MOTOR_SIGN * infantry.error_angle * R2DEG_RATIO;

        arm_sqrt_f32(infantry.target_y_v * infantry.target_y_v + infantry.target_x_v * infantry.target_x_v, &infantry.robot_vector.module); // 计算速度模值

        /*平动向量赋值,以底盘建立坐标系*/
        for (int i = 0; i < 4; i++)
        {
            infantry.steer_vector[i].angle = infantry.robot_vector.angle;   // degree 角度转换到底盘坐标系
            infantry.steer_vector[i].module = infantry.robot_vector.module; // m/s
        }

        /*转动向量赋值*/
        Vector w_vector[4];
        if (remote_controller.control_mode_action == CV_ROTATE || remote_controller.control_mode_action == FOLLOW_GIMBAL)
        {
            // 底盘旋转3508线速度大小赋值 m/s
            w_vector[STEER1].module = +infantry.target_yaw_v * STEER_INFANTRY_RADIUS; // 从角速度rad/s到线速度m/s
            w_vector[STEER2].module = -infantry.target_yaw_v * STEER_INFANTRY_RADIUS;
            w_vector[STEER3].module = -infantry.target_yaw_v * STEER_INFANTRY_RADIUS;
            w_vector[STEER4].module = +infantry.target_yaw_v * STEER_INFANTRY_RADIUS;
            // 底盘旋转6020角度朝向赋值 degree
            w_vector[STEER1].angle = +SPIN_ANGLE;
            w_vector[STEER2].angle = -SPIN_ANGLE;
            w_vector[STEER3].angle = +SPIN_ANGLE;
            w_vector[STEER4].angle = -SPIN_ANGLE;

            /*执行向量加法,把转动、平动向量相加*/
            for (int u = 0; u < 4; u++)
            {
                infantry.steer_vector[u] = add_vector(&(infantry.steer_vector[u]), &(w_vector[u]));
            }
        }

        for (int i = 0; i < 4; i++)
        {
            /*6020转角最小和3508反向转动策略*/
            infantry.steer_vector[i].angle = infantry.sensors_info.steer_decode[i].angle + steer_moving_optimization(i);

            /*电机安装方向修正*/
            infantry.steer_vector[i].module *= infantry.steer_wheel_install_direction[i];

            /*单位转换 从线速度m/s -> 轴degree/s*/
            infantry.steer_vector[i].module *= STEER_SPEED_TO_DEGEREE_S;

            /*pid计算*/
            infantry.excute_info.wheels_set_current[i] = PID_Calculate(&infantry.wheels_pid[i], infantry.sensors_info.wheels_decode[i].speed, infantry.steer_vector[i].module);

            float Steer_Speed_Setpoint = PID_Calculate(&infantry.steers_angle_pid[i], infantry.sensors_info.steer_decode[i].angle, infantry.steer_vector[i].angle);
            infantry.steers_set_current = PID_Calculate(&infantry.steers_speed_pid[i], infantry.sensors_info.steer_decode[i].speed, Steer_Speed_Setpoint) + Feedforward_Calculate(&infantry.Steer_6020_FF, Steer_Speed_Setpoint);
            infantry.excute_info.steers_set_current[i] = infantry.steers_set_current;
        }
    //}
//    else
//    {
//        for (int m = 0; m < 4; m++)
//        {
//            infantry.excute_info.steers_set_current[m] = 0.0f;
//            infantry.excute_info.wheels_set_current[m] = PID_Calculate(&infantry.wheels_pid[m], infantry.sensors_info.wheels_decode[m].speed, 0.0f);
//        }
//    }
}
