#include "GimbalTask.h"

int8_t dji_motors_send_data_can1[8];
int8_t dji_motors_send_data_can2[8];

SI_t SI_object;
SawToothWave saw_tooth_wave;

/**
 * @brief 拨盘转速检测&自动反转
 * @param[in] void
 */
void autoReverse(GimbalController *gimbal, ToggleController *toggle)
{
    // 暂时只有位置控
    if (fabsf(toggle->toggle_pos_pid.Err) > ONE_GRID_ANGLE)
    {
        if (gimbal->if_spin_reverse >= 0.99f) // 正转时候检测卡弹
        {
            // 当拨盘速度小于参考参考值
            if (fabsf(toggle->toggle_info.speed) < 0.3f * fabsf(toggle->toggle_speed_pid.Ref))
            {
                gimbal->if_spin_reverse = -1.3f; // 开启反拨
            }
        }
    }
    else if (gimbal->if_spin_reverse <= -1) // 进入这说明误差到了一格内
    {
        if (fabsf(toggle->toggle_pos_pid.Err) < 0.3f * ONE_GRID_ANGLE) // 进一步收敛
        {
            // 结束反拨
            gimbal->if_spin_reverse = 1.0f;
        }
    }
}

// 云台YAW回正
void Gimbal_Return(GimbalController *gimbal, RemoteController *remote)
{
    if (gimbal->return_flag)
    {
        if (gimbal->return_flag == 1)
        { // 记录当前模式
            remote->last_chassis_mode_action = remote->chassis_mode_action;
            gimbal->return_flag = 2;
            setChassisModeAction(NOT_CONTROL_MODE);
        }

        gimbal->target_yaw_angle = gimbal->gyro_yaw_angle + gimbal->err_angle;
        if (fabsf(gimbal->err_angle) < 0.5f)
        {
            gimbal->return_flag = 0;
            // 能不能改成上一次的模式
            setChassisModeAction(remote->last_chassis_mode_action);
            return;
        }
    }
}

void Gimbal_Act_Cal(void)
{
    // pitch限制幅值
    limitPitchAngle();
    // yaw计算

    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);

    pc_send_data.mode_want = NOT_USE_AIM;
}

void Gimbal_Auto_aim_Cal(void) // 打车
{
    gimbal_controller.pc_recv_rad[PITCH_MOTOR] = pc_recv_data.pitch;
    gimbal_controller.pc_recv_rad[YAW_MOTOR] = pc_recv_data.yaw;

    // 设置目标角度
    if (pc_recv_data.detect_number != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data.pitch) < 60.0f && fabsf(gimbal_controller.target_yaw_angle - pc_recv_data.yaw) < 70.0f)
    {
        if (global_debugger.pc_receive_debugger.state == ON)
        {
            gimbal_controller.target_pitch_angle = gimbal_controller.pc_recv_rad[PITCH_MOTOR];
            gimbal_controller.target_yaw_angle = gimbal_controller.pc_recv_rad[YAW_MOTOR];
        }
    }

    pc_send_data.mode_want = STD_AUTO_AIM; // 改变发送的模式

    // pitch限制幅值
    limitPitchAngle();

    // 角度计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
}

void Gimbal_Buff_Cal(void)
{
    gimbal_controller.pc_recv_rad[PITCH_MOTOR] = pc_recv_data.pitch;
    gimbal_controller.pc_recv_rad[YAW_MOTOR] = pc_recv_data.yaw;

    // 设置目标角度
    if (pc_recv_data.detect_number != 0 && fabsf(gimbal_controller.target_pitch_angle - pc_recv_data.pitch) < 60.0f && fabsf(gimbal_controller.target_yaw_angle - pc_recv_data.yaw) < 70.0f)
    {
        if (global_debugger.pc_receive_debugger.state == ON && remote_controller.auto_arm == 1) // pc掉线后数据不会更新，所以要加检测
        {
            gimbal_controller.target_pitch_angle = gimbal_controller.pc_recv_rad[PITCH_MOTOR];
            gimbal_controller.target_yaw_angle = gimbal_controller.pc_recv_rad[YAW_MOTOR];
        }
    }
    if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
    {
        pc_send_data.mode_want = BUFF; // 改变发送的模式
    }
    else if (remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
    {
        pc_send_data.mode_want = BIG_BUFF;
    }

    // pitch限制幅值
    limitPitchAngle();

    // yaw计算
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
}

void Gimbal_SI_Cal()
{
#if GIMBAL_TEST
    // 求力矩与电机转动角度之间的传递函数
    gimbal_controller.yaw_speed_pid.Output = SIRun(&SI_object, gimbal_controller.delta_t);
    // 限幅

#endif
}

void Shoot_Power_down_Cal(void)
{
    if (fabs(friction_wheels.friction_motor_msgs[LEFT_FRICTION_WHEEL].speed) > 6000 || fabs(friction_wheels.friction_motor_msgs[RIGHT_FRICTION_WHEEL].speed) > 6000)
    {
        FrictionWheel_Set(0);
    }
    else
    {
        PID_Clear(&friction_wheels.PidFrictionSpeed[LEFT_FRICTION_WHEEL]);
        PID_Clear(&friction_wheels.PidFrictionSpeed[RIGHT_FRICTION_WHEEL]);

        friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL] = 0;
        friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL] = 0;
    }

    Toggle_Calculate(TOGGLE_STOP, 0.0f);
}

// 比赛打弹
void Shoot_Fire_Cal(void)
{
    // 摩擦轮转速
    setFrictionSpeed();
    FrictionWheel_Set(friction_wheels.set_speed);
    // 设置拨盘转动速度
    Toggle_SelectShootFreq();
    if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE || remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
    {
        // 打符模式，采用单发模式
        if (remote_controller.single_shoot_flag) // 触发单发射击标志
        {
            Toggle_AddGrid(&toggle_controller, 1);
            remote_controller.single_shoot_flag = FALSE;
        }
        Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
    }
    else if (remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE)
    {
        remote_controller.single_shoot_flag = FALSE; // 自瞄连发
        if (pc_recv_data.shoot_flag == 1)
        {
            Toggle_Fire(chassis_pack_get_1.shoot_avaiable);
        }
        else
        {
            Toggle_Calculate(TOGGLE_STOP, 0.0f);
        }
    }
    else // 手动开火
    {
        remote_controller.single_shoot_flag = FALSE;
        Toggle_Fire(chassis_pack_get_1.shoot_avaiable);
    }
}

void Gimbal_Msg_Update(void)
{
    // 更新传感器信息
    updateGyro();

    M2006_Decode(&toggle_controller.toggle_recv, &toggle_controller.toggle_info, WITH_REDUCTION, 0.90);
    M2006_Decode(&lifting_controller.lift_recv, &lifting_controller.lift_info, WITH_REDUCTION, 0.90);
    // 摩擦轮
    for (int i = 0; i < 2; i++)
    {
        M3508_Decode(&friction_wheels.friction_motor_recv[i], &friction_wheels.friction_motor_msgs[i], ONLY_SPEED, 0.90);
    }
}

void execute_func(void)
{
    int8_t send_data[2][8] = {0};

    // 单独接电机进行测试时，会进行保护，tff不会赋值
    if (global_debugger.pitch_debugger.state == ON && global_debugger.yaw_debugger.state == ON)
    {
        if (remote_controller.gimbal_action == GIMBAL_POWER_DOWN)
        {
            // 设置为0力矩
            gimbal_controller.DM_Pitch_Motor.t_ff = 0;
            DM_Motor_Control(&gimbal_controller.DM_Pitch_Motor, send_data[PITCH_MOTOR], DM_MIT_CONTROL);

            gimbal_controller.DM_Yaw_Motor.t_ff = 0;
            DM_Motor_Control(&gimbal_controller.DM_Yaw_Motor, send_data[YAW_MOTOR], DM_MIT_CONTROL);
        }
        else
        {
            gimbal_controller.DM_Pitch_Motor.t_ff = GIMBAL_PITCH_MOTOR_SIGN * gimbal_controller.pitch_out;
            DM_Motor_Control(&gimbal_controller.DM_Pitch_Motor, send_data[PITCH_MOTOR], DM_MIT_CONTROL);

            gimbal_controller.DM_Yaw_Motor.t_ff = GIMBAL_YAW_MOTOR_SIGN * gimbal_controller.yaw_out;
            DM_Motor_Control(&gimbal_controller.DM_Yaw_Motor, send_data[YAW_MOTOR], DM_MIT_CONTROL);
        }
    }
    else
    {
        // 两种情况：
        // 少电机，全部下电
        // 初始化，力矩给0  如果发现重新上电后电机初始化不了,reset后才行能初始化，建议检查can芯片是不是掉了
        gimbal_controller.DM_Pitch_Motor.t_ff = 0;
        gimbal_controller.DM_Yaw_Motor.t_ff = 0;
        DM_Motor_Control(&gimbal_controller.DM_Pitch_Motor, send_data[PITCH_MOTOR], DM_MIT_CONTROL);
        DM_Motor_Control(&gimbal_controller.DM_Yaw_Motor, send_data[YAW_MOTOR], DM_MIT_CONTROL);
    }
    // DM send_data
    CanSend(PITCH_MOTOR_CAN, send_data[PITCH_MOTOR], PITCH_MOTOR_SLAVE_CAN_ID);
    CanSend(YAW_MOTOR_CAN, send_data[YAW_MOTOR], YAW_MOTOR_SLAVE_CAN_ID);

    // M3508
    M3508_SendPack(dji_motors_send_data_can2, C620_STD_ID_5_8, LEFT_FRICTION_WHEEL_CAN_ID - 0x200, friction_wheels.send_to_motor_current[LEFT_FRICTION_WHEEL], SEND_CURRENT);
    M3508_SendPack(dji_motors_send_data_can2, C620_STD_ID_5_8, RIGHT_FRICTION_WHEEL_CAN_ID - 0x200, friction_wheels.send_to_motor_current[RIGHT_FRICTION_WHEEL], SEND_CURRENT);
    CanSend(FRICTION_WHEEL_CAN, dji_motors_send_data_can2, C620_STD_ID_5_8);

    M2006_SendPack(dji_motors_send_data_can1, C610_STD_ID_5_8, TOGGLE_MOTOR_CAN_ID - 0x200, toggle_controller.send_current);
    CanSend(TOGGLE_MOTOR_CAN, dji_motors_send_data_can1, C610_STD_ID_5_8);
    M2006_SendPack(dji_motors_send_data_can1, C610_STD_ID_5_8, LIFT_MOTOR_CAN_ID - 0x200, lifting_controller.send_current);
    CanSend(LIFT_MOTOR_CAN, dji_motors_send_data_can1, C610_STD_ID_5_8);
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void Gimbal_Task(void *pvParameters)
{

    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1000HZ

    gimbal_controller.if_spin_reverse = 1;
    memset(dji_motors_send_data_can2, 0, 8);
    memset(dji_motors_send_data_can1, 0, 8);

    // 摩擦轮PID初始化
    FrictionWheel_Init();
    // 云台PID初始化
    GimbalMotorInit();
    GimbalPidInit();
    // 拨弹电机PID初始化
    Toggle_Init();
    // 丝杆电机初始化
    LiftPidInit();
    /* 系统辨识以及测试 */
    MX_USB_DEVICE_Init(); // USB初始化会默认放到freertos的第一个task里面,一定要确保调用了,否则无法成功初始化
    vTaskDelay(1000);

    int index = 0;

    while (1)
    {

        xLastWakeTime = xTaskGetTickCount();

        Gimbal_Msg_Update();
        Gimbal_ErrorAngle();
        Gimbal_Return(&gimbal_controller, &remote_controller);

        switch (remote_controller.gimbal_action)
        {
        case GIMBAL_POWER_DOWN: // 掉电模式
            GimbalClear();
            break;
        case GIMBAL_ACT_MODE: // 云台运动模式
            Gimbal_Act_Cal();
            break;
        case GIMBAL_AUTO_AIM_MODE: // 自瞄模式
            Gimbal_Auto_aim_Cal();
            break;
        case GIMBAL_SMALL_BUFF_MODE:
            Gimbal_Buff_Cal();
            break;
        case GIMBAL_BIG_BUFF_MODE:
            Gimbal_Buff_Cal();
            break;
        default:
            GimbalClear();
            break;
        }

        switch (remote_controller.shoot_action)
        {
        case SHOOT_POWER_DOWN_MODE: // 掉电模式
            Shoot_Power_down_Cal();
            break;
        case SHOOT_FIRE_MODE: // 开火模式
            Shoot_Fire_Cal();
            break;
        default:
            Shoot_Power_down_Cal();
            break;
        }

        // 控制丝杆升降
        Lifting_Control();

        if (index % 2 == 0) // 500hz
        {
            SendtoPC(); // 将信息发送给上位机
                        // 执行函数
            execute_func();
        }
        index++;

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
