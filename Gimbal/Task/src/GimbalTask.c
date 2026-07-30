#include "GimbalTask.h"

#include "Gimbal.h"
#include "GimbalSystemID.h"
#include "remote_control.h"
#include "FrictionWheel.h"
#include "pc_serial.h"
#include "lifting_control.h"
#include "KeyMouse.h"

#include "debug.h"
#include "Offline_Task.h"
#include "usb_device.h"

#include "can_config.h"
#include "bsp_can.h"
#include "Motor_Typdef.h"

#define RAD_TO_ANGLE_COEF 57.295779513f
#define ANGLE_TO_RAD_COEF 0.0174532925f

/**------------云台运动控制--------------*/

/**
 * @brief 云台YAW回正
 * @param[in] gimbal 云台控制器指针
 * @param[in] remote 远程控制器指针
 */
void Gimbal_Return(GimbalController *gimbal, RemoteController *remote)
{
    if (gimbal->return_flag)
    {
        if (gimbal->return_flag == 1)
        { // 记录当前模式
            remote->last_chassis_mode_action = remote->chassis_mode_action;
            gimbal->return_flag = 2;
        }

        gimbal->target_yaw_angle = gimbal->gyro_yaw_angle + gimbal->err_angle;
        if (fabsf(gimbal->err_angle) < 0.5f)
        {
            gimbal->return_flag = 0;
            setChassisModeAction(remote->last_chassis_mode_action);
            return;
        }
        setChassisModeAction(NOT_CONTROL_MODE);
    }
}

/**
 * @brief 云台PC控制
 * @param[in] void
 */
void Gimbal_PC_Cal(void)
{
    float yaw_friction_ratio;
    // PITCH
    gimbal_controller.target_pitch_angle = pc_recv_data.pitch_setpoint;
    limitPitchAngle(); // pitch限制幅值
    gimbal_controller.gravity_comp = GIMBAL_PITCH_SIN * sin(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) +
                                     GIMBAL_PITCH_COS * cos(gimbal_controller.gyro_pitch_angle * ANGLE_TO_RAD_COEF) +
                                     GIMBAL_PITCH_C * sign(gimbal_controller.gyro_pitch_speed);
    gimbal_controller.ff_tff_pitch = GIMBAL_PITCH_J * pc_recv_data.pitch_acc_setpoint +  // 惯量 × 加速度，主要输出项
                                     GIMBAL_PITCH_B * pc_recv_data.pitch_omega_setpoint; // 阻尼 × 速度
    PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.gyro_pitch_angle, gimbal_controller.target_pitch_angle);
    PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.gyro_pitch_speed, pc_recv_data.pitch_omega_setpoint);
    gimbal_controller.pitch_out = gimbal_controller.pitch_angle_pid.Output + gimbal_controller.pitch_speed_pid.Output + gimbal_controller.gravity_comp + gimbal_controller.ff_tff_pitch;

    // YAW：增量式控制
    gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + gimbal_controller.pc_dt_yaw;
    yaw_friction_ratio = LIMIT_MAX_MIN(pc_recv_data.yaw_omega_setpoint /
                                           BORDER_FRICTION_SPEED,
                                       1.0f, -1.0f);
    gimbal_controller.ff_tff =
        GIMBAL_YAW_J * pc_recv_data.yaw_acc_setpoint +   // 惯量 × 加速度，主要输出项
        GIMBAL_YAW_B * pc_recv_data.yaw_omega_setpoint + // 阻尼 × 速度
        GIMBAL_YAW_C * yaw_friction_ratio;               // 低速平滑的运动库仑摩擦
    PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, gimbal_controller.target_yaw_angle);
    PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.gyro_yaw_speed, pc_recv_data.yaw_omega_setpoint); // 速度环类似阻尼项，因为前馈会拉着电机加速，所以实际速度比设定速度快，一开始速度环输出会是负的，如果影响大，可以适当减小速度环的p
    // 总输出 = 前馈 + 角度环输出 + 速度环输出
    gimbal_controller.yaw_out = gimbal_controller.ff_tff + gimbal_controller.yaw_angle_pid.Output + gimbal_controller.yaw_speed_pid.Output;

    // 同步TD的输入，防止退出时产生波动
    TD_Clear(&gimbal_controller.pos_yaw_td, gimbal_controller.target_yaw_angle);
    TD_Clear(&gimbal_controller.pos_pitch_td, gimbal_controller.target_pitch_angle);
}

/**
 * @brief 云台手动控制
 * @param[in] void
 */
void Gimbal_Act_Cal(void)
{
    // pitch限制幅值
    limitPitchAngle();
    // 计算云台角度
    Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
    Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
}

/**
 * @brief 自瞄模式的云台控制，处理不同情况下的云台控制方式
 * @param[in] void
 */
void Gimbal_Auto_aim_Cal(void)
{
    gimbal_controller.pc_dt_yaw = find_min_angle(pc_recv_data.yaw_setpoint, gimbal_controller.gyro_yaw_angle);
    if (pc_recv_data.detect_number == 0 || fabs(gimbal_controller.pc_dt_yaw) > 70.0f || global_debugger.pc_receive_debugger.state != ON) // 没有识别到目标或者目标角度过大，或者pc掉线
    {
        // 无PC数据处理
        Gimbal_Act_Cal();
    }
    else if (pc_recv_data.mode_select == 0x11)
    {
        // 位置控制
        if (remote_controller.auto_arm == 1)
        {
            gimbal_controller.target_pitch_angle = pc_recv_data.pitch_setpoint;
            gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + gimbal_controller.pc_dt_yaw;
        }
        Gimbal_Act_Cal();
    }
    else if (pc_recv_data.mode_select == 0x22 && remote_controller.auto_arm == 1)
    {
        // 前馈控制
        Gimbal_PC_Cal();
    }
    else
    {
        // 手动控制
        Gimbal_Act_Cal();
    }
}

/**------------发射机构控制--------------*/

/**
 * @brief 发射机构清空下电
 * @param[in] void
 */
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

/**
 * @brief 开火控制函数
 * @param[in] void
 * @return void
 */
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
				//更新预测弹量，防止剩余弹量不更新，打不了弹------
				Toggle_FilterBulletData(chassis_pack_get_1.shoot_avaiable , chassis_pack_get_1.heat_cooling/100.0f);
				Toggle_BulletPrediction();
				//------------------------------------------------
        Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
    }
    else if (remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE)
    {
        remote_controller.single_shoot_flag = FALSE; // 自瞄连发
        if (pc_recv_data.shoot_flag == 1)
        {
            Toggle_Fire(chassis_pack_get_1.shoot_avaiable , chassis_pack_get_1.heat_cooling/100.0f);
        }
        else
        {
            Toggle_Calculate(TOGGLE_STOP, 0.0f);
        }
    }
    else // 手动开火
    {
        remote_controller.single_shoot_flag = FALSE;
        Toggle_Fire(chassis_pack_get_1.shoot_avaiable , chassis_pack_get_1.heat_cooling/100.0f);
    }
}

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

/**------------云台数据更新--------------*/

/**
 * @brief 更新云台数据
 * @param[in] void
 * @return void
 */
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

/**------------电机执行控制--------------*/

/**
 * @brief 电机控制函数，执行发送最后计算的力矩
 * @param[in] void
 * @return void
 */
void execute_func(void)
{
    int8_t send_data[2][8] = {0};
    int8_t dji_motors_send_data_can1[8] = {0};
    int8_t dji_motors_send_data_can2[8] = {0};

    // 单独接电机进行测试时，会进行保护，tff不会赋值
    if (global_debugger.pitch_debugger.state == ON && global_debugger.yaw_debugger.state == ON)
    {
        if (remote_controller.gimbal_action == GIMBAL_POWER_DOWN)
        {
            // 设置为0力矩
            gimbal_controller.DM_Pitch_Motor.t_ff = 0;
            DM_Motor_Control(&gimbal_controller.DM_Pitch_Motor, send_data[PITCH_MOTOR], DM_DISABLE); // 失能有阻力，防止下降磕头

            gimbal_controller.DM_Yaw_Motor.t_ff = 0;
            DM_Motor_Control(&gimbal_controller.DM_Yaw_Motor, send_data[YAW_MOTOR], DM_DISABLE);
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
        // 少电机，全部下电，留个绿灯，方便判断是那个电机掉了
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

    gimbal_controller.if_spin_reverse = 1;

    // 摩擦轮PID初始化
    FrictionWheel_Init();
    // 云台PID初始化
    GimbalMotorInit();
    GimbalPidInit();
    /* 系统辨识以及测试 */
    GimbalSystemID_Init(&gimbal_controller);
    // 拨弹电机PID初始化
    Toggle_Init();
    // 丝杆电机初始化
    LiftPidInit();

    // USB初始化会默认放到freertos的第一个task里面,一定要确保调用了,否则无法成功初始化
    MX_USB_DEVICE_Init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    int index = 0;
    xLastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        Gimbal_Msg_Update();
        gimbal_controller.delta_t = DWT_GetDeltaT(&gimbal_controller.last_cnt);
        Gimbal_ErrorAngle();
        Gimbal_Return(&gimbal_controller, &remote_controller);

#if GIMBAL_SYSID
        if (!gimbal_sysid.yaw.sysid_done || !gimbal_sysid.pitch.sysid_done)
        {
            GimbalSystemID_Run();
        }
#endif
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
            Gimbal_Auto_aim_Cal();
            break;
        case GIMBAL_BIG_BUFF_MODE:
            Gimbal_Auto_aim_Cal();
            break;
        case GIMBAL_AUTO_ATM_TEST_MODE:
            Gimbal_Auto_aim_Cal();
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

        Lifting_Control(); // 控制丝杆升降

        if (index % 2 == 0) // 500hz
        {
            SendtoPC();     // 将信息发送给上位机
            execute_func(); // 执行函数
        }
        index++;

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
