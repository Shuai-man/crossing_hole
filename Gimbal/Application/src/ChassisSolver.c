/**
 ******************************************************************************
 * @file    ChassisSolver.c
 * @brief   底盘解算器
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChassisSolver.h"

RobotInfo robotInfo;
ChassisSolver chassis_solver;
Turn_Back turn_back;

void Turn(void) // 转头
{
    if (turn_back.turn_finish == 0)
    {
        float angle = (gimbal_controller.DM_Yaw_Motor.P_angle - GIMBAL_ANGLE_ZERO);
        gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + angle;
        turn_back.turn_finish = 1;
    }
}

void State_Clear(void)
{
    // 清空状态
    setGameModeAction(OFF_MODE);
    turn_back.turn_finish = 0;
}

TickType_t delta_ts = 0;

uint8_t fly_flag = 0;
uint8_t up_flag = 0;

uint16_t up_stair_cnt = 0;
uint8_t sbuff_flag = 0;
uint8_t is_upstairs = 0;

void DJIKeyMouseUpdate(ChassisSolver *infantry)
{

    //    static uint8_t sbuff_flag = 0;
    static TickType_t last_recv_ts = 0;
    TickType_t now_ts = xTaskGetTickCount();
    delta_ts = now_ts - last_recv_ts;
    last_recv_ts = now_ts;

    // 无遥控直接下电
    if ((global_debugger.DT7_debugger.state != ON) &&
        (global_debugger.VTM_debugger.state != ON))
    {
        setGameModeAction(OFF_MODE);
        return;
    }
    // DT7
    if (global_debugger.DT7_debugger.state == ON)
    {
        // 记得设置好接口可以跳转回遥控器模式
        if (dt7_remote.s[RIGHT_SW] != Mid)
        {
            initRemoteControl(REMOTE_CONTROL); // 遥控模式
            State_Clear();
        }
        else
        {
            // 模式设置
            if (dt7_remote.s[LEFT_SW] == Down) // 检录模式
            {
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGameModeAction(TEST_MODE);
                Turn();
            }
            else if (dt7_remote.s[LEFT_SW] == Mid) // 检录模式
            {
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGameModeAction(GAME_MODE);
            }
            else if (dt7_remote.s[LEFT_SW] == Up) // 比赛模式
            {
                setShootAction(SHOOT_FIRE_MODE);
                setGameModeAction(GAME_MODE);
            }
            else
            {
                setGameModeAction(OFF_MODE);
                infantry->chassis_speed_w = 0.00f; // rad /s
            }
        }
    }
    // VTM
    else if (global_debugger.VTM_debugger.state == ON)
    {

        // 记得设置好接口可以跳转回遥控器模式
        if (vtm_remote.gear != gear_N)
        {
            initRemoteControl(REMOTE_CONTROL); // 切换遥控模式
            VTM_Init();
            State_Clear();
        }
        else if (chassis_pack_get_1.alive_flag == 0)
        {
            // 死后需要重新按开始
            VTM_Init();
            State_Clear();
        }
        else
        {
            if (vtm_remote.sw[Pause] == 1)
            {
                State_Clear(); // 不能加VTM_Init 否则pause会重置
            }
            else if (vtm_remote.sw[Left_up] == 1) // 启动按键
            {
                setGameModeAction(GAME_MODE);
                VTM_Fire();
            }
            else
            {
                Turn();
                setGameModeAction(TEST_MODE);
                VTM_Fire();
            }
        }
    }

    chassis_solver.speed_state = NORMAL_SPEED_STATE; // 主动电容
    // 按键检测
    volatile uint16_t keyValue = remote_controller.dji_remote.keyValue;
    remote_controller.dji_remote.keyChangeOn = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) & remote_controller.dji_remote.keyValue;

    remote_controller.dji_remote.keyChangeOff = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) & remote_controller.dji_remote.last_keyValue;

    remote_controller.dji_remote.last_keyValue = keyValue;

    /* 按键操作 */
    //    if (gimbal_controller.turn_back_start_flag == FALSE && gimbal_controller.turn_back_finish_flag == TRUE)
    //    {
    //        gimbal_controller.turn_back_finish_flag = FALSE;
    //        setChassisModeAction(FOLLOW_GIMBAL);
    //    }

    float speed_x = 0.0f, speed_y = 0.0f;
    // 长按生效
    for (uint16_t i = 1; i > 0; i <<= 1)
    {
        uint16_t key_and = keyValue & i;
        switch (key_and) // 按键
        {
        case KEY_B:
            break;
        case KEY_V:
            break;
        case KEY_SHIFT:
            chassis_solver.speed_state = HIGH_SPEED_STATE;
            break;
        case KEY_CTRL:
            break;
        case KEY_Q:
            break;
        case KEY_E:
            break;
        case KEY_R:
            break;
        case KEY_F:
            break;
        case KEY_G:
            break;
        case KEY_Z:

            break;
        case KEY_X:
            break;
        case KEY_C:
            break;
        case KEY_D:
            speed_y = 0.5;
            break;
        case KEY_A:
            speed_y = -0.5;
            break;
        case KEY_S:
            speed_x = -0.5; // 后退太灵敏，衰减一下
            break;
        case KEY_W:
            speed_x = 0.5;
            break;
        default:
            break;
        }

        key_and = remote_controller.dji_remote.keyChangeOn & i;
        // 按键按下生效
        switch (key_and) // 按键上升沿
        {
        case KEY_B:
            //					if(remote_controller.game_mode == GAME_MODE)
            //					{
            //            setGimbalAction(GIMBAL_ACT_MODE);
            //            setChassisModeAction(BALANCE);
            //            if (gimbal_controller.turn_back_start_flag == FALSE)
            //            {
            //                if (gimbal_controller.chassis_direction == CHASSIS_BACK)
            //                {
            //                    gimbal_controller.target_yaw_angle = gimbal_controller.gyro_yaw_angle + 180.0f;
            //                }
            //            }
            //            gimbal_controller.turn_back_start_flag = TRUE;
            //
            //					}
            break;
        case KEY_V:
            break;
        case KEY_SHIFT:
            break;
        case KEY_CTRL:

            break;
        case KEY_Q:
            if (remote_controller.game_mode == GAME_MODE)
            {
                setGimbalPosition(UP);
            }
            break;
        case KEY_E:
            if (remote_controller.game_mode == GAME_MODE)
            {
                setGimbalPosition(DOWN);
            }
            break;
        case KEY_R:

            break;
        case KEY_F:
            friction_wheels.friction_speed += 0.5f;
            break;
        case KEY_G:
            friction_wheels.friction_speed -= 0.5f;
            break;
        case KEY_Z:

            break;
        case KEY_X:
            if (sbuff_flag == 0)
            {

                setGimbalAction(GIMBAL_SMALL_BUFF_MODE);
                sbuff_flag = 1;
            }
            else if (sbuff_flag == 1)
            {

                setGimbalAction(GIMBAL_ACT_MODE);
                sbuff_flag = 0;
            }
            break;
        case KEY_C:
            if (remote_controller.game_mode == GAME_MODE && remote_controller.gimbal_position == UP)
            {
                setChassisModeAction(CV_ROTATE);
                chassis_solver.chassis_speed_w = 1.0f;
            }

            break;
        case KEY_D:
            break;
        case KEY_A:
            break;
        case KEY_S:
            break;
        case KEY_W:
            break;
        default:
            break;
        }

        key_and = remote_controller.dji_remote.keyChangeOff & i;
        // 按键松开生效
        switch (key_and) // 按键下降沿
        {
        case KEY_B:
            break;
        case KEY_V:
            break;
        case KEY_SHIFT:
            break;
        case KEY_CTRL:
            break;
        case KEY_Q:
            //            setLegAction(RAW_LENGTH);
            break;
        case KEY_E:
            break;
        case KEY_R:
            break;
        case KEY_F:
            break;
        case KEY_G:
            break;
        case KEY_Z:
            break;
        case KEY_X:
            break;
        case KEY_C:
            if (remote_controller.game_mode == GAME_MODE)
            {
                setChassisModeAction(FOLLOW_GIMBAL);
                chassis_solver.chassis_speed_w = 0.0;
            }
            break;
        case KEY_D:
            break;
        case KEY_A:
            break;
        case KEY_S:
            break;
        case KEY_W:
            break;
        default:
            break;
        }
    }

    chassis_solver.chassis_speed_x = speed_x;
    chassis_solver.chassis_speed_y = speed_y;

    // 鼠标操作
    if (remote_controller.gimbal_action == GIMBAL_ACT_MODE ||
        remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE || remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
    {
        //        if (gimbal_controller.turn_back_start_flag == FALSE)
        //        {
        gimbal_controller.target_yaw_angle -= remote_controller.dji_remote.mouse.x * 0.005f;
        gimbal_controller.target_pitch_angle += remote_controller.dji_remote.mouse.y * 0.004;
        gimbal_controller.target_pitch_angle += remote_controller.dji_remote.mouse.z * 0.001;
        //        }
    }

    // 鼠标左键检测
    volatile unsigned char press_l = remote_controller.dji_remote.mouse.press_l;
    remote_controller.dji_remote.mouse.mouseChangeOn_l = (remote_controller.dji_remote.mouse.last_press_l ^ press_l) & press_l;

    remote_controller.dji_remote.mouse.mouseChangeOff_l = (remote_controller.dji_remote.mouse.last_press_l ^ press_l) & remote_controller.dji_remote.mouse.last_press_l;

    remote_controller.dji_remote.mouse.last_press_l = press_l;

    // 按键操作，连发情况
    if (press_l)
    {
        toggle_controller.is_shoot = TRUE;
    }
    else
    {
        toggle_controller.is_shoot = FALSE;
    }
    if (remote_controller.dji_remote.mouse.mouseChangeOn_l)
    {
        remote_controller.single_shoot_flag = TRUE;
    }

    volatile unsigned char press_r = remote_controller.dji_remote.mouse.press_r;
    remote_controller.dji_remote.mouse.mouseChangeOn_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                         press_r;
    remote_controller.dji_remote.mouse.mouseChangeOff_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                          remote_controller.dji_remote.mouse.last_press_r;
    remote_controller.dji_remote.mouse.last_press_r = press_r;

    if (remote_controller.dji_remote.mouse.mouseChangeOn_r)
    {
        // 开启辅瞄
        remote_controller.auto_arm = 1;

        // 利用上升沿做初始化，防止重复赋值
        if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
        {
            Small_Buff_Change(); // 小符
            // X键进入小符
        }
        else
        {
            // 右键进入打车
            Auto_GimbalPidChange(); // 打车
            setGimbalAction(GIMBAL_AUTO_AIM_MODE);
        }
    }
    else if (remote_controller.dji_remote.mouse.mouseChangeOff_r)
    {
        // 关闭辅瞄
        remote_controller.auto_arm = 0;

        GimbalPidChange();
        if (remote_controller.gimbal_action == GIMBAL_AUTO_AIM_MODE)
        {
            setGimbalAction(GIMBAL_ACT_MODE); // 打车没有按键退出，单独加判断
        }
    }
}

/**********************************************************
                        掉电模式
右下左下                    掉电状态
右下左中                    倒地自救测试模式
右下左上                    检录打弹模式（底盘下电）
                        键鼠模式（遥控器模式）
右中左下                    键鼠模式云台运动（正常运动模式）
右中左中                    键鼠模式正常运动（上台阶模式）
右中左上                    键鼠模式开摩擦轮（跳跃模式）
                        遥控器模式
右上左下                    正常运动模式
右上左中                    检录左右小陀螺模式
右上左上                    检录打弹模式（底盘上电）


WASD                    正常移动
c                       小陀螺 (按住小陀螺，松开停止)转速7rad/s
shift                   超电加速
E                       飞坡伸腿 (双击收腿，单击恢复正常腿长)
Q                       上台阶伸腿 (双击伸腿，单击恢复正常腿长)
R                       侧对 (双击进入侧对模式，单击恢复正常模式)
左键                  射击
右键                  辅瞄

**********************************************************/
void DJIRemoteUpdate(ChassisSolver *infantry)
{
    // DT7
    if (global_debugger.DT7_debugger.state == ON)
    {
        DT7_Update(infantry->delta_t);
    }
    else if (global_debugger.VTM_debugger.state == ON)
    {
        VTM_Update(infantry->delta_t);
    }
    else
    {
        setAllModeOff();
    }
}

/**
 * @brief 根据遥控器或者蓝牙更新控制状态
 * @param[in] infantry
 */
void get_control_info(ChassisSolver *infantry)
{
    switch (remote_controller.control_type)
    {
    case REMOTE_CONTROL:
        DJIRemoteUpdate(infantry);
        break;
    case KEY_MOUSE:
        DJIKeyMouseUpdate(infantry);
        break;
    default:
        break;
    }
}

/*解码函数*/
void InfoUnpack(ChassisGetPack_1 *pack)
{
    // 更新底盘数据相隔时间
    robotInfo.last_cnt = global_debugger.receive_chassis_debugger[0].last_cnt;
    robotInfo.delta_t = DWT_GetDeltaT(&robotInfo.last_cnt);

    robotInfo.yaw_last_cnt = global_debugger.yaw_debugger.last_cnt;
    robotInfo.yaw_delta_t = DWT_GetDeltaT(&robotInfo.yaw_last_cnt);

    robotInfo.pitch_last_cnt = global_debugger.pitch_debugger.last_cnt;
    robotInfo.pitch_delta_t = DWT_GetDeltaT(&robotInfo.pitch_last_cnt);

    // 如果当前云台已经超过了1S没有收到底盘数据，则自动将底盘初始化标志位置零
    if (robotInfo.delta_t > 1.0f)
    {
        robotInfo.if_communication_lost = TRUE;
        robotInfo.chassis_state = CHASSIS_SENSORS_OFF;
    }
    else
    {
        robotInfo.if_communication_lost = FALSE;
        robotInfo.chassis_state = CONTROLLING;
    }
}
