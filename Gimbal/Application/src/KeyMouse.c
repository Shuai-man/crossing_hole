/**
 ******************************************************************************
 * @file    KeyMouse.c
 * @brief   键鼠更新
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

// 移动超功率；缓启动平移没效果
// 小陀螺后跟随零点发送变化

#include "KeyMouse.h"

ChassisSolver chassis_solver;

void State_Clear(void)
{
    // 清空状态
    setGameModeAction(OFF_MODE);
}

void KeyMouse_Init(void)
{
    // 初始化微分器
    // 底盘速度，低敏
    speed_smoother_init(&chassis_solver.speed_x_smoother, 0.10f, 0.6f, 2.0f);
    speed_smoother_init(&chassis_solver.speed_y_smoother, 0.10f, 0.6f, 2.0f);
    speed_smoother_init(&chassis_solver.speed_w_smoother, 0.10f, 0.6f, 2.0f);

    // 鼠标速度，高敏
    TD_Init(&chassis_solver.mouse_x_td, 50000, 0.005);
    TD_Init(&chassis_solver.mouse_y_td, 50000, 0.005);
}

void KeyMouseUpdate(ChassisSolver *infantry)
{
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
                setGameModeAction(TEST_MODE);
                VTM_Fire();
            }
        }
    }

    chassis_solver.speed_state = NORMAL_SPEED_STATE; // 普通速度
    // 按键检测
    uint16_t keyValue = remote_controller.dji_remote.keyValue;
    remote_controller.dji_remote.keyChangeOn = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) & remote_controller.dji_remote.keyValue;

    remote_controller.dji_remote.keyChangeOff = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) & remote_controller.dji_remote.last_keyValue;

    remote_controller.dji_remote.last_keyValue = keyValue;

    /* 按键操作 */

    float speed_x = 0.0f, speed_y = 0.0f, speed_w = 0.0f;
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
            if (remote_controller.game_mode == GAME_MODE && remote_controller.gimbal_position == UP)
            {
                setChassisModeAction(CV_ROTATE);
                speed_w = 0.5f;
            }
            break;
        case KEY_D:
            speed_y = 0.4f; // 建议加个缓启动
            break;
        case KEY_A:
            speed_y = -0.4f;
            break;
        case KEY_S:
            speed_x = -0.4f; // 后退太灵敏，衰减一下
            break;
        case KEY_W:
            speed_x = 0.4f;
            break;
        default:
            break;
        }

        key_and = remote_controller.dji_remote.keyChangeOn & i;
        // 按键按下生效
        switch (key_and) // 按键上升沿
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
            setGimbalAction(GIMBAL_ACT_MODE);
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
            setGimbalAction(GIMBAL_SMALL_BUFF_MODE);
            break;
        case KEY_C:
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
                speed_w = 0.0f;
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

    // 鼠标操作
    TD_Calculate(&chassis_solver.mouse_x_td, remote_controller.dji_remote.mouse.x);
    TD_Calculate(&chassis_solver.mouse_y_td, remote_controller.dji_remote.mouse.y);
    gimbal_controller.target_yaw_angle -= chassis_solver.mouse_x_td.x * MOUSE_YAW_SENSITIVITY;
    gimbal_controller.target_pitch_angle += chassis_solver.mouse_y_td.x * MOUSE_PITCH_SENSITIVITY;
    gimbal_controller.target_pitch_angle += remote_controller.dji_remote.mouse.z * MOUSE_SCROLL_SENSITIVITY;

    if (remote_controller.gimbal_position == DOWN)
    {
        chassis_solver.chassis_speed_w = gimbal_controller.pos_yaw_td.dx * MOUSE_YAW_SPEED;
    }
    else
    {
        chassis_solver.chassis_speed_w = speed_smoother_update(&chassis_solver.speed_w_smoother, speed_w, chassis_solver.delta_t);
    }
    chassis_solver.chassis_speed_x = speed_smoother_update(&chassis_solver.speed_x_smoother, speed_x, chassis_solver.delta_t);
    chassis_solver.chassis_speed_y = speed_smoother_update(&chassis_solver.speed_y_smoother, speed_y, chassis_solver.delta_t);

    // 鼠标左键检测
    unsigned char press_l = remote_controller.dji_remote.mouse.press_l;
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

    unsigned char press_r = remote_controller.dji_remote.mouse.press_r;
    remote_controller.dji_remote.mouse.mouseChangeOn_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                         press_r;
    remote_controller.dji_remote.mouse.mouseChangeOff_r = (remote_controller.dji_remote.mouse.last_press_r ^ press_r) &
                                                          remote_controller.dji_remote.mouse.last_press_r;
    remote_controller.dji_remote.mouse.last_press_r = press_r;

    // 打算单发和双发改成中键切换，不用按键切换，这样逻辑就不用那么复杂了
    if (remote_controller.dji_remote.mouse.mouseChangeOn_r)
    {
        remote_controller.auto_arm =1;
        // 利用上升沿做初始化，防止重复赋值
        if (remote_controller.gimbal_action == GIMBAL_SMALL_BUFF_MODE)
        {
            pc_send_data.mode_want = SMALL_BUFF;
        }
        else if (remote_controller.gimbal_action == GIMBAL_BIG_BUFF_MODE)
        {
            pc_send_data.mode_want = BIG_BUFF;
        }
        else
        {
            pc_send_data.mode_want = STD_AUTO_AIM;
            setGimbalAction(GIMBAL_AUTO_AIM_MODE);
        }
    }
    else if (remote_controller.dji_remote.mouse.mouseChangeOff_r)
    {
        remote_controller.auto_arm =0;
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
        KeyMouseUpdate(infantry);
        break;
    default:
        break;
    }
}

void speed_smoother_init(SpeedSmoother *sm, float start_speed, float accel, float decel)
{
    sm->current_speed = 0.0f;
    sm->start_speed = start_speed;
    sm->accel = accel;
    sm->decel = decel;
}

float speed_smoother_update(SpeedSmoother *sm, float target, float dt)
{
    // -------- 1. 防御性处理：防止 dt 异常 ----------
    if (dt > 0.1f)
        dt = 0.005f; // 若卡顿，使用默认步长

    // -------- 2. 目标为 0（松手）：执行减速停止 ----------
    if (fabsf(target) < 0.001f)
    {
        // 如果当前速度已经极小，直接归零
        if (fabsf(sm->current_speed) < 0.001f)
        {
            sm->current_speed = 0.0f;
        }
        else
        {
            float step = sm->decel * dt;
            if (sm->current_speed > 0)
            {
                sm->current_speed = (sm->current_speed - step > 0) ? (sm->current_speed - step) : 0.0f;
            }
            else
            {
                sm->current_speed = (sm->current_speed + step < 0) ? (sm->current_speed + step) : 0.0f;
            }
        }
        return sm->current_speed;
    }

    // -------- 3. 目标非 0：处理起步和加速 ----------
    // 3.1 判断当前是否处于“静止”或“接近静止”
    if (fabsf(sm->current_speed) < 0.001f)
    {
        // 3.1.1 目标速度 <= 冲击阈值：直接透传（保证微操不窜）
        if (fabsf(target) <= sm->start_speed)
        {
            sm->current_speed = target;
        }
        else
        {
            // 3.1.2 目标速度 > 冲击阈值：瞬间给一个冲击速度突破静摩擦
            sm->current_speed = (target > 0) ? sm->start_speed : -sm->start_speed;
        }
        return sm->current_speed;
    }

    // 3.2 车辆已在运动中：执行加速度限幅（速率限制）
    float delta = target - sm->current_speed;
    float max_delta = sm->accel * dt;

    // 限幅变化量（正负都限制）
    if (delta > max_delta)
        delta = max_delta;
    if (delta < -max_delta)
        delta = -max_delta;

    sm->current_speed += delta;

    // 3.3 防止超调
    if ((delta > 0 && sm->current_speed > target) || (delta < 0 && sm->current_speed < target))
    {
        sm->current_speed = target;
    }

    return sm->current_speed;
}
