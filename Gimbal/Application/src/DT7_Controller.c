#include "DT7_Controller.h"

// 检查右拨杆是否有值
uint8_t on_right_lr = 0;
uint8_t on_right_ud = 0;

void DT7_GimbalControl(float delta_t)
{
    // 过洞姿态下，直接发yaw速度，不动云台
    if (remote_controller.gimbal_position == DOWN)
    {
        if (abs(dt7_remote.ch[LEFT_CH_LR] - CH_MIDDLE) > 50)
        {
            chassis_solver.chassis_speed_w = -(float)(dt7_remote.ch[LEFT_CH_LR] - CH_MIDDLE) / CH_RANGE;
        }
    }
    // 根据遥控器输入控制云台
    // yaw
    if (abs(dt7_remote.ch[LEFT_CH_LR] - CH_MIDDLE) > 50) // 过零检测
    {
        gimbal_controller.target_yaw_angle -= (dt7_remote.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * delta_t;
    }

    // pitch
    if (abs(dt7_remote.ch[LEFT_CH_UD] - CH_MIDDLE) > 50) // 过零检测
    {
        gimbal_controller.target_pitch_angle += (dt7_remote.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * delta_t;
    }
}

void DT7_ChassisControl(void)
{

    // 根据遥控器输入控制底盘
    // 前后x
    if (abs(dt7_remote.ch[RIGHT_CH_UD] - CH_MIDDLE) > 50) // 过零检测
    {
        on_right_ud = 1;
        chassis_solver.chassis_speed_x = (float)(dt7_remote.ch[RIGHT_CH_UD] - CH_MIDDLE) / CH_RANGE;
    }
    else // 松手保护
    {
        if (on_right_ud == 1)
        {
            chassis_solver.chassis_speed_x = 0;
        }
        on_right_ud = 0;
    }
    // 左右y
    if (abs(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) > 50) // 过零检测
    {
        on_right_lr = 1;
        chassis_solver.chassis_speed_y = (float)(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) / CH_RANGE;
    }
    else // 松手保护
    {
        if (on_right_lr == 1)
        {
            chassis_solver.chassis_speed_y = 0;
        }
        on_right_lr = 0;
    }
    // 旋转w
    if (remote_controller.chassis_mode_action == CV_ROTATE)
    {
        chassis_solver.chassis_speed_w = 1.0f;
    }
    else
    {
        chassis_solver.chassis_speed_w = 0.0f;
    }
}

void DT7_Update(float delta_t)
{
    bool sw_changed = 0;
    if (dt7_remote.Previous_rc_Right_SW != dt7_remote.s[RIGHT_SW] || dt7_remote.Previous_rc_Left_SW != dt7_remote.s[LEFT_SW])
    {
        sw_changed = 1;
    }

    if (global_debugger.DT7_debugger.state != ON)
    {
        dt7_remote.Previous_rc_Left_SW = dt7_remote.s[LEFT_SW];
        dt7_remote.Previous_rc_Right_SW = dt7_remote.s[RIGHT_SW];
        setAllModeOff();
        return;
    }
    switch (dt7_remote.s[RIGHT_SW])
    {
    case Down:
        switch (dt7_remote.s[LEFT_SW])
        {
        case Down:
            setAllModeOff();
            break;
        case Mid:
            // 辅瞄模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setChassisModeAction(NOT_CONTROL_MODE);
                setGimbalAction(GIMBAL_AUTO_AIM_MODE);
                setShootAction(SHOOT_FIRE_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }

            if (global_debugger.pc_receive_debugger.state != ON)
            {
                DT7_GimbalControl(delta_t);
            }
            if (abs(dt7_remote.ch[RIGHT_LR] - CH_MIDDLE) > 150)
            {
                toggle_controller.is_shoot = 1;
            }
            else
            {
                toggle_controller.is_shoot = 0;
            }
            break;
        case Up:
            // 过洞测试模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(DOWN);
                Gimbal_Return();
            }

            //应该改为转向完成之前不跟随，要不然下降完成之前一直动不了
            if (fabsf(gimbal_controller.min_angle_err) > 0.5f)
            {
                
                setChassisModeAction(NOT_CONTROL_MODE);
            }
            else
            {
                setChassisModeAction(FOLLOW_GIMBAL);
                DT7_ChassisControl();
            }

            DT7_GimbalControl(delta_t); // 顺序必须在底盘控制后面，否则speedw被覆盖
            break;
        default:
            setAllModeOff();
            break;
        }
        break;

    case Mid:
        // 切换键鼠控制
        initRemoteControl(KEY_MOUSE);
        break;
    case Up:
        switch (dt7_remote.s[LEFT_SW])
        {
        case Down:
            // 云台正常运动
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setChassisModeAction(FOLLOW_GIMBAL);
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }

            DT7_GimbalControl(delta_t);
            DT7_ChassisControl();
            break;
        case Mid:
// 分模式进行测试
#if TEST_MODE_SELECT1 == SELF_ROTATE_MODE
            // 左右小陀螺模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setChassisModeAction(CV_ROTATE);
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }

            DT7_GimbalControl(delta_t);
            DT7_ChassisControl();

            if (remote_controller.chassis_mode_action == CV_ROTATE)
            {
                //  陀螺变向
                if (dt7_remote.Previous_rc_Left_SW != Mid)
                    chassis_solver.Rotate_Counter++;
                if (chassis_solver.Rotate_Counter % 2 == 1)
                {
                    chassis_solver.chassis_speed_w *= -1;
                    if (chassis_solver.chassis_speed_w < 0)
                    {
                        chassis_solver.chassis_speed_w = chassis_solver.chassis_speed_w;
                    }
                    else if (chassis_solver.chassis_speed_w > 0)
                    {
                        chassis_solver.chassis_speed_w = chassis_solver.chassis_speed_w;
                    }
                }
            }
            else
            {
                chassis_solver.chassis_speed_w = 0.0f;
            }
            break;
#endif
        case Up:
#if TEST_MODE_SELECT2 == SHOOT_MODE
            // 检录打弹模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setChassisModeAction(NOT_CONTROL_MODE);
                setShootAction(SHOOT_FIRE_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }
            DT7_GimbalControl(delta_t);
            if (abs(dt7_remote.ch[RIGHT_LR] - CH_MIDDLE) > 150)
            {
                toggle_controller.is_shoot = 1;
            }
            else
            {
                toggle_controller.is_shoot = 0;
            }

            break;
#endif
        default:
            setAllModeOff();
            break;
        }
        break;
    default:
        setAllModeOff();
        break;
    }

    dt7_remote.Previous_rc_Left_SW = dt7_remote.s[LEFT_SW];
    dt7_remote.Previous_rc_Right_SW = dt7_remote.s[RIGHT_SW];
}
