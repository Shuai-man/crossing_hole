#include "DT7_Controller.h"

#include "stdbool.h"
#include "bsp_DT7.h"
#include "KeyMouse.h"
#include "remote_control.h"
#include "Gimbal.h"
#include "ToggleBullet.h"
#include "pc_serial.h"

void DT7_GimbalControl(float delta_t)
{
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
        chassis_solver.chassis_speed_x = (float)(dt7_remote.ch[RIGHT_CH_UD] - CH_MIDDLE) / CH_RANGE;
    }
    else // 松手保护
    {
        chassis_solver.chassis_speed_x = 0;
    }
    // 左右y
    if (abs(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) > 50) // 过零检测
    {
        chassis_solver.chassis_speed_y = (float)(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) / CH_RANGE;
    }
    else // 松手保护
    {
        chassis_solver.chassis_speed_y = 0;
    }
    // 旋转w
    if (remote_controller.gimbal_position == DOWN) // 过洞姿态下，不允许小陀螺
    {
        chassis_solver.chassis_speed_w = 0.0f;
    }
    else if (remote_controller.chassis_mode_action == CV_ROTATE) // 必须与过洞模式并列，否则硬件干涉
    {
        chassis_solver.chassis_speed_w = 0.5f;
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
                setGimbalAction(GIMBAL_AUTO_ATM_TEST_MODE);
                setShootAction(SHOOT_FIRE_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }

            if (global_debugger.pc_receive_debugger.state != ON)
            {
                DT7_GimbalControl(delta_t);
            }
            if (abs(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) > 150)
            {
                toggle_controller.is_shoot = 1;
                pc_send_data.mode_want = STD_AUTO_AIM;
            }
            else
            {
                toggle_controller.is_shoot = 0;
                pc_send_data.mode_want = NOT_USE_AIM;
            }
            break;
        case Up:
            // 过洞测试模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setChassisModeAction(FOLLOW_GIMBAL);
                setShootAction(SHOOT_POWER_DOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(DOWN);
            }
            DT7_GimbalControl(delta_t);
            DT7_ChassisControl();

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
                }
            }
            else
            {
                chassis_solver.chassis_speed_w = 0.0f;
            }
            break;
        case Up:
            // 检录打弹模式
            if (sw_changed)
            {
                setRobotState(CONTROL_MODE);
                setShootAction(SHOOT_FIRE_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);
                setGimbalPosition(UP);
            }
            setChassisModeAction(NOT_CONTROL_MODE);
            DT7_GimbalControl(delta_t);
            if (abs(dt7_remote.ch[RIGHT_CH_LR] - CH_MIDDLE) > 150)
            {
                toggle_controller.is_shoot = 1;
            }
            else
            {
                toggle_controller.is_shoot = 0;
            }

            break;
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
    if (remote_controller.gimbal_action == GIMBAL_AUTO_ATM_TEST_MODE)
    {
        remote_controller.auto_arm = 1;
    }
    else
    {
        remote_controller.auto_arm = 0;
    }
}
