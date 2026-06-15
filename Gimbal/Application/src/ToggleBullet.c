#include "ToggleBullet.h"

ToggleController toggle_controller;

/* 根据机器人等级以及增益来确定射频 */
void Toggle_SelectShootFreq(void)
{
    if (toggle_controller.remaining_bullets <= BULLET_LOW_THRESHOLD)
    {
        toggle_controller.freq = FREQ_LOW;
    }
    else
    {
        toggle_controller.freq = FREQ_NORMAL;
    }
    toggle_controller.freq_time = 1.0f / toggle_controller.freq;
}

void Toggle_Init(void)
{
    PID_Init(&toggle_controller.toggle_pos_pid, 800.0f, 0, 0, 10, 0, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&toggle_controller.toggle_speed_pid, C610_MAX_CURRENT, 3.0f, 0, 5.0f, 0.0, 0.2f, 50, 100, 0, 0, 1, Integral_Limit | Trapezoid_Intergral | ChangingIntegrationRate);
}

/**
 * @brief 拨弹电机控制计算
 * @param[in] control_mode 控制模式
 * @param[in] set_point 目的位置(位置控制)，目标速度(速度控制)
 */
void Toggle_Calculate(enum TOGGLE_CONTROL_MODE control_mode, float set_point)
{
    if (control_mode == TOGGLE_SPEED)
    {
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = set_point;
        PID_Clear(&toggle_controller.toggle_pos_pid);
        toggle_controller.send_current = PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, set_point);
    }
    else if (control_mode == TOGGLE_POS)
    {
        toggle_controller.set_speed = PID_Calculate(&toggle_controller.toggle_pos_pid, toggle_controller.toggle_info.angle, set_point);
        toggle_controller.send_current = PID_Calculate(&toggle_controller.toggle_speed_pid, toggle_controller.toggle_info.speed, toggle_controller.set_speed);
    }
    else
    {
        PID_Clear(&toggle_controller.toggle_pos_pid);
        PID_Clear(&toggle_controller.toggle_speed_pid);
        toggle_controller.set_pos = toggle_controller.toggle_info.angle;
        toggle_controller.set_speed = 0;
        toggle_controller.send_current = 0;
    }
}

/**
 * @brief 拨弹电机推动N格
 * @param[in] controller 拨弹控制器指针
 * @param[in] num 推动格数（正数）
 */
void Toggle_AddGrid(ToggleController *controller, uint8_t num)
{
    if (controller->remaining_bullets > num)
    {
        controller->set_pos = controller->set_pos + ONE_GRID_ANGLE * num * SIGN_ROTATE;
        controller->predict_bullets -= num;
        if (controller->predict_bullets < 0)
        {
            controller->predict_bullets = 0;
        }
    }
    else
    {
        // 清空目标值，放置继续拨弹
        controller->set_pos = controller->toggle_info.angle;
    }
}

/* 过滤裁判系统数据 */
static void Toggle_FilterBulletData(uint8_t raw)
{
    // 裁判系统超热量会回传255，需要过滤
    if (raw > BULLET_MAX_VALID)
    {
        toggle_controller.receive_bullets = 0;
    }
    else if(referee_check.is_connected ==0)//裁判系统离线
    {
        toggle_controller.receive_bullets = BULLET_DEFAULT_REMAINING;
    }
    else//裁判系统在线
    {
        toggle_controller.receive_bullets = raw;
    }
}

/* 更新弹量预测 */
static void Toggle_BulletPrediction(void)
{
    // 不开火时清空预测值、剩余量和时间累计
    if (toggle_controller.is_shoot == 0)
    {
        toggle_controller.predict_bullets = toggle_controller.receive_bullets;
        toggle_controller.remaining_bullets = toggle_controller.receive_bullets;
        toggle_controller.dt_accumulated = 0;
        Toggle_Calculate(TOGGLE_STOP, 0.0f);
        return;
    }
    // 开火
    // 热量恢复，更新剩余弹量，避免预测弹量用完打不出弹
    if (toggle_controller.last_receive_bullets >= toggle_controller.receive_bullets)
    {
        toggle_controller.predict_bullets = toggle_controller.receive_bullets;
        toggle_controller.remaining_bullets = toggle_controller.receive_bullets;
    }
    else
    {
        toggle_controller.remaining_bullets = toggle_controller.predict_bullets;
    }
}

/* 拨弹间隔计算 */
static bool Toggle_Scheduler(void)
{
    // 输出拨盘目标
    if (toggle_controller.is_shoot == 0)
    {
        // 清空时间累计
        toggle_controller.dt_accumulated = 0;
        return false;
    }
    toggle_controller.dt_current = DWT_GetDeltaT(&toggle_controller.current_cnt);
    toggle_controller.dt_accumulated += toggle_controller.dt_current;

    if (toggle_controller.dt_accumulated >= toggle_controller.freq_time)
    {
        toggle_controller.dt_accumulated = 0;
        Toggle_AddGrid(&toggle_controller, 1);
    }
    return true;
}

/* 控制拨弹电机 */
void Toggle_Control(uint8_t is_run)
{
    // 由于拨弹电机的控制需要时间，所以需要循环执行
    if (!is_run)
    {
        Toggle_Calculate(TOGGLE_STOP, 0.0f);
    }
    else
    {
        // 缺点是没弹的时候拨盘会原地抖动
        Toggle_Calculate(TOGGLE_POS, toggle_controller.set_pos);
    }
}

/* 拨弹主函数 */
void Toggle_Fire(uint8_t receive_bullets)
{
    Toggle_FilterBulletData(receive_bullets); // 过滤数据
    Toggle_BulletPrediction();                // 计算剩余弹量
    bool is_run = Toggle_Scheduler();         // 拨弹间隔计算
    Toggle_Control(is_run);                   // 控制拨弹电机

    toggle_controller.last_receive_bullets = toggle_controller.receive_bullets;
}
