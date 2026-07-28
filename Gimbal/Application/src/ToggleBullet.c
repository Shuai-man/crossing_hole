#include "ToggleBullet.h"

#include "ChassisGet.h"
#include "bsp_dwt.h"

ToggleController toggle_controller;

/**---------剩余弹丸接收----------*/

/**
 * @brief 过滤裁判系统数据
 * @param[in] bullets 原始数据
 */
void Toggle_FilterBulletData(uint8_t bullets , uint8_t heat_cooling)
{
    // 裁判系统超热量会回传255，需要过滤
    if (bullets > BULLET_MAX_VALID)
    {
        toggle_controller.receive_bullets = 0;
    }
    else if (referee_check.is_connected == 0) // 裁判系统离线
    {
        toggle_controller.receive_bullets = BULLET_DEFAULT_REMAINING;
    }
    else // 裁判系统在线
    {
        toggle_controller.receive_bullets = bullets;
    }
    if(heat_cooling!=0)
    {
        toggle_controller.heat_cooling = heat_cooling;
    }
    else
    {
        toggle_controller.heat_cooling = 2.0f;//最慢2s恢复1发
    }
}

/**---------剩余弹量预测----------*/

/**
 * @brief 未确认弹量超时检测
 * @param[in] dt 时间间隔
 *弹丸超时未确认，认为已经通过冷却恢复
 */
void Toggle_UpdatePendingTimeout(float dt)
{
    uint8_t timeout_bullets = 0;

    for (uint8_t i = 0; i < toggle_controller.pending_bullets; i++)
    {
        toggle_controller.pending_bullet_time[i] += dt;
    }

    while (timeout_bullets < toggle_controller.pending_bullets &&
           toggle_controller.pending_bullet_time[timeout_bullets] >= toggle_controller.heat_cooling)
    {
        timeout_bullets++;
    }

    if (timeout_bullets > 0)
    {
        Toggle_RemovePendingBullets(timeout_bullets);
        Toggle_UpdateRemainingByPending();
    }
}

/**
 * @brief 未确认弹量更新
 * @param[in] num 已确认减少的弹量 或 超时已恢复的弹量
 */
void Toggle_RemovePendingBullets(uint8_t num)
{
    if (num >= toggle_controller.pending_bullets)
    {
        toggle_controller.pending_bullets = 0;
        return;
    }

    toggle_controller.pending_bullets -= num;
    for (uint8_t i = 0; i < toggle_controller.pending_bullets; i++)
    {
        toggle_controller.pending_bullet_time[i] = toggle_controller.pending_bullet_time[i + num];
    }
}

/**
 * @brief 剩余弹量更新
 * 剩余弹量 = 接收发弹量 - 未确认弹量
 * 必须减去未确认弹量，因为打出去的瞬间是检测不到的
 */
void Toggle_UpdateRemainingByPending(void)
{
    if (toggle_controller.receive_bullets > toggle_controller.pending_bullets)
    {
        toggle_controller.remaining_bullets = toggle_controller.receive_bullets - toggle_controller.pending_bullets;
    }
    else
    {
        toggle_controller.remaining_bullets = 0;
    }
}

/**
 * @brief 剩余弹量预测主函数
 */
void Toggle_BulletPrediction(void)
{
    // 不开火时清空预测值、剩余量和时间累计
    if (toggle_controller.is_shoot == 0)
    {
        toggle_controller.pending_bullets = 0;
        toggle_controller.remaining_bullets = toggle_controller.receive_bullets;
        toggle_controller.dt_accumulated = 0;
        Toggle_Calculate(TOGGLE_STOP, 0.0f);
        return;
    }

    // 弹量减少时更新未确认弹量
    if (toggle_controller.receive_bullets < toggle_controller.last_receive_bullets)
    {
        uint8_t confirmed_bullets = toggle_controller.last_receive_bullets - toggle_controller.receive_bullets;
        Toggle_RemovePendingBullets(confirmed_bullets);
    }

    // 实时更新剩余弹量
    Toggle_UpdateRemainingByPending();
}

/**---------拨弹电机控制----------*/

/**
 * @brief 初始化拨弹电机
 */
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
    if (controller->remaining_bullets >= num)
    {
        controller->set_pos = controller->set_pos + ONE_GRID_ANGLE * num * SIGN_ROTATE;
        for (uint8_t i = 0; i < num; i++)
        {
            if (controller->pending_bullets < PENDING_BULLET_MAX)
            {
                controller->pending_bullet_time[controller->pending_bullets] = 0.0f;
                controller->pending_bullets++;
            }
        }
        Toggle_UpdateRemainingByPending();
    }
    else
    {
        // 清空目标值，放置继续拨弹
        controller->set_pos = controller->toggle_info.angle;
    }
}

/**
 * @brief 根据剩余弹量来确定射频
 */
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

/**
 * @brief 定频率拨弹
 * @return 是否运行拨弹电机
 */
bool Toggle_Scheduler(void)
{
    //松手和剩余弹量不够时停止控制，防止抖动
    if (toggle_controller.is_shoot == 0 || toggle_controller.remaining_bullets<1.0f)
    {
        // 清空时间累计
        toggle_controller.dt_accumulated = 0;
        return false;
    }
    toggle_controller.dt_current = DWT_GetDeltaT(&toggle_controller.current_cnt);
    toggle_controller.dt_accumulated += toggle_controller.dt_current;
    Toggle_UpdatePendingTimeout(toggle_controller.dt_current);

    if (toggle_controller.dt_accumulated >= toggle_controller.freq_time)
    {
        toggle_controller.dt_accumulated = 0;
        Toggle_AddGrid(&toggle_controller, 1);
    }
    return true;
}

/**
 * @brief 控制拨弹电机
 * @param[in] is_run 是否运行拨弹电机
 */
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

/**
 * @brief 拨弹主函数
 * @param[in] receive_bullets 接收发弹量
 */
void Toggle_Fire(uint8_t receive_bullets, uint8_t heat_cooling)
{
    Toggle_FilterBulletData(receive_bullets, heat_cooling); // 过滤数据
    Toggle_BulletPrediction();                // 计算剩余弹量
    bool is_run = Toggle_Scheduler();         // 拨弹间隔计算
    Toggle_Control(is_run);                   // 控制拨弹电机

    toggle_controller.last_receive_bullets = toggle_controller.receive_bullets;
}
