#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H

/*  机器人宏定义列举 */
#define OLD 0
#define NEW 1

#define ROBOT OLD


typedef enum CHASSIS_TYPE
{ 
    STEER_WHEEL,   // 舵轮
    MECANUM_WHEEL, // 麦克纳姆轮
    OMNI_WHEEL,    // 全向轮
} CHASSIS_TYPE;

// 麦轮参数
#define MECANUM_WIDTH 0.15f  // 麦轮宽
#define MECANUM_LENGTH 0.20f // 麦轮长

// yaw轴电机类型
typedef enum YAW_MOTOR_TYPE
{
    YAW_GM6020,
    YAW_DM_MOTOR
} YAW_MOTOR_TYPE;


//机械拆头后需要重新标零点，否则可能前后左右反过来
#if ROBOT == OLD
#define GIMBAL_FOLLOW_ZERO 9.51965332f  // 底盘跟随角度零点
#elif ROBOT == NEW
#define GIMBAL_FOLLOW_ZERO 105.309448f
#endif

#define GIMBAL_MOTOR_SIGN -1    // 云台电机方向，以逆时针为正
void setRobotType(void);
#endif
