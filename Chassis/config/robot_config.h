#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H

/*  机器人宏定义列举 */
#define NIU_MO_SON 0       // 舵轮
#define CHEN_JING_YUAN 1   // 2025 7.7  新麦轮
#define NIUNIU 2           // 2025.5.6  新全向
#define QI_TIAN_DA_SHENG 3 // 2024.7.23 老全向

#define ROBOT  CHEN_JING_YUAN
//#define ROBOT  NIUNIU

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
#if ROBOT == CHEN_JING_YUAN
#define GIMBAL_FOLLOW_ZERO 9.51965332f  // 底盘跟随角度零点
#define GIMBAL_MOTOR_SIGN -1    // 云台电机方向，以逆时针为正
#endif
void setRobotType(void);

#endif
