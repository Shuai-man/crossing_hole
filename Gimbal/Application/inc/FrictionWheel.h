#ifndef _FRICTION_WHEEL_H
#define _FRICTION_WHEEL_H

#include "pid.h"
#include "M2006.h"
#include "M3508.h"
#include "ChassisGet.h" 

#define LEFT_FRICTION_WHEEL 0
#define RIGHT_FRICTION_WHEEL 1

#define SPEED_RATIO 1520 //速度对应到摩擦轮电流值

#define BULLET_17MM_15MS_SPEED_L 24500
#define BULLET_17MM_18MS_SPEED_L 27300
#define BULLET_17MM_24MS_SPEED 36500  //24弹速
#define BULLET_17MM_28MS_SPEED_L 40000
//#define BULLET_TEST				 43000
#define BULLET_TEST				 37000
#define BULLET_17MM_15MS_SPEED_R BULLET_17MM_15MS_SPEED_L
#define BULLET_17MM_18MS_SPEED_R BULLET_17MM_18MS_SPEED_L
#define BULLET_17MM_26MS_SPEED_R BULLET_17MM_26MS_SPEED_L
#define BULLET_17MM_30MS_SPEED_R BULLET_17MM_30MS_SPEED_L

typedef struct Calculate_Acc
{
	uint32_t dwt;
	float last_speed_l;
	float last_speed_r;

	float acc_l;
	float acc_r;
} Calculate_Acc_t;

/*  摩擦轮结构体  */
typedef struct FrictionWheel_t
{
    PID_t PidFrictionSpeed[2];
		Feedforward_t feedforward[2];
    M3508_Recv friction_motor_recv[2];
    M3508_Info friction_motor_msgs[2];
    float send_to_motor_current[2];

		Calculate_Acc_t calculate_acc;
	
		float shoot_speed;
		float last_shoot_Speed;
	
		float friction_speed;
		float over_speed;					 //超速弹速
		uint8_t over_times;
		float lower_speed;
		uint8_t lower_speed_times; //掉速次数
    float set_speed;

} FrictionWheel_t;

extern FrictionWheel_t friction_wheels;

void FrictionWheel_Init(void);
void FrictionWheel_Set(float speed); // 度/s
void setFrictionSpeed(void);

#endif // !_FRICTION_WHEEL_H
