#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H

#include <string.h>
#include <stdint.h>
#include "main.h"
#include "can.h"


// 电机接收配置(注意不会影响发送)
//CAN2
// pitch 电机
#define PITCH_MOTOR_SLAVE_CAN_ID 0x0C  // pitch轴ID
#define PITCH_MOTOR_MASTER_CAN_ID 0x0D  // pitch轴ID  //换电机后要记得改
#define PITCH_LOSS_TIME	0.0025f //s
#define PITCH_MOTOR_CAN &hcan2

// 摩擦轮电机
#define LEFT_FRICTION_WHEEL_CAN_ID 0x207  // 摩擦轮ID1(左)
#define RIGHT_FRICTION_WHEEL_CAN_ID 0x208 // 摩擦轮ID2(右)
#define FRICTION_LOSS_TIME	0.0075f //s
#define FRICTION_WHEEL_CAN &hcan2

//CAN1
// Yaw轴电机
#define YAW_MOTOR_SLAVE_CAN_ID 0x0A    // yaw轴电机
#define YAW_MOTOR_MASTER_CAN_ID 0x0B    // yaw轴电机
#define YAW_LOSS_TIME	0.0025f //s
#define YAW_MOTOR_CAN &hcan1

// 拨弹电机
#define TOGGLE_MOTOR_CAN_ID 0x206 // 拨弹电机
#define TOGGLE_LOSS_TIME	0.0035f //s
#define TOGGLE_MOTOR_CAN &hcan1

//丝杆电机
#define LIFT_MOTOR_CAN_ID 0x205
#define LIFT_LOSS_TIME	0.0035f //s
#define LIFT_MOTOR_CAN &hcan1

// 底盘通信
#define SEND_TO_CHASSIS_CAN_ID_1 0x150
#define SEND_TO_CHASSIS_CAN_ID_2 0x151
#define GET_FROM_CHASSIS_CAN_ID_1 0x160
#define GET_FROM_CHASSIS_CAN_ID_2 0x161
#define CHASSIS_LOSS_TIME	0.0055f //s
#define CHASSIS_CAN_COMM_CANx &hcan1

// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 0x000
#define CAN1_FIFO0_ID1 LIFT_MOTOR_CAN_ID
#define CAN1_FIFO0_ID2 TOGGLE_MOTOR_CAN_ID
#define CAN1_FIFO0_ID3 YAW_MOTOR_MASTER_CAN_ID

// FIFO 1 接收ID
#define CAN1_FIFO1_ID0 GET_FROM_CHASSIS_CAN_ID_2
#define CAN1_FIFO1_ID1 GET_FROM_CHASSIS_CAN_ID_1
#define CAN1_FIFO1_ID2 0x002
#define CAN1_FIFO1_ID3 0x003

// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 0x001
#define CAN2_FIFO0_ID1 PITCH_MOTOR_MASTER_CAN_ID
#define CAN2_FIFO0_ID2 0x004
#define CAN2_FIFO0_ID3 0x000

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 0x003
#define CAN2_FIFO1_ID1 LEFT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID2 RIGHT_FRICTION_WHEEL_CAN_ID
#define CAN2_FIFO1_ID3 0x000


#endif // !_MOTOR_CONFIG_H
