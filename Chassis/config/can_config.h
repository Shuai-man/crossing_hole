#ifndef _CAN_CONFIG_H
#define _CAN_CONFIG_H

#include "stm32f4xx_hal_conf.h"
#include <string.h>
#include <stdint.h>

#include "can.h"
#include "robot_config.h"

/*-----------轮电机-------------*/
#define DJI_3508_MOTORS_1 0x201
#define DJI_3508_MOTORS_2 0x202
#define DJI_3508_MOTORS_3 0x203
#define DJI_3508_MOTORS_4 0x204
#define DJI_WHEELS_LOSS_TIME 0.0045f // 3508电机丢帧时间阈值
#define DJI_WHEELS_CAN &hcan1

/*-----------关节电机-------------*/
#define DJI_6020_MOTORS_1 0x205
#define DJI_6020_MOTORS_2 0x206
#define DJI_6020_MOTORS_3 0x207
#define DJI_6020_MOTORS_4 0x208
#define DJI_STEERS_LOSS_TIME 0.0045f // 6020电机丢帧时间阈值
#define DJI_STEERS_CAN &hcan1
/*-----------云台通信-------------*/
//接收
#define GIMBAL_COMM_CAN_ID_1 0x150 // 和云台通信CAN ID1
#define GIMBAL_COMM_CAN_ID_2 0x151 // 和云台通信CAN ID2
//发送
#define SEND_TO_GIMBAL_CAN_ID_1 0x160
#define SEND_TO_GIMBAL_CAN_ID_2 0x161
#define GIMBAL_LOSS_TIME 0.0055f // 云台丢帧时间阈值
#define GIMBAL_CAN_COMM_CANx &hcan2

/*-----------超级电容-------------*/
#define SUPER_POWER_CAN_ID 0x051
#define SEND_TO_SUPER_POWER_CAN_ID 0x050
#define SUPER_POWER_LOSS_TIME 0.0055f // 超级电容丢帧时间阈值
#define SUPER_POWER_CAN &hcan1

/*-----------无线充电-------------*/
#define WIRELESS_CAN_ID 0x061
#define SEND_TO_WIRELESS_CAN_ID 0x060
#define WIRELESS_LOSS_TIME 0.0055f // 无线充电丢帧时间阈值
#define WIRELESS_CAN &hcan1

// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 DJI_3508_MOTORS_1
#define CAN1_FIFO0_ID1 DJI_3508_MOTORS_2
#define CAN1_FIFO0_ID2 DJI_3508_MOTORS_3
#define CAN1_FIFO0_ID3 DJI_3508_MOTORS_4

// FIFO 1 接收ID
#define CAN1_FIFO1_ID0 SUPER_POWER_CAN_ID
#define CAN1_FIFO1_ID1 WIRELESS_CAN_ID
#define CAN1_FIFO1_ID2 0x001
#define CAN1_FIFO1_ID3 0x002

// FIFO 0 接收ID
#define CAN2_FIFO0_ID0 GIMBAL_COMM_CAN_ID_1
#define CAN2_FIFO0_ID1 GIMBAL_COMM_CAN_ID_2
#define CAN2_FIFO0_ID2 0x002
#define CAN2_FIFO0_ID3 0x001

// FIFO 1 接收ID
#define CAN2_FIFO1_ID0 0x002
#define CAN2_FIFO1_ID1 0x003
#define CAN2_FIFO1_ID2 0x000
#define CAN2_FIFO1_ID3 0x001



#endif // !_CAN_CONFIG_H
