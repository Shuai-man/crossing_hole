/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发	和算力板通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "Gimbal.h"
#include "arm_atan2_f32.h"
#include "debug.h"
#include "usbd_cdc_if.h"
#include "SignalGenerator.h"
#include "remote_control.h"

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
    global_debugger.pc_receive_debugger.loss_time = 0;
}

void PCReceive(unsigned char *PCbuffer)
{
    if (PCbuffer[0] == 0xAA && PCbuffer[1] == 0x55 &&
        Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE - 1) &&
        PCbuffer[PC_RECVBUF_SIZE - 1] == 0x0D)
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.frame_header1 = 0x55;
    pc_send_data.frame_header2 = 0xAA;
    pc_send_data.pitch_now = gimbal_controller.gyro_pitch_angle;
    pc_send_data.yaw_now = gimbal_controller.gyro_yaw_angle;
    pc_send_data.pitch_omega = gimbal_controller.gyro_pitch_speed;
    pc_send_data.yaw_omega = gimbal_controller.gyro_yaw_speed;
    pc_send_data.pitch_tff = gimbal_controller.DM_Pitch_Motor.t_ff_Receive;
    pc_send_data.yaw_tff = gimbal_controller.DM_Yaw_Motor.t_ff_Receive;
    pc_send_data.actual_bullet_speed = chassis_pack_get_1.shoot_speed;
    pc_send_data.shoot_avaiable = chassis_pack_get_1.shoot_avaiable;
    pc_send_data.aim_request = remote_controller.auto_arm;
    // modewant在其他地方处理
    pc_send_data.number_want = 0;
    pc_send_data.enemy_color = !chassis_pack_get_1.robot_color;
    pc_send_data.frame_tail = 0x0D;
    Append_CRC16_Check_Sum((uint8_t *)(&pc_send_data), PC_SENDBUF_SIZE - 1); // 不包含frame_tail

    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE); // 通过USB_CDC发送
}
