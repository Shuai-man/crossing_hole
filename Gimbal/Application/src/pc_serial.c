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
    if (PCbuffer[0] == '!' && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
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
    volatile unsigned char aim_press_r = remote_controller.dji_remote.mouse.press_r;
    pc_send_data.aim_request = remote_controller.auto_arm;
    pc_send_data.start_flag = '!';
    pc_send_data.pitch_now = gimbal_controller.gyro_pitch_angle;
    pc_send_data.yaw_now = gimbal_controller.gyro_yaw_angle;
    pc_send_data.roll_now = 0.0f;
    pc_send_data.actual_bullet_speed = chassis_pack_get_1.shoot_speed;
    pc_send_data.shoot_avaiable = chassis_pack_get_1.shoot_avaiable;
    pc_send_data.number_want = 0;
    pc_send_data.enemy_color = !chassis_pack_get_1.robot_color;
    Append_CRC16_Check_Sum((uint8_t *)(&pc_send_data), PC_SENDBUF_SIZE);

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
