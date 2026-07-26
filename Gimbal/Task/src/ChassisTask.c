#include "ChassisTask.h"

#include "bsp_can.h"
#include "can_config.h"
#include "ChassisSend.h"

int8_t send_to_chassis_data_1[8]; // 模式
int8_t send_to_chassis_data_2[8]; // pitch,yaw

/**
 * @brief 处理速度数据，将底盘期望速度发送给底盘stm32
 * @param[in] void
 */
void Chassis_Task(void const *argument)
{
    portTickType xLastWakeTime;
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待can外设初始化
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        Pack_InfantryMode();
        memcpy(send_to_chassis_data_1, &chassis_send_pack1, 8);
        CanSend(CHASSIS_CAN_COMM_CANx, send_to_chassis_data_1, SEND_TO_CHASSIS_CAN_ID_1);

        Pack_Chassis2();
        memcpy(send_to_chassis_data_2, &chassis_send_pack2, 8);
        CanSend(CHASSIS_CAN_COMM_CANx, send_to_chassis_data_2, SEND_TO_CHASSIS_CAN_ID_2);

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4)); // 4ms发一次，既250hz
    }
}
