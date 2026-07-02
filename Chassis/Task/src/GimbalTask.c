#include "GimbalTask.h"

uint8_t send_to_gimbal_data_1[8];
uint8_t send_to_gimbal_data_2[8];

void SendToGimbalPack()
{
    GimbalSendPack();
    memcpy(send_to_gimbal_data_1, &gimbal_pack_send_1, 8);
    memcpy(send_to_gimbal_data_2, &gimbal_pack_send_2, 8);
}

/**
 * @brief 云台控制任务
 * @param[in] void
 */
void GimbalTask(void const *argument)
{
    portTickType xLastWakeTime;

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        HeatUpdate();

        SendToGimbalPack();
        CanSend(GIMBAL_CAN_COMM_CANx, send_to_gimbal_data_1, SEND_TO_GIMBAL_CAN_ID_1, 8);
        CanSend(GIMBAL_CAN_COMM_CANx, send_to_gimbal_data_2, SEND_TO_GIMBAL_CAN_ID_2, 8);

        Gimbal_msgs_Update1();
        Gimbal_msgs_Update2();

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}
