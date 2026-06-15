#include "VOFATask.h"

/**
 * @brief VOFA debug
 * @param[in] void
 */
void VOFATask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 5; // 200HZ

    vTaskDelay(2000);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();
        VOFA_Send();

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
