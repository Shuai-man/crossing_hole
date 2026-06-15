#ifndef _BLUE_TOOTH_H
#define _BLUE_TOOTH_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "usart.h"
#include "dma.h"
#include "remote_control.h"


#pragma pack(push, 1)
#define CH_COUNT_B 6
typedef struct BlueToothSendData
{
    float fdata[CH_COUNT_B];
    unsigned char tail[4];
} BlueToothSendData;
#pragma pack(pop)

#define BlueTooth_SENDBUF_SIZE sizeof(BlueToothSendData)

void BLUE_TOOTHSendData(BlueToothSendData *data);

#endif // !_BLUE_TOOTH_H
