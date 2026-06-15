#ifndef _WIRELESS_H_
#define _WIRELESS_H_

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"

typedef enum WIRELESS_STATE{
	WIRELESS_UP,
	WIRELESS_DOWN,
}WIRELESS_STATE;

typedef struct WirelessSendData
{
    uint16_t startflag; 
    uint16_t _;         
    uint16_t __;
    uint16_t ___;
} WirelessSendData;

typedef struct WirelessRecvData
{
    uint16_t power_buckin; // ³äµç¹¦ÂÊ
    uint16_t _;
    uint16_t __;
    uint16_t ___;
} WirelessRecvData;

typedef struct{
	WIRELESS_STATE state;
	uint8_t start_flag;
}Wireless_Charge;

extern Wireless_Charge wireless_charge;
extern WirelessSendData wireless_send_data;
extern WirelessRecvData wireless_recv_data;

void wireless_init(void);
void SendWirelessPack(void);


#endif
