#include "wireless.h"


Wireless_Charge wireless_charge;
WirelessSendData wireless_send_data;
WirelessRecvData wireless_recv_data;

void wireless_init(void)
{
	wireless_charge.state = WIRELESS_DOWN;
	wireless_send_data.startflag = 0;
}

void SendWirelessPack(void)
{
	if(wireless_charge.state == WIRELESS_DOWN)//об╫╣
	{
		wireless_send_data.startflag = 0;
	}else{ //иоиЩ
		wireless_send_data.startflag = 1;
	}
}

