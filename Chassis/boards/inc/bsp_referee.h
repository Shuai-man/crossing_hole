#ifndef _REFEREE_H
#define _REFEREE_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "usart.h"
#include "fifo.h"
#include "protocol.h"

#define REFEREE_FIFO_BUF_LENGTH     1024
#define REFEREE_USART_RX_BUF_LENGHT 512

/*  数据定义  */
extern fifo_s_t Referee_FIFO;

extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];
void Referee_Receive(void);

#endif // !_REFEREE_H
