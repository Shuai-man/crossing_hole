#ifndef _REFEREE_H
#define _REFEREE_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Referee.h"
#include "robot_config.h"
#include "usart.h"


#define REFEREE_RECVBUF_SIZE 4096
#define REFEREE_SENDBUF_SIZE 128

//#endif

/*  数据定义  */
extern uint8_t Refereebuffer[REFEREE_RECVBUF_SIZE];
extern uint8_t SendToReferee_Buff[REFEREE_SENDBUF_SIZE];


void REFEREE_SendBytes(uint8_t *data, uint8_t len);

#endif // !_REFEREE_H
