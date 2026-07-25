#ifndef __REFEREETASK_H
#define __REFEREETASK_H

//INCLUDE部分
#include "fifo.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "protocol.h"

#define Max(a,b) ((a) > (b) ? (a) : (b))
#define Robot_ID_Current Robot_Status.robot_id

void Ref_Init(void);
void Refereetask(void const *argument);

void Sightglass_static_show(void);
void Sightglass1_flash_show(void);
void Sightglass2_flash_show(void);

//EXTERN部分
extern int Rest_UI_Flag;
extern bool ref_ready_flag;

extern TaskHandle_t RefereeTask_Handle;
extern unpack_data_t Referee_Unpack_OBJ;

#endif
