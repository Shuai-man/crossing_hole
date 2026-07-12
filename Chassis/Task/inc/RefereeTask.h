#ifndef __REFEREETASK_H
#define __REFEREETASK_H

//INCLUDE部分
#include "fifo.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include "Referee.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "bsp_referee.h"


#define Max(a,b) ((a) > (b) ? (a) : (b))
#define Robot_ID_Current Robot_Status.robot_id

void Ref_Init(void);
void Refereetask(void const *argument);

void Sightglass_static_show(void);
void Sightglass1_static_show(void);
void Sightglass2_static_show(void);

void Show_ZERO_static(void);
void Show_FIRE_static(void);
void Show_FALL_static(void);
void Show_BUMP_static(void);
void Show_SPIN_static(void);
void Show_CHANGE_static(void);

void Sightglass_flash_show(void);
void Sightglass1_flash_show(void);

//EXTERN部分
extern int Rest_UI_Flag;
extern bool ref_ready_flag;

extern TaskHandle_t RefereeTask_Handle;
extern unpack_data_t Referee_Unpack_OBJ;

#endif
