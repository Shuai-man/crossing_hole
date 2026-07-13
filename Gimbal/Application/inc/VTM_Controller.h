#ifndef __VTM_CONTROLLER_H__
#define __VTM_CONTROLLER_H__

#include "main.h"
#include "bsp_VTM.h"
#include "Offline_Task.h"
#include "remote_control.h"
#include "KeyMouse.h"

typedef struct
{
  //功能标志位
  /*升降*/
	bool lift_flag;
	bool last_right_up;

}VTM_Controller_t;

void VTM_Update(float delta_t);
void VTM_Init(void);
void VTM_State_Clear(void);
void VTM_Fire(void);

#endif

