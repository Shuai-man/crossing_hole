#ifndef __VTM_CONTROLLER_H__
#define __VTM_CONTROLLER_H__

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
  //功能标志位
  /*升降*/
	bool lift_flag;
	bool last_right_up;

}VTM_Controller_t;

#define MAX_SW_YAW_SPEED 180  // 云台yaw轴灵敏度(拨杆) 度/s
#define MAX_SW_PITCH_SPEED 80 // 云台pitch轴灵敏度(拨杆) 度/s

void VTM_Update(float delta_t);
void VTM_Init(void);
void VTM_State_Clear(void);
void VTM_Fire(void);

#endif

