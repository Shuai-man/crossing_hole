#ifndef __DT7_CONTROLLER_H__
#define __DT7_CONTROLLER_H__


#include "main.h"
#include "bsp_DT7.h"
#include "KeyMouse.h"
#include "remote_control.h"
#include <stdlib.h>


#define MAX_SW_YAW_SPEED 180  // 云台yaw轴灵敏度(拨杆) 度/s
#define MAX_SW_PITCH_SPEED 80 // 云台yaw轴灵敏度(拨杆) 度/s

#define SELF_ROTATE_MODE 0// 小陀螺模式
#define _FLY_MODE 1// 飞坡模式
#define SHOOT_MODE 2// 射击模式
#define TEST_MODE_SELECT1 SELF_ROTATE_MODE//遥控器左中右上测试模式选择
#define TEST_MODE_SELECT2 SHOOT_MODE//遥控器左上右上测试模式选择

void DT7_Update(float delta_t);

#endif /* __DT7_CONTROLLER_H__ */

