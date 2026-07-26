#ifndef __DT7_CONTROLLER_H__
#define __DT7_CONTROLLER_H__

#include <stdlib.h>

#define MAX_SW_YAW_SPEED 180  // 云台yaw轴灵敏度(拨杆) 度/s
#define MAX_SW_PITCH_SPEED 80 // 云台pitch轴灵敏度(拨杆) 度/s

void DT7_Update(float delta_t);

#endif /* __DT7_CONTROLLER_H__ */

