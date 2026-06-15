#include "ChassisController.h"

void setRobotType()
{
#if ROBOT == CHEN_JING_YUAN
    infantry.chassis_type = MECANUM_WHEEL;
    infantry.yaw_motor_type = YAW_DM_MOTOR;
    infantry.chassis_follow_type = TWO_SIDES_FOLLOW;
    infantry.power_limit_method = SPEED_ERROR_METHOD;
#endif

    if (infantry.chassis_type == MECANUM_WHEEL)
    {
        mecanum_pid_init();
    }

}
