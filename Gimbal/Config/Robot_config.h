#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

#define OLD 0 // 老云台：TOF作为升降位置传感器，并启用TOF联合保护
#define NEW 1 // 新云台：外置绝对编码器作为升降位置传感器

#ifndef ROBOT_SELECT
#define ROBOT_SELECT NEW
#endif


#endif
