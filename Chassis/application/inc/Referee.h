/**
 ******************************************************************************
 * @file    referee.h
 * @author  Deepseek
 * @version V2.0.0
 * @date    2026/06/26
 * @brief   Header file of referee.c
 ******************************************************************************
 * @attention
 *
 *   依据裁判系统 串口协议附录 V2.0.0 (2026)
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREE_H__
#define __REFEREE_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "protocol.h"
#include "algorithmOfCRC.h"

#include "usart.h"

#include "debug.h"
#include "bsp_dwt.h"
#include "bsp_referee.h"

#include "HeatControl.h"

/* Referee Defines -----------------------------------------------------------*/
/* 比赛类型 */
#define Game_Type_RMUC 1     // 超级对抗赛
#define Game_Type_RMUT 2     // 单项赛
#define Game_Type_RMUA 3     // 人工智能挑战赛
#define Game_Type_RMUL_3V3 4 // 高校联盟赛3V3
#define Game_Type_RMUL_1V1 5 // 高校联盟赛1V1

/* 比赛阶段 */
#define Game_Progress_Unstart 0   // 未开始比赛
#define Game_Progress_Prepare 1   // 准备阶段
#define Game_Progress_SelfCheck 2 // 十五秒裁判系统自检阶段
#define Game_Progress_5sCount 3   // 五秒倒计时
#define Game_Progress_Battle 4    // 比赛中
#define Game_Progress_Calculate 5 // 比赛结算中

/* 比赛结果 */
#define Game_Result_Draw 0    // 平局
#define Game_Result_RedWin 1  // 红方胜利
#define Game_Result_BlueWin 2 // 蓝方胜利

/* 警告信息 (V2.0.0: Warning_Yellow=2, Warning_Failure=4) */
#define Warning_Both_Yellow 1 // 双方黄牌
#define Warning_Yellow 2      // 黄牌
#define Warning_Red 3         // 红牌
#define Warning_Failure 4     // 判负

/* 机器人ID */
#define Robot_ID_Red_Hero 1         // 红方英雄
#define Robot_ID_Red_Engineer 2     // 红方工程
#define Robot_ID_Red_Infantry3 3    // 红方步兵3
#define Robot_ID_Red_Infantry4 4    // 红方步兵4
#define Robot_ID_Red_Infantry5 5    // 红方步兵5
#define Robot_ID_Red_Aerial 6       // 红方空中机器人
#define Robot_ID_Red_Sentry 7       // 红方哨兵
#define Robot_ID_Red_Darts 8        // 红方飞镖
#define Robot_ID_Red_Radar 9        // 红方雷达
#define Robot_ID_Blue_Hero 101      // 蓝方英雄
#define Robot_ID_Blue_Engineer 102  // 蓝方工程
#define Robot_ID_Blue_Infantry3 103 // 蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 // 蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 // 蓝方步兵5
#define Robot_ID_Blue_Aerial 106    // 蓝方空中机器人
#define Robot_ID_Blue_Sentry 107    // 蓝方哨兵
#define Robot_ID_Blue_Darts 108     // 蓝方飞镖
#define Robot_ID_Blue_Radar 109     // 蓝方雷达

/* 机器人等级 */
#define Robot_Level_1 1 // 1级
#define Robot_Level_2 2 // 2级
#define Robot_Level_3 3 // 3级

/* 扣血类型 (V2.0.0: 超射速/超热量/超功率已移除) */
#define Hurt_Type_ArmoredPlate 0  // 装甲模块被弹丸攻击导致扣血
#define Hurt_Type_ModuleOffline 1 // 模块离线导致扣血
#define Hurt_Type_Collision 5     // 装甲模块受到撞击导致扣血

/* 发射机构编号 (V2.0.0: 仅保留17mm和42mm各一个) */
#define Shooter_ID1_17mm 1 // 1号17mm发射机构
#define Shooter_ID1_42mm 3 // 1号42mm发射机构

/* 飞镖信息 */
#define Dart_State_Open 0     // 飞镖闸门已经开启
#define Dart_State_Close 1    // 飞镖闸门关闭
#define Dart_State_Changing 2 // 正在开启或者关闭中
#define Dart_Target_None 0         // 未选定
#define Dart_Target_Outpost 1      // 飞镖目标为前哨站
#define Dart_Target_BaseFixed 2    // 基地固定目标
#define Dart_Target_BaseRandom 3   // 基地随机固定目标
#define Dart_Target_BaseMoveRan 4  // 基地随机移动目标
#define Dart_Target_BaseMoveEnd 5  // 基地末端移动目标

/* 操作手ID */
#define Cilent_ID_Red_Hero 0x0101      // 红方英雄操作手
#define Cilent_ID_Red_Engineer 0x0102  // 红方工程操作手
#define Cilent_ID_Red_Infantry3 0x0103 // 红方步兵3操作手
#define Cilent_ID_Red_Infantry4 0x0104 // 红方步兵4操作手
#define Cilent_ID_Red_Infantry5 0x0105 // 红方步兵5操作手
#define Cilent_ID_Red_Aerial 0x0106    // 红方飞手
#define Cilent_ID_Blue_Hero 0x0165     // 蓝方英雄操作手
#define Cilent_ID_Blue_Engineer 0x0166 // 蓝方工程操作手
#define Cilent_ID_Blue_Infantry3 0x0167 // 蓝方步兵3操作手
#define Cilent_ID_Blue_Infantry4 0x0168 // 蓝方步兵4操作手
#define Cilent_ID_Blue_Infantry5 0x0169 // 蓝方步兵5操作手
#define Cilent_ID_Blue_Aerial 0x016A    // 蓝方飞手

/* UI绘制内容cmdID (V2.0.0: 作为0x0301子内容ID) */
#define UI_DataID_Delete 0x100  // 客户端删除图层
#define UI_DataID_Draw1 0x101   // 客户端绘制1个图形
#define UI_DataID_Draw2 0x102   // 客户端绘制2个图形
#define UI_DataID_Draw5 0x103   // 客户端绘制5个图形
#define UI_DataID_Draw7 0x104   // 客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 // 客户端绘制字符图形

/* 0x0301 子内容ID — 哨兵/雷达自主决策 */
#define SubDataID_SentryCmd 0x0120 // 哨兵自主决策指令
#define SubDataID_RadarCmd 0x0121  // 雷达自主决策指令

/* UI删除操作 */
#define UI_Delete_Invalid 0 // 空操作
#define UI_Delete_Layer 1   // 删除图层
#define UI_Delete_All 2     // 删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 // 空操作
#define UI_Graph_Add 1     // 增加图形
#define UI_Graph_Change 2  // 修改图形
#define UI_Graph_Delete 3  // 删除图形

/* UI图形类型 */
#define UI_Graph_Line 0      // 直线
#define UI_Graph_Rectangle 1 // 矩形
#define UI_Graph_Circle 2    // 正圆
#define UI_Graph_Ellipse 3   // 椭圆
#define UI_Graph_Arc 4       // 圆弧
#define UI_Graph_Float 5     // 浮点型
#define UI_Graph_Int 6       // 整型
#define UI_Graph_String 7    // 字符型

/* UI图形颜色 */
#define UI_Color_Main 0    // 红蓝主色，己方颜色
#define UI_Color_Yellow 1  // 黄色
#define UI_Color_Green 2   // 绿色
#define UI_Color_Orange 3  // 橙色
#define UI_Color_Purple 4  // 紫红色
#define UI_Color_Pink 5    // 粉色
#define UI_Color_Cyan 6    // 青色
#define UI_Color_Black 7   // 黑色
#define UI_Color_White 8   // 白色

#pragma pack(push, 1)

/* 0x000X — 比赛全局数据 ----------------------------------------------------*/
typedef struct // 0x0001 比赛状态数据 (11 bytes)
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef struct // 0x0002 比赛结果数据 (1 byte)
{
	uint8_t winner;
} ext_game_result_t;

typedef struct // 0x0003 机器人血量数据 (20 bytes, V2.0.0)
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	int16_t damage_difference;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
	uint16_t enemy_outpost_HP;
	uint16_t enemy_base_HP;
} ext_game_robot_HP_t;

/* 0x010X — 场地/事件数据 ------------------------------------------------*/
typedef struct // 0x0101 场地事件数据 (4 bytes, V2.0.0: 统一为uint32_t)
{
	uint32_t event_data;
} ext_event_data_t;

typedef struct // 0x0102 补给站动作标识
{
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct // 0x0104 裁判警告信息 (3 bytes, V2.0.0)
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} ext_referee_warning_t;

typedef struct // 0x0105 飞镖倒计时与击中信息 (3 bytes)
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 0x020X — 机器人状态数据 ------------------------------------------------*/
typedef struct // 0x0201 比赛机器人状态 (17 bytes, V2.0.0: 新增bullet_speed_limit)
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	float bullet_speed_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

typedef struct // 0x0202 实时功率热量数据 (14 bytes, V2.0.0)
{
	uint16_t reserved_1;
	uint16_t reserved_2;
	float reserved_3;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef struct // 0x0203 机器人位置 (12 bytes, V2.0.0: z/yaw → angle)
{
	float x;
	float y;
	float angle;
} ext_game_robot_pos_t;

typedef struct // 0x0204 机器人增益 (8 bytes, V2.0.0: cooling_buff→uint16_t)
{
	uint8_t recovery_buff;
	uint16_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} ext_buff_musk_t;

typedef struct // 0x0206 伤害状态 (1 byte, V2.0.0)
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;

typedef struct // 0x0207 实时射击信息 (7 bytes, V2.0.0)
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} ext_shoot_data_t;

typedef struct // 0x0208 允许发弹量与金币 (8 bytes, V2.0.0: 新增堡垒储备发弹量)
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;
} ext_bullet_remaining_t;

typedef struct // 0x0209 RFID模块状态 (5 bytes, V2.0.0 新增)
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

typedef struct // 0x020A 飞镖选手端指令 (6 bytes, V2.0.0 新增)
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef struct // 0x020B 地面机器人位置 (40 bytes, V2.0.0: 移除standard_5)
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float reserved_1;
	float reserved_2;
} ground_robot_position_t;

typedef struct // 0x020C 雷达标记进度 (2 bytes, V2.0.0 新增)
{
	uint16_t mark_progress;
} radar_mark_data_t;

typedef struct // 0x020D 哨兵自主决策信息同步 (14 bytes, V2.0.0: 完全重写)
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
	uint64_t sentry_info_3;
} sentry_info_t;

typedef struct // 0x020E 雷达自主决策信息同步 (1 byte, V2.0.0)
{
	uint8_t radar_double_hurt_chance : 2;
	uint8_t radar_if_double_hurt : 1;
	uint8_t encryption_level : 2;
	uint8_t can_modify_key : 1;
	uint8_t reserved : 2;
} radar_info_t;

/* 0x030X — 机器人交互数据 ------------------------------------------------*/
typedef struct // 0x0301 机器人间通信 数据段头 (6 bytes)
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
} ext_student_interactive_header_data_t;

typedef struct // 0x0301 完整数据段 (6+x bytes, V2.0.0: user_data→柔性数组112)
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[112];
} robot_interactive_data_t;

typedef struct // 0x0120 哨兵自主决策指令 (4 bytes, 0x0301子内容)
{
	uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct // 0x0121 雷达自主决策指令 (8 bytes, 0x0301子内容)
{
	uint8_t radar_cmd;
	uint8_t password_cmd;
	uint8_t password_1;
	uint8_t password_2;
	uint8_t password_3;
	uint8_t password_4;
	uint8_t password_5;
	uint8_t password_6;
} radar_cmd_t;

typedef struct // 0x0303 选手端小地图下发 (12 bytes, V2.0.0: 移除z, 新增cmd_source)
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint16_t cmd_source;
} ext_robot_command_t;

typedef struct // 0x0305 双方机器人小地图坐标 (48 bytes, V2.0.0)
{
	uint16_t opponent_hero_position_x;
	uint16_t opponent_hero_position_y;
	uint16_t opponent_engineer_position_x;
	uint16_t opponent_engineer_position_y;
	uint16_t opponent_infantry_3_position_x;
	uint16_t opponent_infantry_3_position_y;
	uint16_t opponent_infantry_4_position_x;
	uint16_t opponent_infantry_4_position_y;
	uint16_t opponent_aerial_position_x;
	uint16_t opponent_aerial_position_y;
	uint16_t opponent_sentry_position_x;
	uint16_t opponent_sentry_position_y;
	uint16_t ally_hero_position_x;
	uint16_t ally_hero_position_y;
	uint16_t ally_engineer_position_x;
	uint16_t ally_engineer_position_y;
	uint16_t ally_infantry_3_position_x;
	uint16_t ally_infantry_3_position_y;
	uint16_t ally_infantry_4_position_x;
	uint16_t ally_infantry_4_position_y;
	uint16_t ally_aerial_position_x;
	uint16_t ally_aerial_position_y;
	uint16_t ally_sentry_position_x;
	uint16_t ally_sentry_position_y;
} map_robot_data_t;

typedef struct // 0x0A05 对方机器人增益效果 (41 bytes, V2.0.0)
{
	uint8_t hero_recovery_buff;
	uint16_t hero_cooling_buff;
	uint8_t hero_defence_buff;
	uint8_t hero_vulnerability_buff;
	uint16_t hero_attack_buff;
	uint8_t engineer_recovery_buff;
	uint16_t engineer_cooling_buff;
	uint8_t engineer_defence_buff;
	uint8_t engineer_vulnerability_buff;
	uint16_t engineer_attack_buff;
	uint8_t infantry_3_recovery_buff;
	uint16_t infantry_3_cooling_buff;
	uint8_t infantry_3_defence_buff;
	uint8_t infantry_3_vulnerability_buff;
	uint16_t infantry_3_attack_buff;
	uint8_t infantry_4_recovery_buff;
	uint16_t infantry_4_cooling_buff;
	uint8_t infantry_4_defence_buff;
	uint8_t infantry_4_vulnerability_buff;
	uint16_t infantry_4_attack_buff;
	uint8_t sentry_recovery_buff;
	uint16_t sentry_cooling_buff;
	uint8_t sentry_defence_buff;
	uint8_t sentry_vulnerability_buff;
	uint16_t sentry_attack_buff;
	uint8_t sentry_attitude;
	uint8_t hero_status;
	uint8_t engineer_status;
	uint8_t infantry_3_status;
	uint8_t infantry_4_status;
	uint8_t sentry_status;
} enemy_buff_data_t;


typedef struct // 0x0A01 对方机器人位置坐标 (24 bytes, V2.0.0 新增)
{
	uint16_t opponent_hero_position_x;
	uint16_t opponent_hero_position_y;
	uint16_t opponent_engineer_position_x;
	uint16_t opponent_engineer_position_y;
	uint16_t opponent_infantry_3_position_x;
	uint16_t opponent_infantry_3_position_y;
	uint16_t opponent_infantry_4_position_x;
	uint16_t opponent_infantry_4_position_y;
	uint16_t opponent_aerial_position_x;
	uint16_t opponent_aerial_position_y;
	uint16_t opponent_sentry_position_x;
	uint16_t opponent_sentry_position_y;
} enemy_position_data_t;

typedef struct // 0x0A02 对方机器人血量 (12 bytes, V2.0.0 新增)
{
	uint16_t opponent_1_hero_HP;
	uint16_t opponent_2_engineer_HP;
	uint16_t opponent_3_infantry_HP;
	uint16_t opponent_4_infantry_HP;
	uint16_t reserved;
	uint16_t opponent_7_sentry_HP;
} enemy_HP_data_t;

typedef struct // 0x0A03 对方机器人允许发弹量 (10 bytes, V2.0.0 新增)
{
	uint16_t opponent_1_hero_projectile_allowance;
	uint16_t opponent_3_infantry_projectile_allowance;
	uint16_t opponent_4_infantry_projectile_allowance;
	uint16_t opponent_6_aerial_projectile_allowance;
	uint16_t opponent_7_sentry_projectile_allowance;
} enemy_projectile_data_t;

typedef struct // 0x0A04 对方队伍宏观状态 (8 bytes, V2.0.0 新增)
{
	uint16_t opponent_remaining_gold_coin;
	uint16_t opponent_total_gold_coin;
	uint32_t opponent_macro_status;
} enemy_macro_state_data_t;

typedef struct // 0x0A06 对方干扰波密钥 (6 bytes, V2.0.0 新增)
{
	uint8_t key[6];
} enemy_key_data_t;

typedef struct // 0x0306 自定义控制器与选手端交互 (8 bytes, V2.0.0 新增)
{
	uint16_t key_value;
	uint16_t x_position : 12;
	uint16_t mouse_left : 4;
	uint16_t y_position : 12;
	uint16_t mouse_right : 4;
	uint16_t reserved_custom;
} custom_client_data_t;

typedef struct // 0x0307 选手端小地图接收路径数据 (103 bytes, V2.0.0 新增)
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} map_data_t;

typedef struct // 0x0308 选手端小地图接收消息 (34 bytes, V2.0.0 新增)
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;


/* 自定义内部通信结构体 ------------------------------------------------------*/
typedef struct
{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1;         // 1 -> red ; 0 -> blue
	uint8_t Enemy_Sentry_shootable : 1; // 对方哨兵是否无敌
	uint16_t self_outpost : 11;
	uint8_t Sentry_HomeReturned_flag : 1;
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; // 0x0208
	uint16_t stage_remain_time;         // 0x0001
} JudgeData_ForSend1_t;

typedef struct
{
	uint16_t x; // 坐标系 x (float*100 → uint16_t)
	uint16_t y;
	uint8_t commd_keyboard; // 云台手指令
	uint8_t Base_Shield;
	uint8_t KeyBoard_Update : 1;
	uint8_t _ : 7;
	uint8_t __;
} JudgeData_ForSend2_t;

/* 自定义绘制UI结构体 (V2.0.0: interaction_figure_t 对齐) -------------------*/
typedef struct // 0x0101 图形数据 interaction_figure_t (15 bytes, V2.0.0 spec uses details_a~e)
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
} graphic_data_struct_t;


typedef struct // 0x0110 字符图形数据 (45 bytes)
{
	uint8_t string_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data_struct_t;

typedef struct // 0x0100 删除图层数据 (2 bytes)
{
	uint8_t delete_type;
	uint8_t layer;
} delete_data_struct_t;

/* UI完整数据包结构体 --------------------------------------------------------*/
typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef struct
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;

#pragma pack(pop)

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);

void Referee_UnpackFifoData(void);
void Referee_SolveFifoData(uint8_t *frame);

void UI_Draw_Line(graphic_data_struct_t *Graph,
				  char GraphName[3],
				  uint8_t GraphOperate,
				  uint8_t Layer,
				  uint8_t Color,
				  uint16_t Width,
				  uint16_t StartX,
				  uint16_t StartY,
				  uint16_t EndX,
				  uint16_t EndY);
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,
					   char GraphName[3],
					   uint8_t GraphOperate,
					   uint8_t Layer,
					   uint8_t Color,
					   uint16_t Width,
					   uint16_t StartX,
					   uint16_t StartY,
					   uint16_t EndX,
					   uint16_t EndY);
void UI_Draw_Circle(graphic_data_struct_t *Graph,
					char GraphName[3],
					uint8_t GraphOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t Width,
					uint16_t CenterX,
					uint16_t CenterY,
					uint16_t Radius);
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,
					 char GraphName[3],
					 uint8_t GraphOperate,
					 uint8_t Layer,
					 uint8_t Color,
					 uint16_t Width,
					 uint16_t CenterX,
					 uint16_t CenterY,
					 uint16_t XHalfAxis,
					 uint16_t YHalfAxis);
void UI_Draw_Arc(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t StartAngle,
				 uint16_t EndAngle,
				 uint16_t Width,
				 uint16_t CenterX,
				 uint16_t CenterY,
				 uint16_t XHalfAxis,
				 uint16_t YHalfAxis);
void UI_Draw_Float(graphic_data_struct_t *Graph,
				   char GraphName[3],
				   uint8_t GraphOperate,
				   uint8_t Layer,
				   uint8_t Color,
				   uint16_t NumberSize,
				   uint16_t Significant,
				   uint16_t Width,
				   uint16_t StartX,
				   uint16_t StartY,
				   float FloatData);
void UI_Draw_Int(graphic_data_struct_t *Graph,
				 char GraphName[3],
				 uint8_t GraphOperate,
				 uint8_t Layer,
				 uint8_t Color,
				 uint16_t NumberSize,
				 uint16_t Width,
				 uint16_t StartX,
				 uint16_t StartY,
				 int32_t IntData);
void UI_Draw_String(string_data_struct_t *String,
					char StringName[3],
					uint8_t StringOperate,
					uint8_t Layer,
					uint8_t Color,
					uint16_t CharSize,
					uint16_t StringLength,
					uint16_t Width,
					uint16_t StartX,
					uint16_t StartY,
					char *StringData);

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);

/* 裁判系统数据解码器 */
typedef struct Referee_Decoder
{
	uint16_t judgementFullCount;
	uint64_t receive_data_len;
	uint64_t decode_data_len;

	uint8_t judgementStep;
	uint16_t index;
	uint16_t data_len;

} Referee_Decoder;

typedef struct Referee_t
{
	/* protocol包头结构体 */
	frame_header_struct_t Referee_Receive_Header;

	/* 0x000X */
	ext_game_status_t Game_Status;
	ext_game_result_t Game_Result;
	ext_game_robot_HP_t Game_Robot_HP;

	/* 0x010X */
	ext_event_data_t Event_Data;
	ext_supply_projectile_action_t Supply_Projectile_Action;
	ext_referee_warning_t Referee_Warning;
	ext_dart_remaining_time_t Dart_Remaining_Time;

	/* 0x020X */
	ext_game_robot_state_t Game_Robot_State;
	ext_power_heat_data_t Power_Heat_Data;
	ext_game_robot_pos_t Game_Robot_Pos;
	ext_buff_musk_t Buff_Musk;
	ext_robot_hurt_t Robot_Hurt;
	ext_shoot_data_t Shoot_Data;
	ext_bullet_remaining_t Bullet_Remaining;
	ext_rfid_status_t RFID_Status;
	ext_dart_client_cmd_t Dart_Client_Cmd;
	ground_robot_position_t Ground_Robot_Position;
	radar_mark_data_t Radar_Mark_Data;
	sentry_info_t Sentry_Info;
	radar_info_t Radar_Info;

	/* 0x030X — 常规链路交互 */
	ext_student_interactive_header_data_t Student_Interactive_Header_Data;
	robot_interactive_data_t Robot_Interactive_Data;
	ext_robot_command_t Robot_Command;
	map_robot_data_t Map_Robot_Data;

	/* 哨兵/雷达自主决策 */
	sentry_cmd_t Sentry_Cmd;
	radar_cmd_t Radar_Cmd;

	/* 0x030X — 图传链路/选手端 */
	custom_client_data_t Custom_Client_Data;
	map_data_t Map_Data;
	custom_info_t Custom_Info;

	/* 0x0A0X — 雷达无线链路 */
	enemy_position_data_t Enemy_Position_Data;
	enemy_HP_data_t Enemy_HP_Data;
	enemy_projectile_data_t Enemy_Projectile_Data;
	enemy_macro_state_data_t Enemy_Macro_State_Data;
	enemy_buff_data_t Enemy_Buff_Data;
	enemy_key_data_t Enemy_Key_Data;

	/* 绘制UI专用结构体 */
	UI_Graph1_t UI_Graph1;
	UI_Graph2_t UI_Graph2;
	UI_Graph5_t UI_Graph5;
	UI_Graph7_t UI_Graph7;
	UI_String_t UI_String;
	UI_Delete_t UI_Delete;

	Referee_Decoder decoder;

} Referee_t;

typedef struct RefereeDataUpdate
{
	int8_t is_max_power_data_update;
	int8_t is_power_data_update;
} RefereeDataUpdate;

extern RefereeDataUpdate referee_data_updater;

extern Referee_t referee_data;

extern uint8_t Radar_double_hurt_chance;
#define MAX_REFEREE_DATA_LEN 45

#endif /* __REFEREE_H__ */
