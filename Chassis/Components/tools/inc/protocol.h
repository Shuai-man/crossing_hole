/**
 ******************************************************************************
 * @file    protocol.h
 * @author  Karolance Future
 * @version V2.0.0
 * @date    2026/06/26
 * @brief   依据裁判系统 串口协议附录 V2.0.0 (2026)
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF 0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2

#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

typedef enum
{
  /* 0x000X — 比赛全局数据 */
  GAME_STATE_CMD_ID = 0x0001,        // 比赛状态数据
  GAME_RESULT_CMD_ID = 0x0002,       // 比赛结果数据
  GAME_ROBOT_HP_CMD_ID = 0x0003,     // 机器人血量数据

  /* 0x010X — 场地/事件数据 */
  FIELD_EVENTS_CMD_ID = 0x0101,              // 场地事件数据
  SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,  // 补给站动作标识
  REFEREE_WARNING_CMD_ID = 0x0104,           // 裁判警告信息
  DART_REMAINING_TIME_CMD_ID = 0x0105,       // 飞镖发射口倒计时

  /* 0x020X — 机器人状态数据 */
  ROBOT_STATE_CMD_ID = 0x0201,         // 比赛机器人状态
  POWER_HEAT_DATA_CMD_ID = 0x0202,     // 实时功率热量数据
  ROBOT_POS_CMD_ID = 0x0203,           // 机器人位置
  BUFF_MUSK_CMD_ID = 0x0204,           // 机器人增益
  ROBOT_HURT_CMD_ID = 0x0206,          // 伤害状态
  SHOOT_DATA_CMD_ID = 0x0207,          // 实时射击信息
  BULLET_REMAINING_CMD_ID = 0x0208,    // 允许发弹量与金币
  RFID_STATUS_CMD_ID = 0x0209,         // 机器人RFID模块状态 (V2.0.0)
  DART_CLIENT_CMD_ID = 0x020A,         // 飞镖选手端指令数据 (V2.0.0)
  GROUND_ROBOT_POS_CMD_ID = 0x020B,    // 地面机器人位置 (V2.0.0)
  RADAR_MARK_CMD_ID = 0x020C,          // 雷达标记进度 (V2.0.0)
  SENTRY_INFO_CMD_ID = 0x020D,         // 哨兵自主决策信息同步 (V2.0.0)
  RADAR_INFO_CMD_ID = 0x020E,          // 雷达自主决策信息同步 (V2.0.0)

  /* 0x030X — 交互数据 */
  STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301, // 机器人间通信
  ROBOT_COMMAND_CMD_ID = 0x0303,            // 小地图下发信息标识
  MAP_ROBOT_DATA_CMD_ID = 0x0305,           // 小地图接收双方机器人坐标 (V2.0.0)
  // 0x0306                                 // 自定义控制器与选手端交互 (图传链路)
  MAP_DATA_CMD_ID = 0x0307,              // 选手端小地图接收路径数据
  CUSTOM_INFO_CMD_ID = 0x0308,           // 选手端小地图接收消息

  /* 0x0A0X — 雷达无线链路 (V2.0.0) */
  ENEMY_POSITION_CMD_ID = 0x0A01,       // 对方机器人位置坐标
  ENEMY_HP_CMD_ID = 0x0A02,             // 对方机器人血量
  ENEMY_PROJECTILE_CMD_ID = 0x0A03,     // 对方机器人允许发弹量
  ENEMY_MACRO_STATE_CMD_ID = 0x0A04,    // 对方队伍宏观状态
  ENEMY_BUFF_CMD_ID = 0x0A05,           // 对方机器人增益效果
  ENEMY_KEY_CMD_ID = 0x0A06,            // 对方干扰波密钥

  IDCustomData,
} referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF = 0,
  STEP_LENGTH_LOW = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16 = 5,
} unpack_step_e;

#pragma pack(push, 1)

typedef struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

#pragma pack(pop)

#endif // ROBOMASTER_PROTOCOL_H
