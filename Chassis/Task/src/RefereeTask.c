#include "RefereeTask.h"

#include "ChassisController.h"
#include "GimbalReceive.h"
#include "NingCap.h"
#include "remote_control.h"
#include "debug.h"
#include "HeatControl.h"
#include "Referee.h"
#include "bsp_referee.h"
#include <stdio.h>

int Rest_UI_Flag;
bool ref_ready_flag; // 当前线程初始化完成标志
static void Ref_UI_LaneFlashShow(int16_t pitch_x100);
static void Ref_UI_SendStatusItem(uint8_t item_index, int16_t pitch_x100);
static bool Ref_UI_StatusInitialAddPending(void);
static bool Ref_UI_GetPriorityStatusItem(uint8_t *item_index);
static bool Ref_UI_DirectionArcUpdatePending(void);
static uint16_t Ref_UI_GetDirectionArcCenterAngle(float yaw_angle);
static uint8_t Ref_UI_IsAmmoAlert(void);

#define UI_STATUS_ITEM_COUNT       9U
#define UI_STATUS_ALL_ADDED_MASK   ((1U << UI_STATUS_ITEM_COUNT) - 1U)

typedef struct
{
	uint16_t added_mask;
	uint16_t valid_mask;
	uint16_t cap_voltage_x10;
	uint16_t direction_angle;
	portTickType ammo_tick;
	portTickType direction_tick;
	enum CHASSIS_MODE_ACTION chassis_mode;
	chassis_direction_e chassis_direction;
	uint8_t item_index;
	uint8_t generation;
	uint8_t readd_generation;
	uint8_t pc_on;
	uint8_t aim_mode;
	uint8_t friction_speed;
	uint8_t ammo_alert;
	uint8_t ammo_color;
} RefUiState;

typedef struct
{
	char name[4];
	uint16_t char_size;
	uint16_t length;
	uint16_t width;
	uint16_t x;
	uint16_t y;
} RefUiTextConfig;

static RefUiState ui_state = {0};

static const RefUiTextConfig UI_TEXT_GIM   = {"gim", 17, 24, 3,   60, 750};
static const RefUiTextConfig UI_TEXT_CHA   = {"cha", 17, 12, 3,   60, 700};
static const RefUiTextConfig UI_TEXT_PC    = {"pc ", 17,  7, 3,   60, 650};
static const RefUiTextConfig UI_TEXT_AIM   = {"aim", 20, 12, 3, 1280, 700};
static const RefUiTextConfig UI_TEXT_SPEED = {"SPD", 15, 15, 3, 1460, 590};
static const RefUiTextConfig UI_TEXT_CAP   = {"CAP", 25, 12, 3,  860, 110};
static const RefUiTextConfig UI_TEXT_FLAG  = {"fal", 30,  4, 3,  910, 880};
static const RefUiTextConfig UI_TEXT_AMMO  = {"amm", 100, 10, 6,  600, 750};

#define AMMO_ALERT_COLOR_INTERVAL_MS 400U
static const uint8_t ui_ammo_alert_colors[] = {
	UI_Color_Yellow,
	UI_Color_Green,
	UI_Color_Orange,
	UI_Color_Purple,
	UI_Color_Pink,
	UI_Color_Cyan,
	UI_Color_White
};

#define CAP_BAR_UI_START_X 750
#define CAP_BAR_UI_START_Y 35
#define CAP_BAR_LENGTH     420
#define CAP_BAR_WIDTH      30

/* 车道线透视参数：以 Pitch = -15.95° 时的交汇点为基准。 */
#define LANE_PITCH_REFERENCE_X100   (-1595)
#define LANE_VANISHING_Y_REFERENCE  600
#define LANE_VANISHING_Y_PER_DEGREE 10
#define LANE_VANISHING_Y_MIN         200
#define LANE_VANISHING_Y_MAX         900

/* 环绕原生中央准心圆环的底盘方向指示器。 */
#define DIRECTION_ARC_CENTER_X        960U
#define DIRECTION_ARC_CENTER_Y        540U
#define DIRECTION_ARC_RADIUS          82U
#define DIRECTION_ARC_WIDTH           10U
#define DIRECTION_ARC_HALF_ANGLE       30U
#define DIRECTION_ARC_YAW_INTERVAL_MS  50U
#define DIRECTION_ARC_KEEPALIVE_MS     200U

/*******************************************************************************************************
Ref任务初始化
********************************************************************************************************/
void Ref_Init(void)
{
	Referee_StructInit();
	fifo_s_init(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);

	osDelay(100);
}

/*******************************************************************************************************
Ref任务
********************************************************************************************************/
void Refereetask(void const *argument)
{
	static uint16_t ui_update_counter = 0;
	static bool ui_clear_sent = false;
	static bool static_ui_added = false;
	static uint8_t steady_ui_phase = 0;
	static uint8_t fri_refresh_slot = 0;
	static int16_t pitch_snapshot_x100 = 0;
	uint8_t ui_tx_slot;
	uint8_t priority_status_item;
	portTickType last_wake_time;

	Ref_Init();
	last_wake_time = xTaskGetTickCount();
	while (1)
	{
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);

		/* 机器人 ID 有效后才能确定选手端接收者。 */
		if (Robot_ID_Current != 0)
		{
			/* 启动时清理旧 UI，避免合并 GIM/CHA 字符串后出现重影。 */
			if (!ui_clear_sent)
			{
				UI_Delete.Delete.operate_tpye = UI_Delete_All;
				UI_Delete.Delete.layer = 0;
				UI_PushUp_Delete(&UI_Delete, Robot_ID_Current);
				ui_clear_sent = true;
				ui_update_counter = 0;
			}
			/* 首次添加 UI。 */
			else if (!static_ui_added)
			{
				/* 删除包后留出 40 ms，再发送较长的七图形包。 */
				if (ui_update_counter >= 4U)
				{
					Sightglass_static_show();
					static_ui_added = true;
					ui_state.readd_generation++;
					steady_ui_phase = 0;
					fri_refresh_slot = 0;
					ui_update_counter = 0;
				}
			}
			/* 每 5 秒低频重新 Add，选手端清空 UI 后可以自动恢复。 */
			else if (ui_update_counter >= 500)
			{
				Sightglass_static_show();
				ui_state.readd_generation++;
				steady_ui_phase = 0;
				fri_refresh_slot = 0;
				ui_update_counter = 0;
			}
			else
			{
				/*
				 * 0x0301 总频率按 30 Hz 调度：100 ms 内在 40/70/100 ms
				 * 各发送一包。初始化时优先发送全部字符串；稳态时
				 * 依次发送车道线、Pitch 状态行和辅助 UI。
				 */
				ui_tx_slot = (uint8_t)(ui_update_counter % 10U);
				if ((ui_tx_slot == 0U) || (ui_tx_slot == 4U) || (ui_tx_slot == 7U))
				{
					if (Ref_UI_StatusInitialAddPending())
					{
						/* GIM、CHA、PC 排在前三项，约 100 ms 完成主状态区加载。 */
						Sightglass1_flash_show();
					}
					else if (Ref_UI_DirectionArcUpdatePending())
					{
						/*
						 * 底盘方向变化时绕过普通状态队列。
						 * 通过重复发送和 5 Hz 保活刷新，避免单帧丢失后
						 * 准心圆弧长期停留在错误位置。
						 */
						Ref_UI_SendStatusItem(8U, gimbal_receiver_pack2.gimbal_pitch);
					}
					else if (steady_ui_phase == 0U)
					{
						pitch_snapshot_x100 = gimbal_receiver_pack2.gimbal_pitch;
						Ref_UI_LaneFlashShow(pitch_snapshot_x100);
						steady_ui_phase = 1U;
					}
					else if (steady_ui_phase == 1U)
					{
						/* Pitch 文字和车道线使用同一个快照。 */
						Ref_UI_SendStatusItem(0U, pitch_snapshot_x100);
						steady_ui_phase = 2U;
					}
					else
					{
						/*
						 * 每秒 10 个第三时隙中，在第 3/6/10 个时隙固定刷新
						 * FRI_SPEED，保证 3 Hz；其余时隙用于超电条或变化状态。
						 */
						fri_refresh_slot++;
						if (fri_refresh_slot >= 10U)
							fri_refresh_slot = 0U;

						if ((fri_refresh_slot == 3U) ||
							(fri_refresh_slot == 6U) ||
							(fri_refresh_slot == 0U))
						{
							Ref_UI_SendStatusItem(4U, gimbal_receiver_pack2.gimbal_pitch);
						}
						else if (Ref_UI_GetPriorityStatusItem(&priority_status_item))
						{
							Ref_UI_SendStatusItem(priority_status_item,
											  gimbal_receiver_pack2.gimbal_pitch);
						}
						else
						{
							Sightglass2_flash_show();
						}
						steady_ui_phase = 0U;
					}
				}
			}
		}

		ui_update_counter++;
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

void Sightglass_static_show(void)
{
	//两条车道线和五条准心左车道线：LLL（Left Lane Line）右车道线：RLL（Right Lane Line） 准心 crosshair ch
	UI_Draw_Line(&UI_Graph7.Graphic[0], "LLL", UI_Graph_Add, 1, UI_Color_Green, 3, 460, 100, 930, 600);
	UI_Draw_Line(&UI_Graph7.Graphic[1], "RLL", UI_Graph_Add, 1, UI_Color_Green, 3, 1460, 100, 990, 600);
	UI_Draw_Line(&UI_Graph7.Graphic[2], "ch1", UI_Graph_Add, 1, UI_Color_Orange, 2, 930, 470, 990, 470);
	UI_Draw_Line(&UI_Graph7.Graphic[3], "ch2", UI_Graph_Add, 1, UI_Color_Orange, 2, 930, 440, 990, 440);
	UI_Draw_Line(&UI_Graph7.Graphic[4], "ch3", UI_Graph_Add, 1, UI_Color_Orange, 2, 930, 410, 990, 410);
	UI_Draw_Line(&UI_Graph7.Graphic[5], "ch4", UI_Graph_Add, 1, UI_Color_Orange, 2, 930, 380, 990, 380);
	UI_Draw_Line(&UI_Graph7.Graphic[6], "ch5", UI_Graph_Add, 1, UI_Color_Orange, 2, 930, 350, 990, 350);


	UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
}

/**
 * @brief 将底盘模式枚举转换为 UI 显示字符串。
 * @param mode 底盘当前工作模式。
 * @return 固定长度的底盘模式文字；未知模式默认显示 DOWN。
 */
static const char *Ref_UI_GetChassisModeText(enum CHASSIS_MODE_ACTION mode)
{
	switch (mode)
	{
	case NOT_CONTROL_MODE:
		return "DOWN    ";
	case NOT_FOLLOW_GIMBAL:
		return "NOT_FOLL";
	case FOLLOW_GIMBAL:
		return "FOLLOW  ";
	case CV_ROTATE:
		return "ROTATE  ";
	default:
		return "DOWN    ";
	}
}

/**
 * @brief 将云台模式枚举转换为 UI 显示字符串。
 * @param mode 云台当前工作模式。
 * @return 固定长度的云台模式文字；未知模式默认显示 DOWN。
 */
static const char *Ref_UI_GetGimbalModeText(enum GIMBAL_ACTION mode)
{
	switch (mode)
	{
	case GIMBAL_POWER_DOWN:
		return "DOWN    ";
	case GIMBAL_ACT_MODE:
		return "ACT     ";
	case GIMBAL_AUTO_AIM_MODE:
		return "AUTOAIM ";
	case GIMBAL_SMALL_BUFF_MODE:
		return "SMA_BUFF";
	case GIMBAL_BIG_BUFF_MODE:
		return "BIG_BUFF";
	default:
		return "DOWN    ";
	}
}

/**
 * @brief 将自瞄模式编号转换为 UI 显示字符串。
 * @param aim_mode 自瞄模式编号：0 无自瞄，1 标准自瞄，2 小符，3 大符。
 * @return 固定长度的自瞄模式文字；未知编号默认显示 NO_AIM。
 */
static const char *Ref_UI_GetAimModeText(uint8_t aim_mode)
{
	switch (aim_mode)
	{
	case 0:
		return "NO_AIM      ";
	case 1:
		return "STANDARD AIM";
	case 2:
		return "SMALL BUFF  ";
	case 3:
		return "BIG BUFF    ";
	default:
		return "NO_AIM      ";
	}
}

/* 补齐固定长度文字并完成组包、发送，避免短文字覆盖后残留。 */
static void Ref_UI_SendText(const RefUiTextConfig *config,
							uint8_t operate,
							uint8_t color,
							const char *text)
{
	size_t copy_length;
	uint16_t length = config->length;

	if (length > sizeof(UI_String.String.stringdata))
		length = sizeof(UI_String.String.stringdata);

	memset(UI_String.String.stringdata, ' ', sizeof(UI_String.String.stringdata));
	copy_length = strlen(text);
	if (copy_length > length)
		copy_length = length;
	memcpy(UI_String.String.stringdata, text, copy_length);

	UI_Draw_String(&UI_String.String,
				   (char *)config->name,
				   operate,
				   2,
				   color,
				   config->char_size,
				   length,
				   config->width,
				   config->x,
				   config->y,
				   (char *)UI_String.String.stringdata);
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

static void Ref_UI_StatusSyncGeneration(void)
{
	if (ui_state.generation != ui_state.readd_generation)
	{
		ui_state.generation = ui_state.readd_generation;
		ui_state.item_index = 0;
		ui_state.added_mask = 0;
		ui_state.valid_mask = 0;
	}
}

static bool Ref_UI_StatusInitialAddPending(void)
{
	Ref_UI_StatusSyncGeneration();
	return ui_state.added_mask != UI_STATUS_ALL_ADDED_MASK;
}

/* 将云台 Yaw 转换为圆环角度：正上方为 0 度，顺时针递增。 */
static uint16_t Ref_UI_GetDirectionArcCenterAngle(float yaw_angle)
{
	return (uint16_t)((int32_t)(GIMBAL_MOTOR_SIGN *
							   (yaw_angle - GIMBAL_FOLLOW_ZERO) +
							   720.5f) % 360);
}

/* Yaw 变化最高约 15 Hz 刷新，静止时保持 5 Hz 重发。 */
static bool Ref_UI_DirectionArcUpdatePending(void)
{
	portTickType elapsed;
	uint16_t angle;

	if ((ui_state.added_mask & (1U << 8)) == 0U)
		return false;

	elapsed = (portTickType)(xTaskGetTickCount() - ui_state.direction_tick);
	angle = Ref_UI_GetDirectionArcCenterAngle(infantry.yaw_angle);
	return (ui_state.chassis_direction != infantry.chassis_direction) ||
		   (elapsed >= pdMS_TO_TICKS(DIRECTION_ARC_KEEPALIVE_MS)) ||
		   ((angle != ui_state.direction_angle) &&
			(elapsed >= pdMS_TO_TICKS(DIRECTION_ARC_YAW_INTERVAL_MS)));
}

static uint8_t Ref_UI_IsAmmoAlert(void)
{
	return (global_debugger.referee_debugger.state == ON) &&
		   (Projectile_Allowance.projectile_allowance_17mm < 30);
}

/**
 * @brief 查找需要优先刷新的主状态项。
 */
static bool Ref_UI_GetPriorityStatusItem(uint8_t *item_index)
{
	uint16_t cap_voltage_x10;
	uint8_t ammo_alert;
	int32_t cap_voltage_delta;

	if (item_index == NULL)
		return false;

	*item_index = 0U;
	if (((ui_state.valid_mask & (1U << 1)) == 0U) ||
		(ui_state.chassis_mode != remote_controller.control_mode_action))
	{
		*item_index = 1U;
		return true;
	}

	if (((ui_state.valid_mask & (1U << 2)) == 0U) ||
		(ui_state.pc_on != (uint8_t)gimbal_receiver_pack1.is_pc_on))
	{
		*item_index = 2U;
		return true;
	}

	if (((ui_state.valid_mask & (1U << 3)) == 0U) ||
		(ui_state.aim_mode != gimbal_receiver_pack1.aim_mode))
	{
		*item_index = 3U;
		return true;
	}

	if (((ui_state.valid_mask & (1U << 4)) == 0U) ||
		(ui_state.friction_speed != gimbal_receiver_pack1.set_friction_speed))
	{
		*item_index = 4U;
		return true;
	}

	ammo_alert = Ref_UI_IsAmmoAlert();
	if (((ui_state.valid_mask & (1U << 7)) == 0U) ||
		(ui_state.ammo_alert != ammo_alert) ||
		((portTickType)(xTaskGetTickCount() - ui_state.ammo_tick) >=
		 pdMS_TO_TICKS(AMMO_ALERT_COLOR_INTERVAL_MS)))
	{
		*item_index = 7U;
		return true;
	}

	/* 电压文字每变化 0.5 V 才更新，避免抢占超电进度条带宽。 */
	cap_voltage_x10 = (uint16_t)(cap_controller.cap_vol * 10.0f + 0.5f);
	cap_voltage_delta = (int32_t)cap_voltage_x10 - (int32_t)ui_state.cap_voltage_x10;
	if (cap_voltage_delta < 0)
		cap_voltage_delta = -cap_voltage_delta;
	if (((ui_state.valid_mask & (1U << 5)) == 0U) ||
		(cap_voltage_delta >= 5))
	{
		*item_index = 5U;
		return true;
	}

	return false;
}

/**
 * @brief 发送指定的状态字符串。
 *
 * GIM、CHA 的固定标签和动态数值合并为单个图形，减少首次
 * 加载需要的 0x0301 数据包数。
 */
static void Ref_UI_SendStatusItem(uint8_t item_index, int16_t pitch_x100)
{
	uint8_t operate;
	char text[30];
	uint16_t cap_voltage_x10;
	uint8_t ammo_alert;
	int pitch_value;
	int pitch_abs;
	chassis_direction_e chassis_direction_snapshot;
	uint16_t direction_arc_center_angle;

	if ((Robot_ID_Current == 0) || (item_index >= UI_STATUS_ITEM_COUNT))
		return;

	Ref_UI_StatusSyncGeneration();
	operate = (ui_state.added_mask & (1U << item_index)) ? UI_Graph_Change : UI_Graph_Add;
	memset(text, 0, sizeof(text));

	switch (item_index)
	{
	case 0:
		/* 云台模式、Pitch 和 GIM: 标签合并为一行。 */
		pitch_value = (int)pitch_x100;
		pitch_abs = (pitch_value < 0) ? -pitch_value : pitch_value;
		snprintf(text, sizeof(text), "GIM:%-8s P:%s%d.%02d",
				 Ref_UI_GetGimbalModeText(remote_controller.gimbal_action),
				 pitch_value < 0 ? "-" : "",
				 pitch_abs / 100,
				 pitch_abs % 100);
		Ref_UI_SendText(&UI_TEXT_GIM, operate, UI_Color_Green, text);
		break;

	case 1:
		/* 底盘模式和 CHA: 标签合并为一行。 */
		snprintf(text, sizeof(text), "CHA:%s",
				 Ref_UI_GetChassisModeText(remote_controller.control_mode_action));
		Ref_UI_SendText(&UI_TEXT_CHA, operate, UI_Color_Green, text);
		ui_state.chassis_mode = remote_controller.control_mode_action;
		ui_state.valid_mask |= (1U << 1);
		break;

	case 2:
		/* PC 状态原本就是单包动态更新。 */
		Ref_UI_SendText(&UI_TEXT_PC, operate,
						gimbal_receiver_pack1.is_pc_on ? UI_Color_Green : UI_Color_Orange,
						gimbal_receiver_pack1.is_pc_on ? "PC :ON " : "PC :OFF");
		ui_state.pc_on = (uint8_t)gimbal_receiver_pack1.is_pc_on;
		ui_state.valid_mask |= (1U << 2);
		break;

	case 3:
		Ref_UI_SendText(&UI_TEXT_AIM, operate, UI_Color_Orange,
						Ref_UI_GetAimModeText(gimbal_receiver_pack1.aim_mode));
		ui_state.aim_mode = gimbal_receiver_pack1.aim_mode;
		ui_state.valid_mask |= (1U << 3);
		break;

	case 4:
		/* 摩擦轮标签和速度合并，又减少一个字符串包。 */
		snprintf(text, sizeof(text), "FRI_SPEED:%u.%u",
				 (unsigned int)(gimbal_receiver_pack1.set_friction_speed / 10U),
				 (unsigned int)(gimbal_receiver_pack1.set_friction_speed % 10U));
		Ref_UI_SendText(&UI_TEXT_SPEED, operate, UI_Color_Green, text);
		ui_state.friction_speed = gimbal_receiver_pack1.set_friction_speed;
		ui_state.valid_mask |= (1U << 4);
		break;

	case 5:
		cap_voltage_x10 = (uint16_t)(cap_controller.cap_vol * 10.0f + 0.5f);
		snprintf(text, sizeof(text), "CAP:%u.%uV",
				 (unsigned int)(cap_voltage_x10 / 10U),
				 (unsigned int)(cap_voltage_x10 % 10U));
		Ref_UI_SendText(&UI_TEXT_CAP, operate, UI_Color_Green, text);
		ui_state.cap_voltage_x10 = cap_voltage_x10;
		ui_state.valid_mask |= (1U << 5);
		break;

	case 6:
		Ref_UI_SendText(&UI_TEXT_FLAG, operate, UI_Color_Green, "NIKO");
		break;

	case 7:
		ammo_alert = Ref_UI_IsAmmoAlert();
		if (ammo_alert)
		{
			/*
			 * A deleted string must be added again; Change cannot recreate it.
			 * While the alert remains active, use Change for color animation.
			 */
			operate = ((ui_state.added_mask & (1U << 7)) != 0U) &&
					  (ui_state.ammo_alert != 0U)
						  ? UI_Graph_Change
						  : UI_Graph_Add;
			Ref_UI_SendText(&UI_TEXT_AMMO, operate,
							ui_ammo_alert_colors[ui_state.ammo_color],
							"BUY AMMO!");
		}
		else
		{
			/* Spaces do not erase an existing client string reliably. */
			Ref_UI_SendText(&UI_TEXT_AMMO, UI_Graph_Delete,
							UI_Color_Orange, "");
		}
		ui_state.ammo_alert = ammo_alert;
		if (ammo_alert)
		{
			ui_state.ammo_color = (uint8_t)((ui_state.ammo_color + 1U) %
				(sizeof(ui_ammo_alert_colors) / sizeof(ui_ammo_alert_colors[0])));
			ui_state.ammo_tick = xTaskGetTickCount();
		}
		else
		{
			ui_state.ammo_color = 0U;
			/*
			 * Keep retrying Delete while the alert is hidden.  A single lost
			 * UI packet must not leave stale "BUY AMMO!" text on the client.
			 */
			ui_state.ammo_tick = xTaskGetTickCount();
		}
		ui_state.valid_mask |= (1U << 7);
		break;

	case 8:
		chassis_direction_snapshot = infantry.chassis_direction;
		direction_arc_center_angle =
			Ref_UI_GetDirectionArcCenterAngle(infantry.yaw_angle);
		UI_Draw_Arc(&UI_Graph1.Graphic[0], "DIR", operate, 2,
					(chassis_direction_snapshot == CHASSIS_FRONT)
						? UI_Color_Green
						: UI_Color_Orange,
					(direction_arc_center_angle + 360U - DIRECTION_ARC_HALF_ANGLE) % 360U,
					(direction_arc_center_angle + DIRECTION_ARC_HALF_ANGLE) % 360U,
					DIRECTION_ARC_WIDTH,
					DIRECTION_ARC_CENTER_X, DIRECTION_ARC_CENTER_Y,
					DIRECTION_ARC_RADIUS, DIRECTION_ARC_RADIUS);
		UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
		ui_state.chassis_direction = chassis_direction_snapshot;
		ui_state.direction_angle = direction_arc_center_angle;
		ui_state.direction_tick = xTaskGetTickCount();
		ui_state.valid_mask |= (1U << 8);
		break;

	default:
		return;
	}

	ui_state.added_mask |= (uint16_t)(1U << item_index);
}

/**
 * @brief 按顺序发送下一个状态项。
 */
void Sightglass1_flash_show(void)
{
	if (Robot_ID_Current == 0)
		return;

	Ref_UI_StatusSyncGeneration();
	if (ui_state.item_index >= UI_STATUS_ITEM_COUNT)
		ui_state.item_index = 0;

	Ref_UI_SendStatusItem(ui_state.item_index,
						  gimbal_receiver_pack2.gimbal_pitch);
	ui_state.item_index = (uint8_t)((ui_state.item_index + 1U) %
									UI_STATUS_ITEM_COUNT);
}

/**
 * @brief 根据 Pitch 计算车道线交汇点的 UI Y 坐标。
 *
 * 裁判端 UI 坐标原点在左下角，因此 Pitch 变大（相机抬起）时，
 * 交汇点的 UI Y 坐标减小，映射到屏幕后会向下移动。
 */
static uint16_t Ref_UI_GetLaneVanishingY(int16_t pitch_x100)
{
	int32_t vanishing_y;

	vanishing_y = LANE_VANISHING_Y_REFERENCE -
				  (((int32_t)pitch_x100 - LANE_PITCH_REFERENCE_X100) *
				   LANE_VANISHING_Y_PER_DEGREE) / 100;

	if (vanishing_y < LANE_VANISHING_Y_MIN)
		vanishing_y = LANE_VANISHING_Y_MIN;
	if (vanishing_y > LANE_VANISHING_Y_MAX)
		vanishing_y = LANE_VANISHING_Y_MAX;

	return (uint16_t)vanishing_y;
}

/**
 * @brief 绘制随 Pitch 变化的左右车道线。
 * @param LeftGraphic  左车道线图形数据地址
 * @param RightGraphic 右车道线图形数据地址
 * @param GraphOperate UI_Graph_Add 或 UI_Graph_Change
 */
static void Ref_UI_DrawLaneLines(graphic_data_struct_t *LeftGraphic,
								 graphic_data_struct_t *RightGraphic,
								 uint8_t GraphOperate,
								 int16_t pitch_x100)
{
	uint16_t vanishing_y = Ref_UI_GetLaneVanishingY(pitch_x100);

	UI_Draw_Line(LeftGraphic,
				 "LLL",
				 GraphOperate,
				 1,
				 UI_Color_Green,
				 3,
				 460,
				 100,
				 930,
				 vanishing_y);
	UI_Draw_Line(RightGraphic,
				 "RLL",
				 GraphOperate,
				 1,
				 UI_Color_Green,
				 3,
				 1460,
				 100,
				 990,
				 vanishing_y);
}

/**
 * @brief 在车道线调度时隙内按 Pitch 变化刷新左右车道线。
 *
 * 车道线使用独立的两图形包，不再等待电容条的五图形包。
 * 每次发送时用同一个 Pitch 快照计算两条线，保证左右线同步。
 */
static void Ref_UI_LaneFlashShow(int16_t pitch_x100)
{
	static bool lane_initialized = false;
	static int16_t last_pitch_x100 = 0;
	static uint8_t observed_generation = 0;

	if (Robot_ID_Current == 0)
		return;

	/* 静态 UI 重建后强制更新；Pitch 未变化时不重复占用带宽。 */
	if ((observed_generation == ui_state.readd_generation) &&
		lane_initialized &&
		(pitch_x100 == last_pitch_x100))
	{
		return;
	}

	observed_generation = ui_state.readd_generation;
	Ref_UI_DrawLaneLines(&UI_Graph2.Graphic[0],
						&UI_Graph2.Graphic[1],
						UI_Graph_Change,
						pitch_x100);
	UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);

	last_pitch_x100 = pitch_x100;
	lane_initialized = true;
}

/**
 * @brief  绘制超级电容能量条，颜色由电容电压状态决定。
 * @param  Graphic      图形数据地址
 * @param  GraphOperate UI_Graph_Add 或 UI_Graph_Change
 */
void drawCapBar(graphic_data_struct_t *Graphic, uint8_t GraphOperate)
{
	uint8_t color;
	float cap_percent = cap_controller.cap_energy_pecent;
	uint16_t cap_end_x;

	if (cap_percent < 0.0f)
		cap_percent = 0.0f;
	if (cap_percent > 1.0f)
		cap_percent = 1.0f;

	if (cap_controller.cap_vol_state == CapVol_High)
		color = UI_Color_Green;
	else if (cap_controller.cap_vol_state == CapVol_Middle)
		color = UI_Color_Yellow;
	else
		color = UI_Color_Orange;

	cap_end_x = CAP_BAR_UI_START_X + (uint16_t)(cap_percent * CAP_BAR_LENGTH);
	UI_Draw_Line(Graphic,
				 "bar",
				 GraphOperate,
				 1,
				 color,
				 CAP_BAR_WIDTH,
				 CAP_BAR_UI_START_X,
				 CAP_BAR_UI_START_Y,
				 cap_end_x,
				 CAP_BAR_UI_START_Y);
}

void Sightglass2_flash_show(void)
{
	static bool cap_bar_added = false;
	static uint8_t observed_generation = 0;
	uint8_t operate;

	if (Robot_ID_Current == 0)
		return;

	/* 静态 UI 重建后，电容条也重新 Add。 */
	if (observed_generation != ui_state.readd_generation)
	{
		observed_generation = ui_state.readd_generation;
		cap_bar_added = false;
	}

	operate = cap_bar_added ? UI_Graph_Change : UI_Graph_Add;
	UI_Draw_Rectangle(&UI_Graph2.Graphic[0], "cap", operate, 1,
					  UI_Color_White, 5,
					  CAP_BAR_UI_START_X,
					  CAP_BAR_UI_START_Y + CAP_BAR_WIDTH / 2,
					  CAP_BAR_UI_START_X + CAP_BAR_LENGTH,
					  CAP_BAR_UI_START_Y - CAP_BAR_WIDTH / 2);
	drawCapBar(&UI_Graph2.Graphic[1], operate);
	UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
	cap_bar_added = true;
}
