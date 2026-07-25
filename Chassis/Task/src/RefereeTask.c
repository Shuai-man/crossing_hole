#include "RefereeTask.h"

#include "ChassisController.h"
#include "GimbalReceive.h"
#include "NingCap.h"
#include "remote_control.h"
#include "debug.h"
#include "Referee.h"
#include "bsp_referee.h"
#include <stdio.h>

int Rest_UI_Flag;
bool ref_ready_flag; // 当前线程初始化完成标志
/* 每次重新发送静态 UI 时递增，通知动态 UI 重新使用 Add。 */
static uint8_t ui_readd_generation = 0;
static void Show_NIKO(uint8_t operate);
static void Ref_UI_LaneFlashShow(int16_t pitch_x100);
static void Ref_UI_SendStatusItem(uint8_t item_index, int16_t pitch_x100);
static bool Ref_UI_StatusInitialAddPending(void);
static bool Ref_UI_GetPriorityStatusItem(uint8_t *item_index);

#define UI_STATUS_ITEM_COUNT       9U
#define UI_STATUS_ALL_ADDED_MASK   ((1U << UI_STATUS_ITEM_COUNT) - 1U)

static uint8_t ui_status_item_index = 0;
static uint16_t ui_status_added_mask = 0;
static uint8_t ui_status_observed_generation = 0;
static uint16_t ui_priority_status_valid_mask = 0;
static enum CHASSIS_MODE_ACTION ui_last_chassis_mode;
static uint8_t ui_last_pc_on = 0;
static uint8_t ui_last_aim_mode = 0;
static uint8_t ui_last_friction_speed = 0;
static uint16_t ui_last_cap_voltage_x10 = 0;
static uint8_t ui_last_ammo_alert = 0;
static chassis_direction_e ui_last_chassis_direction = CHASSIS_FRONT;

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

	Ref_Init();
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
					ui_readd_generation++;
					steady_ui_phase = 0;
					fri_refresh_slot = 0;
					ui_update_counter = 0;
				}
			}
			/* 每 5 秒低频重新 Add，选手端清空 UI 后可以自动恢复。 */
			else if (ui_update_counter >= 500)
			{
				Sightglass_static_show();
				ui_readd_generation++;
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
		osDelay(10);
	}
}

void Sightglass_static_show(void)
{
	//两条车道线和五条准心左车道线：LLL（Left Lane Line）右车道线：RLL（Right Lane Line） 准心 crosshair ch
	UI_Draw_Line(&UI_Graph7.Graphic[0], "LLL", UI_Graph_Add, 1, UI_Color_Green, 3, 460, 100, 930, 600);
	UI_Draw_Line(&UI_Graph7.Graphic[1], "RLL", UI_Graph_Add, 1, UI_Color_Green, 3, 1460, 100, 990, 600);
	UI_Draw_Line(&UI_Graph7.Graphic[2], "ch1", UI_Graph_Add, 1, UI_Color_Orange, 2, 915, 470, 1005, 470);
	UI_Draw_Line(&UI_Graph7.Graphic[3], "ch2", UI_Graph_Add, 1, UI_Color_Orange, 2, 915, 440, 1005, 440);
	UI_Draw_Line(&UI_Graph7.Graphic[4], "ch3", UI_Graph_Add, 1, UI_Color_Orange, 2, 915, 410, 1005, 410);
	UI_Draw_Line(&UI_Graph7.Graphic[5], "ch4", UI_Graph_Add, 1, UI_Color_Orange, 2, 915, 380, 1005, 380);
	UI_Draw_Line(&UI_Graph7.Graphic[6], "ch5", UI_Graph_Add, 1, UI_Color_Orange, 2, 915, 350, 1005, 350);


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

/**
 * @brief 组装并发送一条状态字符串 UI。
 *
 * UI 字符串需要使用固定长度。如果用短文字刷新长文字，旧文字超出
 * 新文字长度的部分不会自动被覆盖，可能残留在屏幕上。因此函数会
 * 先用空格填充字符串缓冲区，再复制实际文字，清除旧文字残留。
 * 当实际文字过长时，只保留 string_length 个字符。
 * @param name UI 字符串的名称。
 * @param operate UI 操作类型，例如 Add 或 Change。
 * @param color 字符串颜色。
 * @param char_size 字符大小。
 * @param string_length UI 字符串长度。
 * @param width 字符宽度。
 * @param start_x 字符串起始 X 坐标。
 * @param start_y 字符串起始 Y 坐标。
 * @param text 要显示的文字。
 */
static void Ref_UI_SendStatusString(char *name,
										 uint8_t operate,
										 uint8_t color,
										 uint16_t char_size,
										 uint16_t string_length,
										 uint16_t width,
										 uint16_t start_x,
										 uint16_t start_y,
										 const char *text)
{
	char text_buffer[30];
	size_t copy_length;

	if (string_length > sizeof(text_buffer))
		string_length = sizeof(text_buffer);

	memset(text_buffer, ' ', sizeof(text_buffer));
	memset(UI_String.String.stringdata, ' ', sizeof(UI_String.String.stringdata));

	copy_length = strlen(text);
	if (copy_length > string_length)
		copy_length = string_length;
	memcpy(text_buffer, text, copy_length);

	UI_Draw_String(&UI_String.String,
					 name,
					 operate,
					 2,
					 color,
					 char_size,
					 string_length,
					 width,
					 start_x,
					 start_y,
					 text_buffer);
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

/**
 * @brief 显示队伍标识 NIKO。
 * @param operate UI 操作类型，例如 Add 或 Change。
 */
static void Show_NIKO(uint8_t operate)
{
	Ref_UI_SendStatusString("fal", operate, UI_Color_Green,
							30, 4, 3, 910, 880, "NIKO");
}

static void Ref_UI_StatusSyncGeneration(void)
{
	if (ui_status_observed_generation != ui_readd_generation)
	{
		ui_status_observed_generation = ui_readd_generation;
		ui_status_item_index = 0;
		ui_status_added_mask = 0;
		ui_priority_status_valid_mask = 0;
	}
}

static bool Ref_UI_StatusInitialAddPending(void)
{
	Ref_UI_StatusSyncGeneration();
	return ui_status_added_mask != UI_STATUS_ALL_ADDED_MASK;
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
	if (((ui_priority_status_valid_mask & (1U << 1)) == 0U) ||
		(ui_last_chassis_mode != remote_controller.control_mode_action))
	{
		*item_index = 1U;
		return true;
	}

	if (((ui_priority_status_valid_mask & (1U << 2)) == 0U) ||
		(ui_last_pc_on != (uint8_t)gimbal_receiver_pack1.is_pc_on))
	{
		*item_index = 2U;
		return true;
	}

	if (((ui_priority_status_valid_mask & (1U << 8)) == 0U) ||
		(ui_last_chassis_direction != infantry.chassis_direction))
	{
		*item_index = 8U;
		return true;
	}

	if (((ui_priority_status_valid_mask & (1U << 3)) == 0U) ||
		(ui_last_aim_mode != gimbal_receiver_pack1.aim_mode))
	{
		*item_index = 3U;
		return true;
	}

	if (((ui_priority_status_valid_mask & (1U << 4)) == 0U) ||
		(ui_last_friction_speed != gimbal_receiver_pack1.set_friction_speed))
	{
		*item_index = 4U;
		return true;
	}

	ammo_alert = (global_debugger.referee_debugger.state == ON &&
				  Projectile_Allowance.projectile_allowance_17mm <= 10U) ? 1U : 0U;
	if (((ui_priority_status_valid_mask & (1U << 7)) == 0U) ||
		(ui_last_ammo_alert != ammo_alert))
	{
		*item_index = 7U;
		return true;
	}

	/* 电压文字每变化 0.5 V 才更新，避免抢占超电进度条带宽。 */
	cap_voltage_x10 = (uint16_t)(cap_controller.cap_vol * 10.0f + 0.5f);
	cap_voltage_delta = (int32_t)cap_voltage_x10 - (int32_t)ui_last_cap_voltage_x10;
	if (cap_voltage_delta < 0)
		cap_voltage_delta = -cap_voltage_delta;
	if (((ui_priority_status_valid_mask & (1U << 5)) == 0U) ||
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
	uint16_t ammo_remaining;
	int pitch_value;
	int pitch_abs;

	if ((Robot_ID_Current == 0) || (item_index >= UI_STATUS_ITEM_COUNT))
		return;

	Ref_UI_StatusSyncGeneration();
	operate = (ui_status_added_mask & (1U << item_index)) ? UI_Graph_Change : UI_Graph_Add;
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
		Ref_UI_SendStatusString("gim", operate, UI_Color_Green,
								17, 24, 3, 60, 750, text);
		break;

	case 1:
		/* 底盘模式和 CHA: 标签合并为一行。 */
		snprintf(text, sizeof(text), "CHA:%s",
				 Ref_UI_GetChassisModeText(remote_controller.control_mode_action));
		Ref_UI_SendStatusString("cha", operate, UI_Color_Green,
								17, 12, 3, 60, 700, text);
		ui_last_chassis_mode = remote_controller.control_mode_action;
		ui_priority_status_valid_mask |= (1U << 1);
		break;

	case 2:
		/* PC 状态原本就是单包动态更新。 */
		Ref_UI_SendStatusString("pc ", operate,
								gimbal_receiver_pack1.is_pc_on ? UI_Color_Green : UI_Color_Orange,
								17, 7, 3, 60, 650,
								gimbal_receiver_pack1.is_pc_on ? "PC :ON " : "PC :OFF");
		ui_last_pc_on = (uint8_t)gimbal_receiver_pack1.is_pc_on;
		ui_priority_status_valid_mask |= (1U << 2);
		break;

	case 3:
		Ref_UI_SendStatusString("aim", operate, UI_Color_Orange,
								20, 12, 3, 1280, 700,
								Ref_UI_GetAimModeText(gimbal_receiver_pack1.aim_mode));
		ui_last_aim_mode = gimbal_receiver_pack1.aim_mode;
		ui_priority_status_valid_mask |= (1U << 3);
		break;

	case 4:
		/* 摩擦轮标签和速度合并，又减少一个字符串包。 */
		snprintf(text, sizeof(text), "FRI_SPEED:%u.%u",
				 (unsigned int)(gimbal_receiver_pack1.set_friction_speed / 10U),
				 (unsigned int)(gimbal_receiver_pack1.set_friction_speed % 10U));
		Ref_UI_SendStatusString("SPD", operate, UI_Color_Green,
								15, 15, 3, 1460, 590, text);
		ui_last_friction_speed = gimbal_receiver_pack1.set_friction_speed;
		ui_priority_status_valid_mask |= (1U << 4);
		break;

	case 5:
		cap_voltage_x10 = (uint16_t)(cap_controller.cap_vol * 10.0f + 0.5f);
		snprintf(text, sizeof(text), "CAP:%u.%uV",
				 (unsigned int)(cap_voltage_x10 / 10U),
				 (unsigned int)(cap_voltage_x10 % 10U));
		Ref_UI_SendStatusString("CAP", operate, UI_Color_Green,
								25, 12, 3, 860, 110, text);
		ui_last_cap_voltage_x10 = cap_voltage_x10;
		ui_priority_status_valid_mask |= (1U << 5);
		break;

	case 6:
		Show_NIKO(operate);
		break;

	case 7:
		ammo_remaining = Projectile_Allowance.projectile_allowance_17mm;
		Ref_UI_SendStatusString("amm", operate, UI_Color_Orange,
								60, 10, 3, 720, 800,
								(global_debugger.referee_debugger.state == ON && ammo_remaining <= 10U)
									? "BUY AMMO!"
									: "          ");
		ui_last_ammo_alert = (global_debugger.referee_debugger.state == ON && ammo_remaining <= 10U) ? 1U : 0U;
		ui_priority_status_valid_mask |= (1U << 7);
		break;

	case 8:
		Ref_UI_SendStatusString("DIR", operate,
								(infantry.chassis_direction == CHASSIS_FRONT)
									? UI_Color_Green
									: UI_Color_Orange,
								15, 10, 3, 1460, 550,
								(infantry.chassis_direction == CHASSIS_FRONT)
									? "HEAD:FRONT"
									: "HEAD:BACK ");
		ui_last_chassis_direction = infantry.chassis_direction;
		ui_priority_status_valid_mask |= (1U << 8);
		break;

	default:
		return;
	}

	ui_status_added_mask |= (uint16_t)(1U << item_index);
}

/**
 * @brief 按顺序发送下一个状态项。
 */
void Sightglass1_flash_show(void)
{
	if (Robot_ID_Current == 0)
		return;

	Ref_UI_StatusSyncGeneration();
	if (ui_status_item_index >= UI_STATUS_ITEM_COUNT)
		ui_status_item_index = 0;

	Ref_UI_SendStatusItem(ui_status_item_index,
						  gimbal_receiver_pack2.gimbal_pitch);
	ui_status_item_index++;
	if (ui_status_item_index >= UI_STATUS_ITEM_COUNT)
		ui_status_item_index = 0;
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
	if ((observed_generation == ui_readd_generation) &&
		lane_initialized &&
		(pitch_x100 == last_pitch_x100))
	{
		return;
	}

	observed_generation = ui_readd_generation;
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

	if (Robot_ID_Current == 0)
		return;

	/* 静态 UI 重建后，电容条也重新 Add。 */
	if (observed_generation != ui_readd_generation)
	{
		observed_generation = ui_readd_generation;
		cap_bar_added = false;
	}

	if (!cap_bar_added)
	{
		/* 超电进度条：首次创建。 */
		UI_Draw_Rectangle(&UI_Graph2.Graphic[0], "cap", UI_Graph_Add, 1, UI_Color_White, 5, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y + CAP_BAR_WIDTH / 2, CAP_BAR_UI_START_X + CAP_BAR_LENGTH, CAP_BAR_UI_START_Y - CAP_BAR_WIDTH / 2);
		drawCapBar(&UI_Graph2.Graphic[1], UI_Graph_Add);

		UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
		cap_bar_added = true;
	}
	else
	{
		/* 超电进度条：后续刷新。 */
		UI_Draw_Rectangle(&UI_Graph2.Graphic[0], "cap", UI_Graph_Change, 1, UI_Color_White, 5, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y + CAP_BAR_WIDTH / 2, CAP_BAR_UI_START_X + CAP_BAR_LENGTH, CAP_BAR_UI_START_Y - CAP_BAR_WIDTH / 2);
		drawCapBar(&UI_Graph2.Graphic[1], UI_Graph_Change);

		UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
	}
}
