
#include "RefereeTask.h"

//float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;
/**超电条
 * @brief 
 * @param[in] void
 */
void drawCapBar(graphic_data_struct_t *Graphic, uint8_t GraphOperate)
{
	uint8_t COLOR;

	if (cap_controller.cap_vol_state == CAP_VOL_HIGH)
		COLOR = UI_Color_Green;
	else if (cap_controller.cap_vol_state == CAP_VOL_MID)
		COLOR = UI_Color_Yellow;
	else
		COLOR = UI_Color_Orange;

	UI_Draw_Line(Graphic, "bar", GraphOperate, LAYER1, COLOR, CAP_BAR_WIDTH, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y, CAP_BAR_UI_START_X + (uint16_t)(cap_controller.cap_energy_pecent * CAP_BAR_LENGTH), CAP_BAR_UI_START_Y);
}
float DT;
//uint32_t T;
	uint16_t UI_PushUp_Counter = 261;

void Refereetask(void const * argument)
{
	portTickType xLastWakeTime;


	uint8_t is_pc_offline = 0;

	static char chassis_state[5][8] = {"OFFLINE ", "NOT_FOLL", "FOLLOW  ", "ROTATE  ", "ROTATE  "};
	static char gimbal_state[7][8] =  {"OFFLINE ", "ACT     ", "AUTOAIM ", "TEST    ", "SI      ", "SMA_BUFF", "BIG_BUFF"};

	uint16_t UI_PushUp_Counter_500;
	uint16_t UI_PushUp_Counter_60;
	uint16_t UI_PushUp_Counter_20;
	uint16_t UI_PushUp_Counter_10;
	
	while (1) //进入异常
	{
//			HAL_UART_Receive_DMA(&huart4, rx_buffer, sizeof(rx_buffer));
		xLastWakeTime = xTaskGetTickCount();
		Referee_UnpackFifoData();

		UI_PushUp_Counter++;
		UI_PushUp_Counter_500 = UI_PushUp_Counter % 500; // 5000毫秒
		UI_PushUp_Counter_60 = UI_PushUp_Counter % 60;	 // 600毫秒
		UI_PushUp_Counter_20 = UI_PushUp_Counter % 20;	 // 200毫秒
		UI_PushUp_Counter_10 = UI_PushUp_Counter % 10;	 // 100毫秒 
		/*****************************************************更新频率为5秒*****************************************************************/
		//坐标系原点在
		if (UI_PushUp_Counter_500 == 13) // 锟斤拷锟斤拷锟斤拷锟斤拷
		{
#if ROBOT == CHEN_JING_YUAN
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "lft", UI_Graph_Add, LAYER1, UI_Color_Green, 3, 210, 0, 640, 600);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "rgt", UI_Graph_Add, LAYER1, UI_Color_Green, 3, 1660, 0, 1278, 600);
#elif ROBOT == NIUNIU
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "lft", UI_Graph_Add, LAYER1, UI_Color_White, 3, 210, 0, 605, 600);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "rgt", UI_Graph_Add, LAYER1, UI_Color_White, 3, 1660, 0, 1278, 600);
#elif ROBOT == NIU_MO_SON
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "lft", UI_Graph_Add, LAYER1, UI_Color_White, 3, 210, 0, 640, 600);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "rgt", UI_Graph_Add, LAYER1, UI_Color_White, 3, 1695, 0, 1197, 600);
#elif ROBOT == QI_TIAN_DA_SHENG
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[0], "lft", UI_Graph_Add, LAYER1, UI_Color_White, 3, 210, 0, 605, 600);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "rgt", UI_Graph_Add, LAYER1, UI_Color_White, 3, 1660, 0, 1278, 600);
#endif		
			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);
		}
		else if(UI_PushUp_Counter_500 == 53)
		{
			//界面右侧，机器人的朝向
			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "bdy", UI_Graph_Add, LAYER1, UI_Color_Purple, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "dir", UI_Graph_Add, LAYER1, UI_Color_Orange, 5, 1700, 750, (uint16_t)(1700 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.sin_dir)), (uint16_t)(750 + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.cos_dir)));
			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
			 // 在矩形上方绘制字符串
			//  UI_Draw_String(&referee_data.UI_String.String, "201", UI_Graph_Add, LAYER2, UI_Color_White, 20, 10, 3, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2 - 200, "          ");
			//  UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		 
		}
		else if(UI_PushUp_Counter_500 == 131)
		{
			//辅瞄模式
			UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Add, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "NO_AIM      ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 103)
		{
			//云台运动状态
			UI_Draw_String(&referee_data.UI_String.String, "GIM", UI_Graph_Add, LAYER2, UI_Color_Green, 17, 10, 3, 60, 750, "GIM: ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 149) 
		{
			//PC是否在线
			is_pc_offline = !gimbal_receiver_pack1.is_pc_on;
			if (!is_pc_offline)
				UI_Draw_String(&referee_data.UI_String.String, "pc ", UI_Graph_Add, LAYER2, UI_Color_Green, 17, 14, 3, 60, 650, "PC      :ON");
			else
				UI_Draw_String(&referee_data.UI_String.String, "pc ", UI_Graph_Add, LAYER2, UI_Color_Orange, 17, 14, 3, 60, 650, "PC      :OFF");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
#if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
//		else if (UI_PushUp_Counter_500 == 179) 
//		{
//			//弹舱盖的状态
//			is_cover_off = remote_controller.bomb_bay ? 0 : 1;
//			if (!is_cover_off)
//				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Green,  18, 15, 3, 60, 800, "COVER   :OPEN ");
//			else
//				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Orange, 18, 15, 3, 60, 800, "COVER   :CLOSE");
//			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
//		}
#endif
		else if (UI_PushUp_Counter_500 == 211) 
		{
			//底盘模式
			UI_Draw_String(&referee_data.UI_String.String, "cha", UI_Graph_Add, LAYER2, UI_Color_Green, 30, 12, 3, 155, 600, chassis_state[remote_controller.control_mode_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 251) 
		{
			//云台模式
			UI_Draw_String(&referee_data.UI_String.String, "gim", UI_Graph_Add, LAYER2, UI_Color_Green, 17, 12, 3, 120, 650, gimbal_state[remote_controller.gimbal_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 283) 
		{
			//ID
			// UI_Draw_String(&referee_data.UI_String.String, "208", UI_Graph_Add, LAYER2, UI_Color_Green, 15, 8, 3, 600, 880, "ID:   ");
			// UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 317) 
		{
			//超电电压值
			UI_Draw_String(&referee_data.UI_String.String, "CAP", UI_Graph_Add, LAYER2, UI_Color_Green, 25, 11, 3, 860, 70, "CAP:     V");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if(UI_PushUp_Counter_500 == 347)
		{
			UI_Draw_String(&referee_data.UI_String.String, "SPD", UI_Graph_Add, LAYER2, UI_Color_Green, 15, 11, 3, 1200, 70, "FRI_SPEED:");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
//		else if (UI_PushUp_Counter_500 == 353) // PITCH锟斤拷锟斤拷
//		{
//			UI_Draw_String(&referee_data.UI_String.String, "201", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1280, 800, "PITCH: ");
//			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
//		}
		else if (UI_PushUp_Counter_500 == 353)
		{
			//显示摩擦轮转速
			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "spd", UI_Graph_Add, LAYER3, UI_Color_Orange, 15, 2, 3, 1200, 40, infantry.friction_speed);
			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 377) 
		{
			//雷达
			// UI_Draw_String(&referee_data.UI_String.String, "209", UI_Graph_Add, 2, UI_Color_Green, 15, 8, 3, 1280, 800, "Radar: ");
			// UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 389)
		{
			//超电进度条
			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "cap", UI_Graph_Add, LAYER1, UI_Color_White, 5, CAP_BAR_UI_START_X, CAP_BAR_UI_START_Y + CAP_BAR_WIDTH / 2, CAP_BAR_UI_START_X + CAP_BAR_LENGTH, CAP_BAR_UI_START_Y - CAP_BAR_WIDTH / 2);

			drawCapBar(&referee_data.UI_Graph2.Graphic[1], UI_Graph_Add);

			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 431)
		{
			//UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Add, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack1.gimbal_pitch / 100.0f);
			
			//闆疯揪鑷畾涔夋秷鎭�
			// UI_Draw_Int(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Add, 3, UI_Color_Orange, 15, 4, 1380, 800, Radar_double_hurt_chance);	
			
			// UI_Draw_Float(&referee_data.UI_Graph5.Graphic[1], "301", UI_Graph_Add, 3, UI_Color_Orange, 25, 2, 4, 960, 70, cap_controller.cap_vol);
			
			//ID
			// UI_Draw_Int(&referee_data.UI_Graph5.Graphic[2], "302", UI_Graph_Add, 3, UI_Color_Cyan, 20, 2, 640, 880, infantry.aim_id);
			
			// UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}
		else if (UI_PushUp_Counter_500 == 467)
		{
			UI_Draw_String(&referee_data.UI_String.String, "CHA", UI_Graph_Add, LAYER2, UI_Color_Green, 17, 10, 3, 60, 700, "CHA: ");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}

		/*****************************************************更新频率为600毫秒*****************************************************************/
		// 锟斤拷锟斤拷锟斤拷台锟斤拷锟斤拷状态UI
		else if (UI_PushUp_Counter_60 == 10)
		{
			UI_Draw_String(&referee_data.UI_String.String, "gim", UI_Graph_Change, LAYER2, UI_Color_Orange, 17, 12, 3, 220, 750, gimbal_state[remote_controller.gimbal_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		
//		else if(UI_PushUp_Counter_60 == 20)
//		{
//			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "501", UI_Graph_Change, 5, UI_Color_Purple, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
//			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "502", UI_Graph_Change, 5, UI_Color_Orange, 5, 1700, 750, (uint16_t)(1700 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.sin_dir)), (uint16_t)(750 + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * infantry.cos_dir)));
//			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
//		}

		// 锟斤拷锟狡碉拷锟教匡拷锟斤拷状态UI
		else if (UI_PushUp_Counter_60 == 30)
		{
			UI_Draw_String(&referee_data.UI_String.String, "cha", UI_Graph_Change, LAYER2, UI_Color_Orange, 30, 12, 3, 155, 600, chassis_state[remote_controller.control_mode_action]);
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}

		// UI锟斤拷锟斤拷(2Hz)
		else if (UI_PushUp_Counter_60 == 40)
		{
			// pitch锟斤拷锟�
//			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Change, 3, UI_Color_Pink, 15, 2, 4, 1380, 800, gimbal_receiver_pack1.gimbal_pitch / 100.0f);

			// Radar鍙屽€嶆槗浼ゆ鏁�
			// UI_Draw_Int(&referee_data.UI_Graph5.Graphic[0], "300", UI_Graph_Change, 3, UI_Color_Orange, 15, 4, 1380, 800, Radar_double_hurt_chance);			
			
				// UI_Draw_Float(&referee_data.UI_Graph5.Graphic[1], "301", UI_Graph_Change, 3, UI_Color_Orange, 25, 2, 4, 960, 70, cap_controller.cap_vol);
				// // 锟斤拷锟斤拷锟斤拷锟絀D
				// UI_Draw_Int(&referee_data.UI_Graph5.Graphic[2], "302", UI_Graph_Change, 3, UI_Color_Cyan, 20, 2, 640, 880, gimbal_receiver_pack1.autoaim_id);

				// UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}
#if ROBOT == NIUNIU || ROBOT == QI_TIAN_DA_SHENG
//		else if (UI_PushUp_Counter_60 == 50)
//		{
//			is_cover_off = remote_controller.bomb_bay ? 0 : 1;
//			if (!is_cover_off)
//				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Change, 2, UI_Color_Green,  18, 15, 3, 60, 800, "COVER   :OPEN ");
//			else
//				UI_Draw_String(&referee_data.UI_String.String, "003", UI_Graph_Change, 2, UI_Color_Orange, 18, 15, 3, 60, 800, "COVER   :CLOSE");
//			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
//		}
#endif
		else if (UI_PushUp_Counter_60 == 59)
		{
			is_pc_offline = !gimbal_receiver_pack1.is_pc_on;
			if (!is_pc_offline)
				UI_Draw_String(&referee_data.UI_String.String, "pc ", UI_Graph_Change, LAYER2, UI_Color_Green, 17, 14, 3, 60, 650, "PC       :ON");
			else
				UI_Draw_String(&referee_data.UI_String.String, "pc ", UI_Graph_Change, LAYER2, UI_Color_Orange, 17, 14, 3, 60, 650, "PC       :OFF");
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		// 更新频率为200毫秒---------------//
		else if (UI_PushUp_Counter_20 == 1)
		{
			drawCapBar(referee_data.UI_Graph1.Graphic, UI_Graph_Change);

			UI_PushUp_Graphs(1, &referee_data.UI_Graph1, referee_data.Game_Robot_State.robot_id);
		}
		else if(UI_PushUp_Counter_20 == 7)
		{
			//显示摩擦轮转速
			UI_Draw_Float(&referee_data.UI_Graph5.Graphic[0], "spd", UI_Graph_Change, LAYER3, UI_Color_Orange, 15, 2, 3, 1200, 40, infantry.friction_speed);
			UI_PushUp_Graphs(5, &referee_data.UI_Graph5, referee_data.Game_Robot_State.robot_id);
		}
		//更新频率为100毫秒---------------//
		else if(UI_PushUp_Counter_10 == 5)
		{
			if(gimbal_receiver_pack1.chassis_mode_action == CV_ROTATE)//小陀螺旋转
			{
				UI_Draw_String(&referee_data.UI_String.String, "rta", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 10, 3, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2 - 200, "CV_ROTA:ON");
			}
			else{
				UI_Draw_String(&referee_data.UI_String.String, "rta", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 10, 3, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2 - 200, "          ");
			}
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		else if(UI_PushUp_Counter_10 == 7)
		{
			if(gimbal_receiver_pack1.autoaim_id == 0)//
			{
				//不使用辅瞄
				UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "NO_AIM      ");
			}else if(gimbal_receiver_pack1.autoaim_id == 1)
			{
				//普通辅瞄
				UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "STANDARD AIM");
			}else if(gimbal_receiver_pack1.autoaim_id == 2)
			{
				//小符
				UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "SMALL BUFF  ");	
			}else if(gimbal_receiver_pack1.autoaim_id == 3)
			{
				//定点小陀螺
				UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "STATIC_AIM  ");
			}else if(gimbal_receiver_pack1.autoaim_id == 4)
			{
				//大符
				UI_Draw_String(&referee_data.UI_String.String, "aim", UI_Graph_Change, LAYER2, UI_Color_Orange, 20, 6, 3, 1280, 700, "BIG BUFF    ");
			}
			UI_PushUp_String(&referee_data.UI_String, referee_data.Game_Robot_State.robot_id);
		}
		
		else if (UI_PushUp_Counter_10 == 9)
		{
			UI_Draw_Rectangle(&referee_data.UI_Graph2.Graphic[0], "bdy", UI_Graph_Change, LAYER1, UI_Color_Green, 5, CHASSIS_POS_UI_START_X, CHASSIS_POS_UI_START_Y + CHASSIS_POS_WIDTH / 2, CHASSIS_POS_UI_START_X + CHASSIS_POS_LENGTH, CHASSIS_POS_UI_START_Y - CHASSIS_POS_WIDTH / 2);
			UI_Draw_Line(&referee_data.UI_Graph2.Graphic[1], "dir", UI_Graph_Change, LAYER1, UI_Color_Orange, 5, 1700, 750, (uint16_t)(CHASSIS_POS_UI_START_X + CHASSIS_POS_WIDTH / 2 - (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * UI_FRONT_SIN)), (uint16_t)(CHASSIS_POS_UI_START_Y + (int16_t)(GIM_CHASSIS_ANGLE_LINE_LEN * UI_FRONT_COS)));
			UI_PushUp_Graphs(2, &referee_data.UI_Graph2, referee_data.Game_Robot_State.robot_id);	
		}

//		xEventGroupSetBits(xCreatedEventGroup, REFEREE_TASK_BIT); 

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
	}
}
