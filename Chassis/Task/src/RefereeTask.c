#include "RefereeTask.h"

int Rest_UI_Flag;
bool ref_ready_flag; // 当前线程初始化完成标志

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
	Ref_Init();
	while (1)
	{
		static int UI_PushUp_Counter = 0;
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);

		if (UI_PushUp_Counter < 1000)
		{
			if (UI_PushUp_Counter % 11 == 0)
				Sightglass_static_show();
			if (UI_PushUp_Counter % 13 == 0)
				Sightglass1_static_show();
			if (UI_PushUp_Counter % 7 == 0)
				Sightglass2_static_show();
			if (UI_PushUp_Counter % 17 == 0)
				Show_FIRE_static();
			if (UI_PushUp_Counter % 19 == 0)
				Show_BUMP_static();
			if (UI_PushUp_Counter % 31 == 0)
				Show_SPIN_static();
			if (UI_PushUp_Counter % 23 == 0)
				Show_FALL_static();
			if (UI_PushUp_Counter % 29 == 0)
				Show_CHANGE_static();
			if (UI_PushUp_Counter % 37 == 0)
				Show_ZERO_static();
		}
		else
		{
			if (UI_PushUp_Counter % 5 == 0)
				Sightglass_flash_show();
			if (UI_PushUp_Counter % 13 == 0)
				Sightglass1_flash_show();
		}

		if (UI_PushUp_Counter > 30000)
			UI_PushUp_Counter = 0;

		UI_PushUp_Counter++;
		osDelay(10);
	}
}

void Sightglass_static_show(void)
{
	UI_Draw_Rectangle(&UI_Graph7.Graphic[0], "11", UI_Graph_Add, 1, UI_Color_Green, 6, 1490, 754, 1639, 810);
	UI_Draw_Rectangle(&UI_Graph7.Graphic[1], "12", UI_Graph_Add, 1, UI_Color_Green, 6, 1490, 654, 1639, 710);
	UI_Draw_Rectangle(&UI_Graph7.Graphic[2], "13", UI_Graph_Add, 1, UI_Color_Green, 6, 1490, 554, 1639, 610);
	UI_Draw_Rectangle(&UI_Graph7.Graphic[3], "14", UI_Graph_Add, 1, UI_Color_Green, 6, 1490, 454, 1639, 510);
	UI_Draw_Rectangle(&UI_Graph7.Graphic[4], "15", UI_Graph_Add, 1, UI_Color_Green, 6, 1490, 354, 1711, 410);

	UI_Draw_Arc(&UI_Graph7.Graphic[5], "21", UI_Graph_Add, 1, UI_Color_Orange, 330, 30, 5, 960, 540, 100, 100);

	UI_Draw_Rectangle(&UI_Graph7.Graphic[6], "22", UI_Graph_Add, 1, UI_Color_Green, 30, 750, 800, 1170, 800);

	UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
}

void Sightglass1_static_show(void)
{
	UI_Draw_Int(&UI_Graph7.Graphic[0], "16", UI_Graph_Add, 0, UI_Color_Green, 24, 6, 640, 700, 99);
	UI_Draw_Int(&UI_Graph7.Graphic[1], "17", UI_Graph_Add, 0, UI_Color_Green, 24, 6, 640, 660, 99);
	UI_Draw_Int(&UI_Graph7.Graphic[6], "25", UI_Graph_Add, 0, UI_Color_Green, 24, 6, 640, 620, 99);
	UI_Draw_Float(&UI_Graph7.Graphic[2], "18", UI_Graph_Add, 0, UI_Color_Green, 24, 1, 6, 1210, 700, 99.9);
	UI_Draw_Float(&UI_Graph7.Graphic[3], "19", UI_Graph_Add, 0, UI_Color_Green, 24, 1, 6, 1210, 660, 99.9);
	UI_Draw_Float(&UI_Graph7.Graphic[4], "20", UI_Graph_Add, 0, UI_Color_Green, 24, 1, 6, 1210, 620, 99.9);

	UI_Draw_Rectangle(&UI_Graph7.Graphic[5], "24", UI_Graph_Add, 1, UI_Color_Pink, 6, 660 + 90, 340, 1260 - 90, 740);

	UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
}

void Sightglass2_static_show(void)
{
	UI_Draw_Circle(&UI_Graph1.Graphic[0], "23", UI_Graph_Add, 0, UI_Color_Green, 2, 960, 430, 34);
	UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
}

void Show_ZERO_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "000", UI_Graph_Add, 2, UI_Color_Cyan, 100, 4, 5, 50, 750, "ZERO");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Show_FALL_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "001", UI_Graph_Add, 2, UI_Color_Pink, 36, 4, 5, 1500, 800, "FALL");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Show_FIRE_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "002", UI_Graph_Add, 2, UI_Color_Pink, 36, 4, 5, 1500, 700, "FIRE");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Show_BUMP_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "003", UI_Graph_Add, 2, UI_Color_Pink, 36, 4, 5, 1500, 600, "BUMP");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Show_SPIN_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "004", UI_Graph_Add, 2, UI_Color_Pink, 36, 4, 5, 1500, 500, "SPIN");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Show_CHANGE_static(void)
{
	memset(UI_String.String.stringdata, ' ', 30);
	UI_Draw_String(&UI_String.String, "005", UI_Graph_Add, 2, UI_Color_Pink, 36, 6, 5, 1500, 400, "CHANGE");
	UI_PushUp_String(&UI_String, Robot_ID_Current);
}

void Sightglass_flash_show(void)
{
	//	static int flow_angle = 0;
	//	static int v_cap = 0;

	//	if(Flag.fall_flag) UI_Draw_Rectangle(&UI_Graph7.Graphic[0], "11", UI_Graph_Change, 1, UI_Color_Green, 6 , 1490 , 754 , 1639 , 810);
	//	else UI_Draw_Rectangle(&UI_Graph7.Graphic[0], "11", UI_Graph_Change, 1, UI_Color_Green, 0 , 1490 , 754 , 1639 , 810);
	//	if(Up_Cboard_Info.Friction_Status) UI_Draw_Rectangle(&UI_Graph7.Graphic[1], "12", UI_Graph_Change, 1, UI_Color_Green, 6 , 1490 , 654 , 1639 , 710);
	//	else UI_Draw_Rectangle(&UI_Graph7.Graphic[1], "12", UI_Graph_Change, 1, UI_Color_Green, 0 , 1490 , 654 , 1639 , 710);
	//	if(Flag.bump_flag) UI_Draw_Rectangle(&UI_Graph7.Graphic[2], "13", UI_Graph_Change, 1, UI_Color_Green, 6 , 1490 , 554 , 1639 , 610);
	//	else UI_Draw_Rectangle(&UI_Graph7.Graphic[2], "13", UI_Graph_Change, 1, UI_Color_Green, 0 , 1490 , 554 , 1639 , 610);
	//	if(Flag.spinning_flag) UI_Draw_Rectangle(&UI_Graph7.Graphic[3], "14", UI_Graph_Change, 1, UI_Color_Green, 6 , 1490 , 454 , 1639 , 510);
	//	else UI_Draw_Rectangle(&UI_Graph7.Graphic[3], "14", UI_Graph_Change, 1, UI_Color_Green, 0 , 1490 , 454 , 1639 , 510);
	//	if(Flag.change_flag) UI_Draw_Rectangle(&UI_Graph7.Graphic[4], "15", UI_Graph_Change, 1, UI_Color_Green, 6 , 1490 , 354 , 1711 , 410);
	//	else UI_Draw_Rectangle(&UI_Graph7.Graphic[4], "15", UI_Graph_Change, 1, UI_Color_Green, 0 , 1490 , 354 , 1711 , 410);
	//
	//	flow_angle = (int)(Find_Min_RADIAN(Body.abs_yaw,ZERO_HEAD_YAW)*57.2957805f);
	//	if(flow_angle<0) flow_angle = (180+flow_angle)+180;
	//	flow_angle = 360 - flow_angle;
	//	UI_Draw_Arc(&UI_Graph7.Graphic[5],"21",UI_Graph_Change,1,UI_Color_Orange,Whole_Circle_ANGLE(flow_angle-30),Whole_Circle_ANGLE(flow_angle+30),5,960,540,100,100);
	//
	//	v_cap = pm_od.v_out - 1500;
	//
	//	if(v_cap<=0) v_cap = 0;
	//
	//	if(v_cap>=100)
	//	{
	//		UI_Draw_Rectangle(&UI_Graph7.Graphic[6], "22", UI_Graph_Change, 1, UI_Color_Green, 30 , 750 , 800 , 1170-(int)(420*((550 - v_cap)/550.0f)) , 800);
	//	}
	//	else
	//	{
	//		UI_Draw_Rectangle(&UI_Graph7.Graphic[6], "22", UI_Graph_Change, 1, UI_Color_Yellow, 30 , 750 , 800 , 1170-(int)(420*((550 - v_cap)/550.0f)) , 800);
	//	}
	//
	//
	//	UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
}

void Sightglass1_flash_show(void)
{
	//	UI_Draw_Int  (&UI_Graph7.Graphic[0], "16", UI_Graph_Change, 0, UI_Color_Green, 24 , 6  , 640 , 700 , Up_Cboard_Info.Friction_Speed);
	//	UI_Draw_Int  (&UI_Graph7.Graphic[1], "17", UI_Graph_Change, 0, UI_Color_Green, 24 , 6  , 640 , 660 , Up_Cboard_Info.Number_Of_Bullets);
	//
	//
	//	if     (Remote_Select == DT7   ) UI_Draw_Int  (&UI_Graph7.Graphic[6], "25", UI_Graph_Change, 0, UI_Color_Green, 24 , 6  , 640 , 620 , 1);
	//	else if(Remote_Select == VT_03 ) UI_Draw_Int  (&UI_Graph7.Graphic[6], "25", UI_Graph_Change, 0, UI_Color_Green, 24 , 6  , 640 , 620 , 2);
	//	else if(Remote_Select == UNLINK) UI_Draw_Int  (&UI_Graph7.Graphic[6], "25", UI_Graph_Change, 0, UI_Color_Green, 24 , 6  , 640 , 620 , 0);
	//
	//	UI_Draw_Float(&UI_Graph7.Graphic[2], "18", UI_Graph_Change, 0, UI_Color_Green, 24 , 1  , 6    , 1210 , 700 , Goal_Setting.Target_L0);
	//	UI_Draw_Float(&UI_Graph7.Graphic[3], "19", UI_Graph_Change, 0, UI_Color_Green, 24 , 1  , 6    , 1210 , 660 , Goal_Setting.MAX_Dx);
	//	UI_Draw_Float(&UI_Graph7.Graphic[4], "20", UI_Graph_Change, 0, UI_Color_Green, 24 , 1  , 6    , 1210 , 620 , Goal_Setting.MAX_Dyaw);
	//
	//	if(Up_Cboard_Info.Auto_Aim_Flag)
	//	{
	//		UI_Draw_Rectangle(&UI_Graph7.Graphic[5], "24", UI_Graph_Change, 1, UI_Color_Green, 6 , 660+90 , 340 , 1260-90 , 740);
	//	}
	//	else
	//	{
	//		UI_Draw_Rectangle(&UI_Graph7.Graphic[5], "24", UI_Graph_Change, 1,  UI_Color_Pink, 6 , 660+90 , 340 , 1260-90 , 740);
	//	}
	//
	//	UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
}
