#include "debug.h"

GlobalDebugger global_debugger;

float Ozone[8]; //使用Ozone显示的变量

void LossUpdate(Loss_Debugger *loss_debugger, float thresh_t)
{
    loss_debugger->cnt_dt = DWT_GetDeltaT(&loss_debugger->current_cnt);
    if (loss_debugger->cnt_dt > thresh_t)
    {
			loss_debugger->loss_num++;
			loss_debugger->state = TIME_OUT;
    }
		else
		{ 
			loss_debugger->recv_msgs_num++;
			loss_debugger->state = ON;
		}
}

void LossDetect(Loss_Debugger *loss_debugger)
{
	if(loss_debugger->last_cnt==loss_debugger->current_cnt)
	{
		loss_debugger->state=OFF;
	}
	else
	{
		loss_debugger->last_cnt=loss_debugger->current_cnt;
	}
}
