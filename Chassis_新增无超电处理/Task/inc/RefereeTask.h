#ifndef __REFEREE_TASK_H
#define __REFEREE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "math.h"

#include "stdint.h"
#include "GimbalReceive.h"
//#include "SuperPower.h"

#include "bsp_referee.h"
#include "Referee.h"

#include "algorithmOfCRC.h"
//#include "remote_control.h"

//#include "ins.h"
#include "ChassisController.h"
#include "PowerControlTask.h"

//#include "iwdg.h"

#define IMAGE_CENTER_X 1920 / 2 // ͼ��X����
#define IMAGE_CENTER_Y 1080 / 2 // ͼ��Y����

#define CAP_BAR_LENGTH 250 // ����UI����
#define CAP_BAR_WIDTH 20   // ����UI�߶�

#define CAP_BAR_UI_START_X IMAGE_CENTER_X - CAP_BAR_LENGTH / 2 + CAP_BAR_WIDTH / 2 // ��ʼ����X
#define CAP_BAR_UI_START_Y 20                                                      // ��ʼ����Y

#define CHASSIS_POS_LENGTH 100 // ����UI����
#define CHASSIS_POS_WIDTH 100   // ����UI�߶�

#define CHASSIS_POS_UI_START_X 1650 // ��ʼ����X
#define CHASSIS_POS_UI_START_Y 700 // ��ʼ����Y       

#define GIM_CHASSIS_ANGLE_LINE_LEN 150
#define LAYER1 1 //graph
#define LAYER2 2 //string
#define LAYER3 3 //other

//��ͬ�����̸���ģʽ��ͬ��һЩ���й̶�ͷ�ķ��򣬼���һ��ƫ������UI����̨����
#if ROBOT == NIUNIU
#define UI_FRONT_BIAS 8160
#elif ROBOT == CHEN_JING_YUAN
#define UI_FRONT_BIAS 0
#elif ROBOT == NIU_MO_SON
#define UI_FRONT_BIAS 0
#elif ROBOT == QI_TIAN_DA_SHENG
#define UI_FRONT_BIAS 0
#endif

extern float UI_FRONT_ERR,UI_FRONT_SIN,UI_FRONT_COS;
extern uint8_t Radar_double_hurt_chance;

void Refereetask(void const * argument);

#endif
