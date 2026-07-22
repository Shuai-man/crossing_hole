#ifndef NING_CAP_H
#define NING_CAP_H

#include <stdint.h>

#define CAP_CAPACITY 4.54f /* 超级电容组等效电容量，单位 F */
#define CAP_MIN_VOL 20.0f  /* 超电停止输出的电压，也是可用能量计算下限 */
#define CAP_MAX_VOL 28.0f  /* 超电充满电压，也是可用能量计算上限 */

#define CAP_VOL_HIGH 26.0f /* 电压高状态阈值，用于状态显示 */
#define CAP_VOL_MID 23.0f  /* 电压中状态阈值，用于状态切换迟滞 */
#define CAP_VOL_LOW 20.0f  /* 电压低状态阈值，低于此值不可使用 */
#define CAP_USE_ENABLE_VOL 22.0f /* 电压恢复到此值以上才重新允许使用超电 */
#define CAP_USE_DISABLE_VOL 20.0f /* 电压低于此值立即禁止使用超电 */

#pragma pack(push, 1)
typedef struct
{
    uint16_t cap_vol;       /* 0.01 V */
    int16_t chassis_power;  /* 0.01 W，超电输出到底盘轮电机侧功率 */
    uint16_t referee_power; /* 0.01 W，裁判系统输入到超电侧功率 */
    uint16_t reverse;
} SuperCapRecvData;

typedef struct
{
    uint16_t buffer_energy;
    uint16_t P_ref;          /* 0.01 W */
    uint16_t reverse;
    uint16_t wireless_start;
} SuperCapSendData;
#pragma pack(pop)

typedef enum
{
    CapVol_Low = 0,
    CapVol_Middle,
    CapVol_High
} CapVolState;

typedef struct
{
    float cap_vol;
    float cap_energy;
    float cap_energy_pecent;
    float cap_energy_max;
    float cap_energy_min;
    float chassis_power;  /* 超电输出到底盘轮电机侧功率，单位 W */
    float referee_power;  /* 裁判系统输入到超电侧功率，单位 W */
    uint32_t power_measurement_sequence; /* 超电每接收一帧功率数据递增 */
    CapVolState cap_vol_state;
    uint8_t energy_available;
    uint8_t power_data_valid; /* 超电在线且测量数据有效 */
} NingCapController;

extern SuperCapSendData cap_send_data;
extern SuperCapRecvData cap_recv_data;
extern NingCapController cap_controller;

void CapControllerInit(void);
void ReceiveCapDecode(const uint8_t *recv_data, SuperCapRecvData *decoded_data);

/* 仅把裁判系统当前功率上限发给超电，不发送本地扩展后的底盘上限。 */
void SendCapPack(SuperCapSendData *send_data, float referee_power_limit);

/* online 必须来自 debug.c 的超电掉线检测。掉线时立即判定为无可用能量。 */
void NingCapUpdateState(uint8_t online);
uint8_t NingCapHasEnergy(void);

#endif
