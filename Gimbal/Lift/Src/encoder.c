#include "encoder.h"
Encoder_t encoder; // 编码器数据结构
volatile uint32_t Encoder_UpdateCounter = 0U; // 外置编码器有效接收帧累计数

