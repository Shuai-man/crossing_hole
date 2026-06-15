#ifndef _TOF_H
#define _TOF_H

#include "main.h"
#include "usart.h"
#include <string.h>
#include "debug.h"

#define TOF_DEVICE_ADDR 0x0001U  // 默认从机地址

// -------------------------- 寄存器地址定义 --------------------------
// 注：寄存器读写属性已标注，使用时需遵循手册限制
typedef enum {
    TOF_REG_SPECIAL         = 0x0001U,  // 特殊寄存器（只写）
    TOF_REG_DEV_ADDR        = 0x0002U,  // 设备地址寄存器（可读可写，0为广播地址）
    TOF_REG_BAUD_RATE       = 0x0003U,  // 波特率寄存器（可读可写）
    TOF_REG_RANGE           = 0x0004U,  // 量程寄存器（可读可写）
    TOF_REG_CONTINUOUS_CTRL = 0x0005U,  // 连续输出控制寄存器（可读可写）
    TOF_REG_LOAD_CALIB      = 0x0006U,  // 加载校准寄存器（可读可写）
    TOF_REG_OFFSET_CORRECT  = 0x0007U,  // 偏移修正值寄存器（可读可写）
    TOF_REG_XTALK_CORRECT   = 0x0008U,  // xtalk修正值寄存器（可读可写）
    TOF_REG_DISABLE_IIC     = 0x0009U,  // 禁止IIC使能寄存器（可读可写）
    TOF_REG_MEASURE_RESULT  = 0x0010U,  // 测量结果寄存器（只读，单位：mm）
    TOF_REG_OFFSET_CALIB    = 0x0020U,  // offset校准寄存器（只写，推荐值：50mm=0x0032U）
    TOF_REG_XTALK_CALIB     = 0x0021U,   // xtalk校准寄存器（只写）
} TOF_RegWriteTypeDef;

// -------------------------- 功能指令定义 --------------------------
typedef enum {
    TOF_CMD_WRITE = 0x0006U,  // 写寄存器
    TOF_CMD_READ  = 0x0003U   // 读寄存器
} TOF_CmdTypeDef;


// -------------------------- 特殊寄存器指令定义 --------------------------
typedef enum {
    TOF_RESETORE = 0xAA55U,  // 恢复默认参数
    TOF_RESET   = 0x1000U,  // 重启设备
    TOF_TEST_COMM = 0x0000U  // 测试通讯
} TOF_SpecialCmdTypeDef;
// -------------------------- 波特率配置选项 --------------------------
typedef enum {
    TOF_BAUD_38400  = 0x0001U,  // 波特率38400
    TOF_BAUD_9600   = 0x0002U,  // 波特率9600
    TOF_BAUD_115200 = 0x0003U   // 波特率115200（手册说明：其他值默认115200）
} TOF_BaudRateTypeDef;

// -------------------------- 量程配置选项 --------------------------
typedef enum {
    TOF_RANGE_HIGH_PRECISION = 0x0001U,  // 高精度模式：100ms周期，0~0.2m量程
    TOF_RANGE_MEDIUM          = 0x0002U,  // 中距离模式：100ms周期，0~0.4m量程
    TOF_RANGE_LONG            = 0x0003U   // 长距离模式：100ms周期，0~0.5m量程
} TOF_RangeTypeDef;

// -------------------------- 加载校准配置选项 --------------------------
typedef enum {
    TOF_LOAD_CALIB_DISABLE = 0x0000U,  // 不加载校准参数
    TOF_LOAD_CALIB_ENABLE   = 0x0001U   // 加载校准参数
} TOF_LoadCalibTypeDef;

// -------------------------- 禁止IIC配置选项 --------------------------
typedef enum {
    TOF_IIC_ENABLE  = 0x0000U,  // 不禁止IIC（默认状态）
    TOF_IIC_DISABLE = 0x0001U   // 禁止IIC，MCU释放IO口控制权
} TOF_DisableIICTypeDef;

// -------------------------- 连续输出控制选项 --------------------------
typedef enum {
    TOF_CONTINUOUS_OUTPUT_DISABLE = 0x0000U,  // 关闭自动输出，需主动读取数据
    TOF_CONTINUOUS_OUTPUT_ENABLE  = 0x0001U   // 启用自动输出，输出间隔，单位为ms（如0x03E8U表示1000ms间隔自动输出）
} TOF_ContinuousOutputTypeDef;

typedef struct Tof_SendDataTypeDef {
  uint8_t deviceAddr;  // 从机地址
  uint8_t cmd;         // 功能
  uint8_t regAddr_H;     // 寄存器地址高8位
  uint8_t regAddr_L;     // 寄存器地址低8位
  uint8_t data_H;        // 数据高8位
  uint8_t data_L;        // 数据低8位
  uint16_t crc;         // CRC校验码
}Tof_SendDataTypeDef;

#pragma pack(1)
typedef struct Tof_ReceiveDataTypeDef {
  uint8_t deviceAddr;  // 从机地址
  uint8_t cmd;         // 功能
  uint8_t data_len;      // 数据长度
  uint16_t distance;        // 数据内容（如测量结果，单位：mm）
  uint16_t crc;         // CRC校验码
}Tof_ReceiveDataTypeDef;
#pragma pack()


#define TOF_FRAME_SIZE sizeof(Tof_SendDataTypeDef)
#define TOF_RECEIVE_FRAME_SIZE sizeof(Tof_ReceiveDataTypeDef)
	
extern Tof_ReceiveDataTypeDef Tof_ReceiveData;

void TOF_BaudRate_Init(void);
void TOF_Reset(void);
void TOF_Init(void);

void TOF_Receive(void);

uint16_t crc16_calculate_modbus(uint8_t *p_data, uint16_t len);


#endif
