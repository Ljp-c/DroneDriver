/**
 * @file bsp_bmp250.h
 * @brief BMP250气压传感器驱动头文件
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 硬件配置：
 * - I2C接口：SCL=PB10, SDA=PB11 (I2C2，与MPU6050共用)
 * - 设备地址：0x76 (SDO接地)
 * 
 * 功能说明：
 * - 实现BMP250初始化、IIC读写
 * - 提供高度数据采集与校准功能
 * - 支持温度补偿
 */

#ifndef __BSP_BMP250_H
#define __BSP_BMP250_H

#include <stdint.h>
#include "stm32f10x.h"
#include "error_code.h"
#include "system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 寄存器定义 ===================== */
#define BMP250_REG_ID                   0xD0
#define BMP250_REG_RESET                0xE0
#define BMP250_REG_STATUS               0xF3
#define BMP250_REG_CTRL_MEAS            0xF4
#define BMP250_REG_CONFIG               0xF5
#define BMP250_REG_PRESS_MSB            0xF7
#define BMP250_REG_PRESS_LSB            0xF8
#define BMP250_REG_PRESS_XLSB           0xF9
#define BMP250_REG_TEMP_MSB             0xFA
#define BMP250_REG_TEMP_LSB             0xFB
#define BMP250_REG_TEMP_XLSB            0xFC

/* 校准参数寄存器 */
#define BMP250_REG_CALIB_START          0x88
#define BMP250_REG_CALIB_LEN            24

/* ===================== 常量定义 ===================== */
#define BMP250_DEVICE_ID                0x58
#define BMP250_ADDR                     0xEC    /* (0x76 << 1) */
#define BMP250_RESET_CMD                0xB6

/* 过采样设置 */
#define BMP250_OVERSAMPLING_SKIP        0
#define BMP250_OVERSAMPLING_X1          1
#define BMP250_OVERSAMPLING_X2          2
#define BMP250_OVERSAMPLING_X4          3
#define BMP250_OVERSAMPLING_X8          4
#define BMP250_OVERSAMPLING_X16         5

/* 工作模式 */
#define BMP250_MODE_SLEEP               0
#define BMP250_MODE_FORCED              1
#define BMP250_MODE_NORMAL              3

/* IIR滤波系数 */
#define BMP250_FILTER_OFF               0
#define BMP250_FILTER_X2                1
#define BMP250_FILTER_X4                2
#define BMP250_FILTER_X8                3
#define BMP250_FILTER_X16               4

/* 待机时间 */
#define BMP250_STANDBY_0_5MS            0
#define BMP250_STANDBY_62_5MS           1
#define BMP250_STANDBY_125MS            2
#define BMP250_STANDBY_250MS            3
#define BMP250_STANDBY_500MS            4
#define BMP250_STANDBY_1000MS           5
#define BMP250_STANDBY_2000MS           6
#define BMP250_STANDBY_4000MS           7

/* ===================== 数据结构 ===================== */
/**
 * @brief BMP250校准参数结构体
 */
typedef struct {
    uint16_t dig_t1;
    int16_t  dig_t2;
    int16_t  dig_t3;
    uint16_t dig_p1;
    int16_t  dig_p2;
    int16_t  dig_p3;
    int16_t  dig_p4;
    int16_t  dig_p5;
    int16_t  dig_p6;
    int16_t  dig_p7;
    int16_t  dig_p8;
    int16_t  dig_p9;
} bmp250_calib_t;

/**
 * @brief BMP250配置结构体
 */
typedef struct {
    uint8_t temp_oversampling;      /* 温度过采样 */
    uint8_t press_oversampling;     /* 压力过采样 */
    uint8_t work_mode;              /* 工作模式 */
    uint8_t filter_coeff;           /* IIR滤波系数 */
    uint8_t standby_time;           /* 待机时间 */
} bmp250_config_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief BMP250初始化
 * @param config: 配置参数，NULL使用默认配置
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_BMP250_INIT_FAILED: 初始化失败
 * @retval ERR_BMP250_ID_MISMATCH: 芯片ID不匹配
 * 
 * 初始化流程：
 * 1. 软件复位
 * 2. 检查芯片ID
 * 3. 读取校准参数
 * 4. 配置工作模式
 * 5. 配置IIR滤波
 */
uint8_t bsp_bmp250_init(const bmp250_config_t *config);

/**
 * @brief 读取高度数据
 * @param data: 高度数据结构体
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_BMP250_I2C_READ: I2C读取失败
 * @retval ERR_NULL_POINTER: 空指针
 * 
 * 数据处理流程：
 * 1. 读取原始温度数据
 * 2. 读取原始压力数据
 * 3. 使用校准参数进行温度补偿
 * 4. 使用校准参数进行压力补偿
 * 5. 将压力转换为高度
 * 
 * 高度计算公式：
 * h = 44330 * (1 - (P/P0)^(1/5.255))
 * 其中P0 = 1013.25 hPa
 */
uint8_t bsp_bmp250_read_altitude(bmp250_data_t *data);

/**
 * @brief 读取原始温度数据（ADC值）
 * @param adc_temp: 输出原始温度ADC值
 * @return 错误码
 */
uint8_t bsp_bmp250_read_raw_temp(int32_t *adc_temp);

/**
 * @brief 读取原始压力数据（ADC值）
 * @param adc_press: 输出原始压力ADC值
 * @return 错误码
 */
uint8_t bsp_bmp250_read_raw_press(int32_t *adc_press);

/**
 * @brief 温度补偿计算
 * @param adc_temp: 原始温度ADC值
 * @param t_fine: 输出精细温度值（用于压力补偿）
 * @return 补偿后的温度值（摄氏度）
 * 
 * 补偿公式来自Bosch官方数据手册
 */
float bsp_bmp250_compensate_temp(int32_t adc_temp, int32_t *t_fine);

/**
 * @brief 压力补偿计算
 * @param adc_press: 原始压力ADC值
 * @param t_fine: 精细温度值
 * @return 补偿后的压力值（帕斯卡）
 * 
 * 补偿公式来自Bosch官方数据手册
 */
float bsp_bmp250_compensate_press(int32_t adc_press, int32_t t_fine);

/**
 * @brief 压力转换为高度
 * @param pressure_pa: 压力值（帕斯卡）
 * @param sea_level_pressure: 海平面参考压力（帕斯卡），0使用默认值
 * @return 高度值（米）
 * 
 * 公式推导：
 * 基于国际标准大气模型（ISA）
 * h = 44330 * [1 - (P/P0)^(1/5.255)]
 */
float bsp_bmp250_press_to_altitude(float pressure_pa, float sea_level_pressure);

/**
 * @brief 强制进行一次测量
 * @return 错误码
 * @note 在强制模式下使用
 */
uint8_t bsp_bmp250_trigger_measurement(void);

/**
 * @brief 检查测量是否完成
 * @param ready: 输出就绪状态，1表示完成
 * @return 错误码
 */
uint8_t bsp_bmp250_check_measurement_ready(uint8_t *ready);

/**
 * @brief 进入低功耗模式
 * @return 错误码
 */
uint8_t bsp_bmp250_enter_sleep(void);

/**
 * @brief 软件复位
 * @return 错误码
 */
uint8_t bsp_bmp250_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BMP250_H */
