/**
 * @file bsp_adc.h
 * @brief ADC电压监测模块头文件
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 硬件配置：
 * - ADC通道：ADC1_IN9 (PB1)
 * - 参考电压：3.3V
 * - 分辨率：12位 (0-4095)
 * 
 * 电压计算推导：
 * - ADC值范围：0-4095对应0-3.3V
 * - 电压计算公式：V = ADC_Value * 3.3 / 4095
 * - 如需测量更高电压，需使用分压电阻
 * 
 * 滤波处理：
 * - 采用滑动平均滤波，提高稳定性
 * - 采样次数：16次，去掉最大最小值后取平均
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include "stm32f10x.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 引脚定义 ===================== */
#define ADC_BAT_PIN         GPIO_Pin_1      /* PB1: ADC1_IN9 */
#define ADC_BAT_PORT        GPIOB
#define ADC_CHANNEL         ADC_Channel_9   /* ADC1通道9 */

/* ===================== 常量定义 ===================== */
#define ADC_RESOLUTION      4095.0f         /* 12位分辨率 */
#define ADC_VREF            3.3f            /* 参考电压3.3V */
#define ADC_FILTER_SAMPLES  16              /* 滤波采样次数 */

/* ===================== 函数声明 ===================== */

/**
 * @brief ADC初始化
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_ADC_INIT_FAILED: 初始化失败
 * 
 * 时钟配置依据：
 * - APB2时钟72MHz
 * - ADC预分频6，ADC时钟12MHz
 * - 采样时间239.5周期，确保高精度
 */
uint8_t bsp_adc_init(void);

/**
 * @brief 读取电池电压
 * @param voltage: 输出电压值（伏特）
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_NULL_POINTER: 空指针
 * @retval ERR_ADC_CONVERSION_TIMEOUT: 转换超时
 * 
 * 数据处理流程：
 * 1. 启动ADC转换
 * 2. 等待转换完成
 * 3. 读取ADC值
 * 4. 转换为电压值
 * 5. 赋值给batteryVotlge变量
 */
uint8_t bsp_adc_read_battery(float *voltage);

/**
 * @brief 获取电池电压（带滤波）
 * @param voltage: 输出电压值（伏特）
 * @return 错误码
 * 
 * 滤波算法：
 * - 采集16次样本
 * - 去掉最大和最小值
 * - 剩余样本取平均
 */
uint8_t bsp_adc_get_battery_filtered(float *voltage);

/**
 * @brief 启动ADC转换
 * @return 错误码
 */
uint8_t bsp_adc_start_conversion(void);

/**
 * @brief 检查转换是否完成
 * @param ready: 输出就绪状态，1表示完成
 * @return 错误码
 */
uint8_t bsp_adc_check_conversion(uint8_t *ready);

/**
 * @brief 读取原始ADC值
 * @param adc_value: 输出ADC原始值
 * @return 错误码
 */
uint8_t bsp_adc_read_raw(uint16_t *adc_value);

/**
 * @brief ADC校准
 * @return 错误码
 * @retval ERR_OK: 校准成功
 * @retval ERR_ADC_CALIBRATION_FAILED: 校准失败
 */
uint8_t bsp_adc_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
