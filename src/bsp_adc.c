/**
 * @file bsp_adc.c
 * @brief ADC电池监测模块实现
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 */

#include "bsp_adc.h"
#include <FreeRTOS.h>
#include <task.h>

/* 静态变量 */
static uint8_t s_initialized = 0;

/* 电池电压全局变量（注意拼写） */
float batteryVotlge = 0.0f;

/* 函数声明 */
static uint8_t adc_wait_conversion(uint32_t timeout_ms);

/**
 * @brief ADC初始化
 * @return 错误码
 */
uint8_t bsp_adc_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;
    
    if(s_initialized) {
        return ERR_OK;
    }
    
    /* 使能GPIOB和ADC1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);
    
    /* 配置PB1为模拟输入 */
    GPIO_InitStruct.GPIO_Pin = ADC_BAT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;  /* 模拟输入模式 */
    GPIO_Init(ADC_BAT_PORT, &GPIO_InitStruct);
    
    /* 配置ADC1 */
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;             /* 独立模式 */
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;                  /* 单通道模式 */
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;            /* 单次转换 */
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /* 软件触发 */
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;         /* 右对齐 */
    ADC_InitStruct.ADC_NbrOfChannel = 1;                        /* 1个通道 */
    ADC_Init(ADC1, &ADC_InitStruct);
    
    /* 配置ADC时钟预分频 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);  /* 72MHz/6 = 12MHz */
    
    /* 配置ADC通道 */
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    
    /* 使能ADC */
    ADC_Cmd(ADC1, ENABLE);
    
    /* ADC校准 */
    uint8_t err = bsp_adc_calibration();
    if(err != ERR_OK) {
        return err;
    }
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief 读取电池电压
 * @param voltage: 输出电压值指针
 * @return 错误码
 */
uint8_t bsp_adc_read_battery(float *voltage)
{
    uint16_t adc_value;
    uint8_t err;
    
    if(voltage == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 读取原始ADC值 */
    err = bsp_adc_read_raw(&adc_value);
    if(err != ERR_OK) {
        return err;
    }
    
    /* 转换为电压值 */
    /* V = ADC_Value * Vref / Resolution */
    *voltage = (float)adc_value * ADC_VREF / ADC_RESOLUTION;
    
    /* 赋值给batteryVotlge（注意拼写） */
    batteryVotlge = *voltage;
    
    return ERR_OK;
}

/**
 * @brief 读取电池电压（带滤波）
 * @param voltage: 输出电压值指针
 * @return 错误码
 */
uint8_t bsp_adc_get_battery_filtered(float *voltage)
{
    uint16_t samples[ADC_FILTER_SAMPLES];
    uint32_t sum = 0;
    uint16_t min = 0xFFFF, max = 0;
    uint8_t err;
    uint8_t i;
    
    if(voltage == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 采集多个样本 */
    for(i = 0; i < ADC_FILTER_SAMPLES; i++) {
        err = bsp_adc_read_raw(&samples[i]);
        if(err != ERR_OK) {
            return err;
        }
        
        /* 记录最大最小值 */
        if(samples[i] < min) min = samples[i];
        if(samples[i] > max) max = samples[i];
        
        sum += samples[i];
        
        /* 延时等待 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    /* 去掉最大最小值后取平均 */
    sum = sum - min - max;
    uint16_t avg = (uint16_t)(sum / (ADC_FILTER_SAMPLES - 2));
    
    /* 转换为电压 */
    *voltage = (float)avg * ADC_VREF / ADC_RESOLUTION;
    
    /* 更新batteryVotlge变量 */
    batteryVotlge = *voltage;
    
    return ERR_OK;
}

/**
 * @brief 启动ADC转换
 * @return 错误码
 */
uint8_t bsp_adc_start_conversion(void)
{
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 清除转换完成标志 */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    
    /* 启动转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    return ERR_OK;
}

/**
 * @brief 检查转换是否完成
 * @param ready: 输出完成状态指针，1表示完成
 * @return 错误码
 */
uint8_t bsp_adc_check_conversion(uint8_t *ready)
{
    if(ready == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 检查转换完成标志 */
    *ready = ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) ? 1 : 0;
    
    return ERR_OK;
}

/**
 * @brief 读取原始ADC值
 * @param adc_value: 输出ADC原始值
 * @return 错误码
 */
uint8_t bsp_adc_read_raw(uint16_t *adc_value)
{
    uint8_t err;
    
    if(adc_value == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 启动转换 */
    err = bsp_adc_start_conversion();
    if(err != ERR_OK) {
        return err;
    }
    
    /* 等待转换完成 */
    err = adc_wait_conversion(100);
    if(err != ERR_OK) {
        return ERR_ADC_CONVERSION_TIMEOUT;
    }
    
    /* 读取转换结果 */
    *adc_value = ADC_GetConversionValue(ADC1);
    
    return ERR_OK;
}

/**
 * @brief ADC校准
 * @return 错误码
 */
uint8_t bsp_adc_calibration(void)
{
    /* 复位校准 */
    ADC_ResetCalibration(ADC1);
    
    /* 等待复位完成 */
    uint32_t timeout = 10000;
    while(ADC_GetResetCalibrationStatus(ADC1)) {
        if(--timeout == 0) {
            return ERR_ADC_CALIBRATION_FAILED;
        }
    }
    
    /* 开始校准 */
    ADC_StartCalibration(ADC1);
    
    /* 等待校准完成 */
    timeout = 10000;
    while(ADC_GetCalibrationStatus(ADC1)) {
        if(--timeout == 0) {
            return ERR_ADC_CALIBRATION_FAILED;
        }
    }
    
    return ERR_OK;
}

/* ===================== 内部函数 ===================== */

/**
 * @brief 等待ADC转换完成
 * @param timeout_ms: 超时时间（毫秒）
 * @return 错误码
 */
static uint8_t adc_wait_conversion(uint32_t timeout_ms)
{
    uint32_t timeout = pdMS_TO_TICKS(timeout_ms);
    TickType_t start = xTaskGetTickCount();
    
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if(xTaskGetTickCount() - start > timeout) {
            return ERR_TIMEOUT;
        }
        taskYIELD();
    }
    
    return ERR_OK;
}
