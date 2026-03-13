/**
 * @file bsp_pwm.c
 * @brief PWM输出模块实现
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 */

#include "bsp_pwm.h"

/* 静态变量 */
static uint8_t s_initialized = 0;
static float s_current_duty[PWM_CHANNEL_MAX] = {0.0f};

/* 函数声明 */
static uint8_t pwm_set_compare(pwm_channel_t channel, uint16_t compare_value);

/**
 * @brief PWM初始化
 * @return 错误码
 */
uint8_t bsp_pwm_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    if(s_initialized) {
        return ERR_OK;
    }
    
    /* 使能GPIOA和TIM2时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* 配置PA0-PA3为复用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = PWM_CH1_PIN | PWM_CH2_PIN | PWM_CH3_PIN | PWM_CH4_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStruct);
    
    /* 配置TIM2时基 */
    /* 时钟72MHz，预分频71 -> 1MHz计数频率 */
    /* 周期1000 -> 1kHz PWM频率 */
    TIM_TimeBaseStruct.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStruct.TIM_Prescaler = 71;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM_PERIPH, &TIM_TimeBaseStruct);
    
    /* 配置PWM模式 */
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;  /* 初始占空比0 */
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    
    /* 初始化四个通道 */
    TIM_OC1Init(PWM_TIM_PERIPH, &TIM_OCInitStruct);
    TIM_OC2Init(PWM_TIM_PERIPH, &TIM_OCInitStruct);
    TIM_OC3Init(PWM_TIM_PERIPH, &TIM_OCInitStruct);
    TIM_OC4Init(PWM_TIM_PERIPH, &TIM_OCInitStruct);
    
    /* 使能预装载寄存器，确保同步更新 */
    TIM_OC1PreloadConfig(PWM_TIM_PERIPH, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(PWM_TIM_PERIPH, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(PWM_TIM_PERIPH, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(PWM_TIM_PERIPH, TIM_OCPreload_Enable);
    
    /* 使能ARR预装载 */
    TIM_ARRPreloadConfig(PWM_TIM_PERIPH, ENABLE);
    
    /* 使能TIM2 */
    TIM_Cmd(PWM_TIM_PERIPH, ENABLE);
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief 设置PWM占空比
 * @param channel: PWM通道
 * @param duty: 占空比（0-100%）
 * @return 错误码
 */
uint8_t bsp_pwm_set_duty(pwm_channel_t channel, float duty)
{
    uint16_t compare_value;
    
    if(channel >= PWM_CHANNEL_MAX) {
        return ERR_PWM_CHANNEL_INVALID;
    }
    
    if(duty < PWM_MIN_DUTY || duty > PWM_MAX_DUTY) {
        return ERR_PWM_DUTY_OUT_OF_RANGE;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 计算比较值 */
    /* compare = duty / 100 * period */
    compare_value = (uint16_t)(duty * PWM_PERIOD / 100.0f);
    
    /* 设置比较值 */
    uint8_t err = pwm_set_compare(channel, compare_value);
    if(err != ERR_OK) {
        return err;
    }
    
    /* 保存当前占空比 */
    s_current_duty[channel] = duty;
    
    return ERR_OK;
}

/**
 * @brief 设置PWM电压输出
 * @param channel: PWM通道
 * @param voltage: 电压值（0-5V）
 * @return 错误码
 */
uint8_t bsp_pwm_set_voltage(pwm_channel_t channel, float voltage)
{
    float duty;
    
    if(voltage < 0.0f || voltage > PWM_VOLTAGE_MAX) {
        return ERR_OUT_OF_RANGE;
    }
    
    /* 电压转换为占空比 */
    /* duty = voltage / 5V * 100% */
    duty = voltage / PWM_VOLTAGE_MAX * 100.0f;
    
    return bsp_pwm_set_duty(channel, duty);
}

/**
 * @brief 同时设置四通道PWM占空比
 * @param duty_ch1: 通道1占空比
 * @param duty_ch2: 通道2占空比
 * @param duty_ch3: 通道3占空比
 * @param duty_ch4: 通道4占空比
 * @return 错误码
 */
uint8_t bsp_pwm_set_all_duty(float duty_ch1, float duty_ch2, 
                             float duty_ch3, float duty_ch4)
{
    uint16_t compare1, compare2, compare3, compare4;
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 检查占空比范围 */
    if(duty_ch1 < PWM_MIN_DUTY || duty_ch1 > PWM_MAX_DUTY ||
       duty_ch2 < PWM_MIN_DUTY || duty_ch2 > PWM_MAX_DUTY ||
       duty_ch3 < PWM_MIN_DUTY || duty_ch3 > PWM_MAX_DUTY ||
       duty_ch4 < PWM_MIN_DUTY || duty_ch4 > PWM_MAX_DUTY) {
        return ERR_PWM_DUTY_OUT_OF_RANGE;
    }
    
    /* 计算比较值 */
    compare1 = (uint16_t)(duty_ch1 * PWM_PERIOD / 100.0f);
    compare2 = (uint16_t)(duty_ch2 * PWM_PERIOD / 100.0f);
    compare3 = (uint16_t)(duty_ch3 * PWM_PERIOD / 100.0f);
    compare4 = (uint16_t)(duty_ch4 * PWM_PERIOD / 100.0f);
    
    /* 禁用TIM2以同步更新 */
    TIM_Cmd(PWM_TIM_PERIPH, DISABLE);
    
    /* 设置四个通道的比较值 */
    TIM_SetCompare1(PWM_TIM_PERIPH, compare1);
    TIM_SetCompare2(PWM_TIM_PERIPH, compare2);
    TIM_SetCompare3(PWM_TIM_PERIPH, compare3);
    TIM_SetCompare4(PWM_TIM_PERIPH, compare4);
    
    /* 重新使能TIM2 */
    TIM_Cmd(PWM_TIM_PERIPH, ENABLE);
    
    /* 保存当前占空比 */
    s_current_duty[0] = duty_ch1;
    s_current_duty[1] = duty_ch2;
    s_current_duty[2] = duty_ch3;
    s_current_duty[3] = duty_ch4;
    
    return ERR_OK;
}

/**
 * @brief 获取当前占空比
 * @param channel: PWM通道
 * @param duty: 输出占空比
 * @return 错误码
 */
uint8_t bsp_pwm_get_duty(pwm_channel_t channel, float *duty)
{
    if(channel >= PWM_CHANNEL_MAX) {
        return ERR_PWM_CHANNEL_INVALID;
    }
    
    if(duty == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    *duty = s_current_duty[channel];
    
    return ERR_OK;
}

/**
 * @brief 启动PWM输出
 * @param channel: PWM通道，PWM_CHANNEL_MAX表示全部通道
 * @return 错误码
 */
uint8_t bsp_pwm_start(pwm_channel_t channel)
{
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if(channel >= PWM_CHANNEL_MAX) {
        /* 启动全部通道 */
        TIM_Cmd(PWM_TIM_PERIPH, ENABLE);
    } else {
        /* 单个通道启动已在初始化时完成 */
        /* 这里可以添加单独通道使能逻辑 */
    }
    
    return ERR_OK;
}

/**
 * @brief 停止PWM输出
 * @param channel: PWM通道，PWM_CHANNEL_MAX表示全部通道
 * @return 错误码
 */
uint8_t bsp_pwm_stop(pwm_channel_t channel)
{
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if(channel >= PWM_CHANNEL_MAX) {
        /* 停止全部通道 */
        TIM_Cmd(PWM_TIM_PERIPH, DISABLE);
    } else {
        /* 设置单个通道占空比为0 */
        pwm_set_compare(channel, 0);
        s_current_duty[channel] = 0.0f;
    }
    
    return ERR_OK;
}

/**
 * @brief 紧急停止所有PWM输出
 * @return 错误码
 */
uint8_t bsp_pwm_emergency_stop(void)
{
    uint8_t i;
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 立即禁用TIM2 */
    TIM_Cmd(PWM_TIM_PERIPH, DISABLE);
    
    /* 清零所有比较值 */
    TIM_SetCompare1(PWM_TIM_PERIPH, 0);
    TIM_SetCompare2(PWM_TIM_PERIPH, 0);
    TIM_SetCompare3(PWM_TIM_PERIPH, 0);
    TIM_SetCompare4(PWM_TIM_PERIPH, 0);
    
    /* 清零占空比记录 */
    for(i = 0; i < PWM_CHANNEL_MAX; i++) {
        s_current_duty[i] = 0.0f;
    }
    
    return ERR_OK;
}

/* ===================== 内部函数 ===================== */

/**
 * @brief 设置PWM比较值
 * @param channel: PWM通道
 * @param compare_value: 比较值
 * @return 错误码
 */
static uint8_t pwm_set_compare(pwm_channel_t channel, uint16_t compare_value)
{
    switch(channel) {
        case PWM_CHANNEL_1:
            TIM_SetCompare1(PWM_TIM_PERIPH, compare_value);
            break;
        case PWM_CHANNEL_2:
            TIM_SetCompare2(PWM_TIM_PERIPH, compare_value);
            break;
        case PWM_CHANNEL_3:
            TIM_SetCompare3(PWM_TIM_PERIPH, compare_value);
            break;
        case PWM_CHANNEL_4:
            TIM_SetCompare4(PWM_TIM_PERIPH, compare_value);
            break;
        default:
            return ERR_PWM_CHANNEL_INVALID;
    }
    
    return ERR_OK;
}
