/**
 * @file bsp_pwm.h
 * @brief PWM输出模块头文件
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 硬件配置：
 * - PWM通道：PA0-PA3 (TIM2_CH1-CH4)
 * - 定时器：TIM2
 * - 频率：1kHz
 * 
 * 占空比与电压对应关系推导：
 * - PWM频率1kHz，周期1000us
 * - 占空比0% -> 输出0V
 * - 占空比100% -> 输出5V
 * - 电压 = 占空比(%) * 5V / 100
 * 
 * 时钟配置依据：
 * - APB1时钟72MHz
 * - TIM2时钟72MHz（APB1无预分频）
 * - 预分频71，计数频率1MHz
 * - 周期1000，对应1kHz频率
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include <stdint.h>
#include "stm32f10x.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 引脚定义 ===================== */
#define PWM_CH1_PIN         GPIO_Pin_0      /* PA0: TIM2_CH1 */
#define PWM_CH2_PIN         GPIO_Pin_1      /* PA1: TIM2_CH2 */
#define PWM_CH3_PIN         GPIO_Pin_2      /* PA2: TIM2_CH3 */
#define PWM_CH4_PIN         GPIO_Pin_3      /* PA3: TIM2_CH4 */
#define PWM_GPIO_PORT       GPIOA
#define PWM_TIM_PERIPH      TIM2

/* ===================== PWM参数 ===================== */
#define PWM_FREQUENCY       1000            /* PWM频率1kHz */
#define PWM_PERIOD          1000            /* 周期值（预分频后1MHz时钟） */
#define PWM_MIN_DUTY        0               /* 最小占空比 */
#define PWM_MAX_DUTY        100             /* 最大占空比 */
#define PWM_VOLTAGE_MAX     5.0f            /* 最大输出电压5V */

/* ===================== 通道枚举 ===================== */
typedef enum {
    PWM_CHANNEL_1 = 0,      /* PA0 */
    PWM_CHANNEL_2,          /* PA1 */
    PWM_CHANNEL_3,          /* PA2 */
    PWM_CHANNEL_4,          /* PA3 */
    PWM_CHANNEL_MAX
} pwm_channel_t;
/* ===================== PWM输出 ===================== */
typedef struct
{
    float pwmout_accel[3];
    float pwmout_gyro[3];

}PWM_Output_t;



/* ===================== 函数声明 ===================== */

/**
 * @brief PWM初始化
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_PWM_INIT_FAILED: 初始化失败
 * 
 * 配置说明：
 * - TIM2时钟72MHz
 * - 预分频71，计数频率1MHz
 * - 周期1000，频率1kHz
 * - 四通道同步输出
 */
uint8_t bsp_pwm_init(void);

/**
 * @brief 设置PWM占空比
 * @param channel: PWM通道
 * @param duty: 占空比（0-100%）
 * @return 错误码
 * @retval ERR_OK: 设置成功
 * @retval ERR_PWM_CHANNEL_INVALID: 通道无效
 * @retval ERR_PWM_DUTY_OUT_OF_RANGE: 占空比超出范围
 * 
 * 占空比与电压关系：
 * - duty = 0%  -> 0V
 * - duty = 50% -> 2.5V
 * - duty = 100% -> 5V
 */
uint8_t bsp_pwm_set_duty(pwm_channel_t channel, float duty);

/**
 * @brief 设置PWM电压输出
 * @param channel: PWM通道
 * @param voltage: 电压值（0-5V）
 * @return 错误码
 * 
 * 电压转换公式：
 * duty = voltage / 5V * 100%
 */
uint8_t bsp_pwm_set_voltage(pwm_channel_t channel, float voltage);

/**
 * @brief 同时设置四通道PWM占空比
 * @param duty_ch1: 通道1占空比
 * @param duty_ch2: 通道2占空比
 * @param duty_ch3: 通道3占空比
 * @param duty_ch4: 通道4占空比
 * @return 错误码
 * 
 * 说明：
 * - 四通道同时更新，确保同步
 * - 使用预装载寄存器，避免输出抖动
 */
uint8_t bsp_pwm_set_all_duty(float duty_ch1, float duty_ch2, 
                             float duty_ch3, float duty_ch4);

/**
 * @brief 获取当前占空比
 * @param channel: PWM通道
 * @param duty: 输出占空比
 * @return 错误码
 */
uint8_t bsp_pwm_get_duty(pwm_channel_t channel, float *duty);

/**
 * @brief 启动PWM输出
 * @param channel: PWM通道，PWM_CHANNEL_MAX表示全部通道
 * @return 错误码
 */
uint8_t bsp_pwm_start(pwm_channel_t channel);

/**
 * @brief 停止PWM输出
 * @param channel: PWM通道，PWM_CHANNEL_MAX表示全部通道
 * @return 错误码
 */
uint8_t bsp_pwm_stop(pwm_channel_t channel);

/**
 * @brief 紧急停止所有PWM输出
 * @return 错误码
 * @note 占空比归零，用于紧急停机
 */
uint8_t bsp_pwm_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
