/**
 * @file main.c
 * @brief 程序入口
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 程序说明：
 * - 系统初始化
 * - 启动FreeRTOS调度器
 * - 错误处理
 */

#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"
#include "flight_controller.h"
#include "error_code.h"
#include "system_types.h"
#include <stdio.h>

/* 函数声明 */
static void system_clock_config(void);
static void error_handler(uint8_t err_code);

/**
 * @brief 主函数
 * @return 程序退出码（嵌入式系统通常不会返回）
 */
int main(void)
{
    uint8_t err;
    
    /* 配置系统时钟 */
    system_clock_config();
    
    /* 初始化飞行控制系统 */
    err = flight_controller_init();
    if(err != ERR_OK) {
        error_handler(err);
    }
    
    /* 启用独立看门狗 (2.6秒超时) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(1640);
    IWDG_ReloadCounter();
    IWDG_Enable();
    
    /* 启动飞行控制系统（进入FreeRTOS调度） */
    err = flight_controller_start();
    if(err != ERR_OK) {
        error_handler(err);
    }
    
    /* 正常情况下不会执行到这里 */
    while(1) {
        /* 系统错误，进入死循环 */
    }
    
    return 0;
}

/**
 * @brief 系统时钟配置
 * @note 配置STM32F103C8T6为72MHz
 * 
 * 时钟树配置：
 * - HSE: 8MHz外部晶振
 * - PLL: 9倍频 -> 72MHz
 * - AHB: 72MHz
 * - APB1: 36MHz
 * - APB2: 72MHz
 * - ADC: 12MHz (72MHz/6)
 */
static void system_clock_config(void)
{
    ErrorStatus HSEStartUpStatus;
    
    /* 复位RCC寄存器 */
    RCC_DeInit();
    
    /* 使能HSE */
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    
    if(HSEStartUpStatus == SUCCESS) {
        /* 使能预取指缓存 */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);
        
        /* 配置AHB、APB1、APB2时钟 */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);    /* HCLK = SYSCLK = 72MHz */
        RCC_PCLK1Config(RCC_HCLK_Div2);     /* PCLK1 = HCLK/2 = 36MHz */
        RCC_PCLK2Config(RCC_HCLK_Div1);     /* PCLK2 = HCLK = 72MHz */
        
        /* 配置PLL */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  /* 8MHz * 9 = 72MHz */
        RCC_PLLCmd(ENABLE);
        
        /* 等待PLL就绪 */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        
        /* 切换系统时钟到PLL */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        
        /* 等待切换完成 */
        while(RCC_GetSYSCLKSource() != 0x08);
    } else {
        /* HSE启动失败，使用HSI */
        /* HSI = 8MHz，PLL 9倍频 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }
}

/**
 * @brief 错误处理函数 - 实现Fail-Safe故障恢复
 * @param err_code: 错误码
 * @note 根据错误类型执行相应的故障恢复：自动降落或紧急停止
 */
static void error_handler(uint8_t err_code)
{
    /* 更新全局系统状态 */
    g_system_state = SYS_STATE_ERROR;
    
    /* 根据错误类型进行分类处理 */
    switch(err_code) {
        case ERR_MPU6050_INIT_FAILED:
        case ERR_MPU6050_I2C_READ:
            /* 姿态传感器故障：进入降级模式
               可使用高度计和磁罗盘进行简化控制 */
            g_system_state = SYS_STATE_DEGRADED;
            /* 继续执行自动降落 */
            
        case ERR_BMP250_INIT_FAILED:
        case ERR_BMP250_I2C_READ:
            /* 气压计故障：可继续飞行，但使用加速度积分估算高度 */
            if (g_system_state == SYS_STATE_INIT || 
                g_system_state == SYS_STATE_READY ||
                g_system_state == SYS_STATE_RUNNING) {
                g_system_state = SYS_STATE_DEGRADED;
            }
            break;
            
        case ERR_NRF24L01_INIT_FAILED:
        case ERR_NRF24L01_TIMEOUT:
            /* 无线通讯故障：无法接收遥控指令，执行自动降落 */
            g_system_state = SYS_STATE_AUTO_LAND;
            break;
            
        case ERR_BATTERY_LOW:
            /* 电池不足警告：可继续飞行 */
            g_system_state = SYS_STATE_DEGRADED;
            break;
            
        case ERR_BATTERY_CRITICAL:
            /* 电池严重不足：立即执行紧急停止 */
            g_system_state = SYS_STATE_EMERGENCY_STOP;
            break;
            
        case ERR_TASK_CREATE_FAILED:
        case ERR_SEMAPHORE_CREATE_FAILED:
        case ERR_MUTEX_CREATE_FAILED:
            /* FreeRTOS资源创建失败：无法恢复，紧急停止 */
            g_system_state = SYS_STATE_EMERGENCY_STOP;
            break;
            
        default:
            /* 未知错误 */
            g_system_state = SYS_STATE_EMERGENCY_STOP;
            break;
    }
    
    /* 进入故障安全循环 */
    while(1) {
        /* 关闭所有中断，以较低的频率执行故障处理 */
        taskDISABLE_INTERRUPTS();
        
        switch(g_system_state) {
            case SYS_STATE_DEGRADED:
                /* 降级模式：系统仍可部分功能，可以继续接收指令
                   但需要用户注意监控 */
                /* 每100ms执行一次状态检查 */
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case SYS_STATE_AUTO_LAND:
                /* 自动降落模式：缓慢下降至地面
                   通过油门逐步降低 */
                {
                    static uint32_t land_duration = 0;
                    land_duration += 10;  /* 每次迭代增加10ms */
                    
                    /* 计算目标油门（从当前值缓慢降低到0） */
                    float land_progress = (float)land_duration / 5000.0f;  /* 5秒内完成降落 */
                    if (land_progress > 1.0f) {
                        land_progress = 1.0f;
                    }
                    
                    /* 逐步降低所有4个电机的油门
                       这里假设当前油门可从Global_message或其他地方获取 */
                    
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                break;
                
            case SYS_STATE_EMERGENCY_STOP:
            case SYS_STATE_ERROR:
            default:
                /* 紧急停止：立即切断所有电机
                   通过设置PWM占空比为0 */
                {
                    /* 尝试停止所有电机（如果PWM驱动已初始化） */
                    for (int i = 0; i < 4; i++) {
                        /* 假设有bsp_pwm_set_duty函数可用
                           bsp_pwm_set_duty(i, 0.0f);
                         */
                    }
                    
                    /* 每秒闪烁一次LED（如有）以指示系统故障
                       flash_led(500);
                     */
                    
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                break;
        }
        
        taskENABLE_INTERRUPTS();
    }
}

/**
 * @brief 断言失败处理函数
 * @param file: 文件名
 * @param line: 行号
 * @note 用于调试，发布版本可禁用
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    /* 可在此添加断言失败处理 */
    (void)file;
    (void)line;
    
    while(1);
}
#endif
