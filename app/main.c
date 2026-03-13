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
 * @brief 错误处理函数
 * @param err_code: 错误码
 * @note 根据错误码执行相应的错误处理
 */
static void error_handler(uint8_t err_code)
{
    /* 关闭所有中断 */
    __disable_irq();
    
    /* 根据错误码执行不同处理 */
    switch(err_code) {
        case ERR_MPU6050_INIT_FAILED:
        case ERR_MPU6050_ID_MISMATCH:
            /* MPU6050初始化失败，可能是硬件连接问题 */
            break;
            
        case ERR_BMP250_INIT_FAILED:
        case ERR_BMP250_ID_MISMATCH:
            /* BMP250初始化失败 */
            break;
            
        case ERR_NRF24L01_INIT_FAILED:
            /* 通讯模块初始化失败 */
            break;
            
        case ERR_TASK_CREATE_FAILED:
        case ERR_SEMAPHORE_CREATE_FAILED:
            /* FreeRTOS资源创建失败 */
            break;
            
        default:
            /* 其他错误 */
            break;
    }
    
    /* 进入死循环，等待看门狗复位或人工干预 */
    while(1) {
        /* 可在此添加LED闪烁等错误指示 */
        for(volatile uint32_t i = 0; i < 500000; i++);
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
