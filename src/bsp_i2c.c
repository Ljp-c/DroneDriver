/**
 * @file bsp_i2c.c
 * @brief I2C总线驱动实现
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 功能说明：
 * - 实现I2C2总线驱动（PB10=SCL, PB11=SDA）
 * - 支持MPU6050和BMP250共用总线
 * - 使用FreeRTOS信号量实现线程安全和总线仲裁
 * 
 * 时钟配置依据：
 * - STM32F103C8T6 APB1总线时钟36MHz
 * - I2C标准模式100KHz
 * - CCR = 36MHz / (2 * 100KHz) = 180 = 0xB4
 * - TRISE = (1000ns / 28ns) + 1 = 36
 */

#include "bsp_i2c.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

/* 静态变量 */
static SemaphoreHandle_t s_i2c_mutex = NULL;
static uint8_t s_initialized = 0;

/* 函数声明 */
static uint8_t i2c_wait_event(uint32_t event, uint32_t timeout_ms);
static uint8_t i2c_send_start(void);
static uint8_t i2c_send_stop(void);
static uint8_t i2c_send_addr(uint8_t addr, uint8_t direction);

/**
 * @brief I2C总线初始化
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_I2C_BUS_ERROR: I2C总线错误
 */
uint8_t bsp_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    
    if(s_initialized) {
        return ERR_OK;
    }
    
    /* 创建互斥信号量用于总线仲裁 */
    s_i2c_mutex = xSemaphoreCreateMutex();
    if(s_i2c_mutex == NULL) {
        return ERR_SEMAPHORE_CREATE_FAILED;
    }
    
    /* 使能GPIOB和I2C2时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    /* 配置PB10(SCL)和PB11(SDA)为复用开漏输出 */
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
    
    /* 配置I2C2 */
    I2C_InitStruct.I2C_ClockSpeed = 100000;         /* 100KHz标准模式 */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;         /* I2C模式 */
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; /* 占空比2:1 */
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;          /* 从机地址（主机模式不需要） */
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;        /* 使能应答 */
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C_PERIPH, &I2C_InitStruct);
    
    /* 使能I2C2 */
    I2C_Cmd(I2C_PERIPH, ENABLE);
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief I2C读取寄存器
 * @param dev_addr: 设备地址（已左移1位）
 * @param reg_addr: 寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 * @return 错误码
 */
uint8_t bsp_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, 
                         uint8_t *buf, uint8_t len)
{
    uint8_t err = ERR_OK;
    uint8_t i;
    
    if(buf == NULL || len == 0) {
        return ERR_INVALID_PARAM;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 获取总线互斥锁 */
    if(xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ERR_I2C_BUS_BUSY;
    }
    
    /* 发送START条件 */
    err = i2c_send_start();
    if(err != ERR_OK) {
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送设备地址+写标志 */
    err = i2c_send_addr(dev_addr, I2C_Direction_Transmitter);
    if(err != ERR_OK) {
        i2c_send_stop();
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送寄存器地址 */
    I2C_SendData(I2C_PERIPH, reg_addr);
    err = i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS);
    if(err != ERR_OK) {
        i2c_send_stop();
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送RESTART条件 */
    err = i2c_send_start();
    if(err != ERR_OK) {
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送设备地址+读标志 */
    err = i2c_send_addr(dev_addr, I2C_Direction_Receiver);
    if(err != ERR_OK) {
        i2c_send_stop();
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 读取数据 */
    for(i = 0; i < len; i++) {
        if(i == len - 1) {
            /* 最后一个字节，发送NACK */
            I2C_AcknowledgeConfig(I2C_PERIPH, DISABLE);
        }
        
        err = i2c_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_MS);
        if(err != ERR_OK) {
            i2c_send_stop();
            I2C_AcknowledgeConfig(I2C_PERIPH, ENABLE);
            xSemaphoreGive(s_i2c_mutex);
            return err;
        }
        
        buf[i] = I2C_ReceiveData(I2C_PERIPH);
    }
    
    /* 发送STOP条件 */
    i2c_send_stop();
    
    /* 重新使能ACK */
    I2C_AcknowledgeConfig(I2C_PERIPH, ENABLE);
    
    /* 释放总线互斥锁 */
    xSemaphoreGive(s_i2c_mutex);
    
    return ERR_OK;
}

/**
 * @brief I2C写入寄存器
 * @param dev_addr: 设备地址（已左移1位）
 * @param reg_addr: 寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 写入长度
 * @return 错误码
 */
uint8_t bsp_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, 
                          const uint8_t *buf, uint8_t len)
{
    uint8_t err = ERR_OK;
    uint8_t i;
    
    if(buf == NULL || len == 0) {
        return ERR_INVALID_PARAM;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 获取总线互斥锁 */
    if(xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ERR_I2C_BUS_BUSY;
    }
    
    /* 发送START条件 */
    err = i2c_send_start();
    if(err != ERR_OK) {
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送设备地址+写标志 */
    err = i2c_send_addr(dev_addr, I2C_Direction_Transmitter);
    if(err != ERR_OK) {
        i2c_send_stop();
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送寄存器地址 */
    I2C_SendData(I2C_PERIPH, reg_addr);
    err = i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS);
    if(err != ERR_OK) {
        i2c_send_stop();
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送数据 */
    for(i = 0; i < len; i++) {
        I2C_SendData(I2C_PERIPH, buf[i]);
        err = i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS);
        if(err != ERR_OK) {
            i2c_send_stop();
            xSemaphoreGive(s_i2c_mutex);
            return err;
        }
    }
    
    /* 发送STOP条件 */
    i2c_send_stop();
    
    /* 释放总线互斥锁 */
    xSemaphoreGive(s_i2c_mutex);
    
    return ERR_OK;
}

/**
 * @brief I2C单字节写入（便捷函数）
 * @param dev_addr: 设备地址
 * @param reg_addr: 寄存器地址
 * @param data: 写入数据
 * @return 错误码
 */
uint8_t bsp_i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return bsp_i2c_write_reg(dev_addr, reg_addr, &data, 1);
}

/**
 * @brief 检查I2C设备是否在线
 * @param dev_addr: 设备地址
 * @return 错误码
 */
uint8_t bsp_i2c_check_device(uint8_t dev_addr)
{
    uint8_t err;
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 获取总线互斥锁 */
    if(xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ERR_I2C_BUS_BUSY;
    }
    
    /* 发送START条件 */
    err = i2c_send_start();
    if(err != ERR_OK) {
        xSemaphoreGive(s_i2c_mutex);
        return err;
    }
    
    /* 发送设备地址 */
    err = i2c_send_addr(dev_addr, I2C_Direction_Transmitter);
    
    /* 发送STOP条件 */
    i2c_send_stop();
    
    /* 释放总线互斥锁 */
    xSemaphoreGive(s_i2c_mutex);
    
    return err;
}

/**
 * @brief I2C总线复位
 * @return 错误码
 */
uint8_t bsp_i2c_bus_reset(void)
{
    uint8_t i;
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 获取总线互斥锁 */
    if(xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return ERR_I2C_BUS_BUSY;
    }
    
    /* 禁用I2C */
    I2C_Cmd(I2C_PERIPH, DISABLE);
    
    /* 配置SCL和SDA为GPIO输出 */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
    
    /* 发送9个时钟脉冲，释放SDA */
    for(i = 0; i < 9; i++) {
        GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
        GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    /* 重新配置为复用开漏 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
    
    /* 重新使能I2C */
    I2C_Cmd(I2C_PERIPH, ENABLE);
    
    /* 释放总线互斥锁 */
    xSemaphoreGive(s_i2c_mutex);
    
    return ERR_OK;
}

/* ===================== 内部函数 ===================== */

/**
 * @brief 等待I2C事件
 * @param event: 等待的事件
 * @param timeout_ms: 超时时间（毫秒）
 * @return 错误码
 */
static uint8_t i2c_wait_event(uint32_t event, uint32_t timeout_ms)
{
    uint32_t timeout = pdMS_TO_TICKS(timeout_ms);
    TickType_t start = xTaskGetTickCount();
    
    while(I2C_CheckEvent(I2C_PERIPH, event) == ERROR) {
        if(xTaskGetTickCount() - start > timeout) {
            return ERR_TIMEOUT;
        }
        taskYIELD();
    }
    
    return ERR_OK;
}

/**
 * @brief 发送START条件
 * @return 错误码
 */
static uint8_t i2c_send_start(void)
{
    /* 发送START条件 */
    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    
    /* 等待START发送完成 */
    return i2c_wait_event(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS);
}

/**
 * @brief 发送STOP条件
 * @return 错误码
 */
static uint8_t i2c_send_stop(void)
{
    /* 发送STOP条件 */
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
    
    /* 等待STOP发送完成 */
    uint32_t timeout = 1000;
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF) == RESET) {
        if(--timeout == 0) {
            return ERR_TIMEOUT;
        }
    }
    
    return ERR_OK;
}

/**
 * @brief 发送设备地址
 * @param addr: 设备地址
 * @param direction: 传输方向
 * @return 错误码
 */
static uint8_t i2c_send_addr(uint8_t addr, uint8_t direction)
{
    /* 发送设备地址和方向 */
    I2C_Send7bitAddress(I2C_PERIPH, addr, direction);
    
    /* 等待地址发送完成 */
    if(direction == I2C_Direction_Transmitter) {
        return i2c_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_MS);
    } else {
        return i2c_wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_MS);
    }
}
