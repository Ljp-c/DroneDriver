/**
 * @file bsp_i2c.h
 * @brief I2C总线驱动头文件
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 功能说明：
 * - 实现I2C2总线驱动（PB10=SCL, PB11=SDA）
 * - 支持MPU6050和BMP250共用总线
 * - 提供总线仲裁机制，避免通讯冲突
 * - 使用FreeRTOS信号量实现线程安全
 * 
 * 时钟配置依据：
 * - STM32F103C8T6 APB1总线时钟36MHz
 * - I2C标准模式100KHz，快速模式400KHz
 * - 根据I2C协议标准配置时序寄存器
 */

#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include <stdint.h>
#include "stm32f10x.h"
#include "error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 引脚定义 ===================== */
#define I2C_SCL_PIN         GPIO_Pin_10     /* PB10: I2C2_SCL */
#define I2C_SDA_PIN         GPIO_Pin_11     /* PB11: I2C2_SDA */
#define I2C_GPIO_PORT       GPIOB
#define I2C_PERIPH          I2C2

/* ===================== 设备地址定义 ===================== */
#define MPU6050_ADDR        0xD0            /* MPU6050 I2C地址 (0x68 << 1) */
#define BMP250_ADDR         0xEC            /* BMP250 I2C地址 (0x76 << 1) */

/* ===================== 时序参数 ===================== */
#define I2C_TIMEOUT_MS      100             /* I2C操作超时时间 */
#define I2C_RETRY_COUNT     3               /* 重试次数 */

/* ===================== 函数声明 ===================== */

/**
 * @brief I2C总线初始化
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_I2C_BUS_ERROR: I2C总线错误
 * 
 * 配置说明：
 * - 使用I2C2，APB1时钟36MHz
 * - 标准模式100KHz
 * - 开启ACK应答
 */
uint8_t bsp_i2c_init(void);

/**
 * @brief I2C读取寄存器
 * @param dev_addr: 设备地址（已左移1位）
 * @param reg_addr: 寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_I2C_BUS_BUSY: 总线忙
 * @retval ERR_I2C_NACK: 无应答
 * @retval ERR_TIMEOUT: 超时
 * 
 * 协议时序：
 * 1. 发送START条件
 * 2. 发送设备地址+写标志
 * 3. 发送寄存器地址
 * 4. 发送RESTART条件
 * 5. 发送设备地址+读标志
 * 6. 读取数据
 * 7. 发送STOP条件
 */
uint8_t bsp_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, 
                         uint8_t *buf, uint8_t len);

/**
 * @brief I2C写入寄存器
 * @param dev_addr: 设备地址（已左移1位）
 * @param reg_addr: 寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 写入长度
 * @return 错误码
 * @retval ERR_OK: 写入成功
 * @retval ERR_I2C_BUS_BUSY: 总线忙
 * @retval ERR_I2C_NACK: 无应答
 * @retval ERR_TIMEOUT: 超时
 * 
 * 协议时序：
 * 1. 发送START条件
 * 2. 发送设备地址+写标志
 * 3. 发送寄存器地址
 * 4. 发送数据
 * 5. 发送STOP条件
 */
uint8_t bsp_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, 
                          const uint8_t *buf, uint8_t len);

/**
 * @brief I2C单字节写入（便捷函数）
 * @param dev_addr: 设备地址
 * @param reg_addr: 寄存器地址
 * @param data: 写入数据
 * @return 错误码
 */
uint8_t bsp_i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief 检查I2C设备是否在线
 * @param dev_addr: 设备地址
 * @return 错误码
 * @retval ERR_OK: 设备在线
 * @retval ERR_I2C_NACK: 设备无应答
 */
uint8_t bsp_i2c_check_device(uint8_t dev_addr);

/**
 * @brief I2C总线复位
 * @return 错误码
 * @note 在总线死锁时调用
 */
uint8_t bsp_i2c_bus_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H */
