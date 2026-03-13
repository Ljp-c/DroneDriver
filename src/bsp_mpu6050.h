/**
 * @file bsp_mpu6050.h
 * @brief MPU6050传感器驱动头文件
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 硬件配置：
 * - I2C接口：SCL=PB10, SDA=PB11 (I2C2)
 * - 中断引脚：PA11 (预留)
 * - 设备地址：0x68 (AD0接地)
 * 
 * 功能说明：
 * - 实现MPU6050初始化、IIC读写、中断配置
 * - 提供XYZ轴加速度数据采集接口
 * - 支持数据就绪中断，避免频繁查询
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include <stdint.h>
#include "stm32f10x.h"
#include "error_code.h"
#include "system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 寄存器定义 ===================== */
#define MPU6050_REG_SELF_TEST_X         0x0D
#define MPU6050_REG_SELF_TEST_Y         0x0E
#define MPU6050_REG_SELF_TEST_Z         0x0F
#define MPU6050_REG_SELF_TEST_A         0x10
#define MPU6050_REG_SMPLRT_DIV          0x19
#define MPU6050_REG_CONFIG              0x1A
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_FIFO_EN             0x23
#define MPU6050_REG_INT_PIN_CFG         0x37
#define MPU6050_REG_INT_ENABLE          0x38
#define MPU6050_REG_INT_STATUS          0x3A
#define MPU6050_REG_ACCEL_XOUT_H        0x3B
#define MPU6050_REG_ACCEL_XOUT_L        0x3C
#define MPU6050_REG_ACCEL_YOUT_H        0x3D
#define MPU6050_REG_ACCEL_YOUT_L        0x3E
#define MPU6050_REG_ACCEL_ZOUT_H        0x3F
#define MPU6050_REG_ACCEL_ZOUT_L        0x40
#define MPU6050_REG_TEMP_OUT_H          0x41
#define MPU6050_REG_TEMP_OUT_L          0x42
#define MPU6050_REG_GYRO_XOUT_H         0x43
#define MPU6050_REG_GYRO_XOUT_L         0x44
#define MPU6050_REG_GYRO_YOUT_H         0x45
#define MPU6050_REG_GYRO_YOUT_L         0x46
#define MPU6050_REG_GYRO_ZOUT_H         0x47
#define MPU6050_REG_GYRO_ZOUT_L         0x48
#define MPU6050_REG_USER_CTRL           0x6A
#define MPU6050_REG_PWR_MGMT_1          0x6B
#define MPU6050_REG_PWR_MGMT_2          0x6C
#define MPU6050_REG_WHO_AM_I            0x75

/* ===================== 常量定义 ===================== */
#define MPU6050_DEVICE_ID               0x68
#define MPU6050_ADDR                    0xD0    /* (0x68 << 1) */

/* 加速度计量程 */
#define MPU6050_ACCEL_RANGE_2G          0       /* ±2g */
#define MPU6050_ACCEL_RANGE_4G          1       /* ±4g */
#define MPU6050_ACCEL_RANGE_8G          2       /* ±8g */
#define MPU6050_ACCEL_RANGE_16G         3       /* ±16g */

/* 陀螺仪量程 */
#define MPU6050_GYRO_RANGE_250DPS       0       /* ±250°/s */
#define MPU6050_GYRO_RANGE_500DPS       1       /* ±500°/s */
#define MPU6050_GYRO_RANGE_1000DPS      2       /* ±1000°/s */
#define MPU6050_GYRO_RANGE_2000DPS      3       /* ±2000°/s */

/* 采样率分频值 (1kHz / (1 + div)) */
#define MPU6050_SMPLRT_DIV_DEFAULT      0       /* 1kHz采样率 */

/* 中断配置 */
#define MPU6050_INT_DATA_RDY_EN         0x01    /* 数据就绪中断使能 */

/* ===================== 数据结构 ===================== */
/**
 * @brief MPU6050配置结构体
 */
typedef struct {
    uint8_t accel_range;        /* 加速度计量程 */
    uint8_t gyro_range;         /* 陀螺仪量程 */
    uint8_t sample_rate_div;    /* 采样率分频 */
    uint8_t use_interrupt;      /* 是否使用中断 */
} mpu6050_config_t;

/* ===================== 函数声明 ===================== */

/**
 * @brief MPU6050初始化
 * @param config: 配置参数，NULL使用默认配置
 * @return 错误码
 * @retval ERR_OK: 初始化成功
 * @retval ERR_MPU6050_INIT_FAILED: 初始化失败
 * @retval ERR_MPU6050_ID_MISMATCH: 芯片ID不匹配
 * 
 * 初始化流程：
 * 1. 复位MPU6050
 * 2. 检查芯片ID
 * 3. 配置时钟源
 * 4. 配置加速度计和陀螺仪量程
 * 5. 配置采样率
 * 6. 使能数据就绪中断（如配置）
 */
uint8_t bsp_mpu6050_init(const mpu6050_config_t *config);

/**
 * @brief 读取加速度数据
 * @param data: 加速度数据结构体
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_MPU6050_I2C_READ: I2C读取失败
 * @retval ERR_NULL_POINTER: 空指针
 * 
 * 数据转换：
 * - 根据量程将原始数据转换为m/s²
 * - ±2g量程：16384 LSB/g
 * - ±4g量程：8192 LSB/g
 * - ±8g量程：4096 LSB/g
 * - ±16g量程：2048 LSB/g
 */
uint8_t bsp_mpu6050_read_accel(mpu6050_data_t *data);

/**
 * @brief 读取陀螺仪数据
 * @param data: 陀螺仪数据结构体
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_MPU6050_I2C_READ: I2C读取失败
 * @retval ERR_NULL_POINTER: 空指针
 */
uint8_t bsp_mpu6050_read_gyro(mpu6050_data_t *data);

/**
 * @brief 同时读取加速度和陀螺仪数据
 * @param data: 传感器数据结构体
 * @return 错误码
 * @retval ERR_OK: 读取成功
 * @retval ERR_MPU6050_I2C_READ: I2C读取失败
 * @retval ERR_NULL_POINTER: 空指针
 */
uint8_t bsp_mpu6050_read_all(mpu6050_data_t *data);

/**
 * @brief 配置数据就绪中断
 * @param enable: 1使能中断，0禁用中断
 * @return 错误码
 * @retval ERR_OK: 配置成功
 * 
 * 中断配置说明：
 * - 使用PA11引脚
 * - 下降沿触发
 * - 中断服务程序中读取数据
 */
uint8_t bsp_mpu6050_config_interrupt(uint8_t enable);

/**
 * @brief 检查数据是否就绪
 * @param ready: 输出就绪状态，1表示就绪
 * @return 错误码
 */
uint8_t bsp_mpu6050_check_data_ready(uint8_t *ready);

/**
 * @brief 执行自检
 * @param result: 输出自检结果，1表示通过
 * @return 错误码
 */
uint8_t bsp_mpu6050_self_test(uint8_t *result);

/**
 * @brief 进入低功耗模式
 * @return 错误码
 */
uint8_t bsp_mpu6050_enter_sleep(void);

/**
 * @brief 退出低功耗模式
 * @return 错误码
 */
uint8_t bsp_mpu6050_exit_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MPU6050_H */
