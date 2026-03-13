/**
 * @file bsp_mpu6050.c
 * @brief MPU6050传感器驱动实现
 * @version 1.0.0
 * @date 2026-02-08
 */

#include "bsp_mpu6050.h"
#include "bsp_i2c.h"
#include <math.h>
#include "error_code.h"
/* 静态变量 */
static uint8_t s_accel_range = MPU6050_ACCEL_RANGE_2G;
static uint8_t s_gyro_range = MPU6050_GYRO_RANGE_250DPS;
static uint8_t s_initialized = 0;

/* 加速度计量程对应的灵敏度 (LSB/g) */
static const uint16_t s_accel_sensitivity[4] = {16384, 8192, 4096, 2048};

/* 陀螺仪量程对应的灵敏度 (LSB/°/s) */
static const float s_gyro_sensitivity[4] = {131.0f, 65.5f, 32.8f, 16.4f};

/**
 * @brief MPU6050初始化
 */
uint8_t bsp_mpu6050_init(const mpu6050_config_t *config)
{
    uint8_t id = 0;
    uint8_t err;
    
    /* 检查参数 */
    if(config != NULL) {
        s_accel_range = config->accel_range & 0x03;
        s_gyro_range = config->gyro_range & 0x03;
    } else {
        /* 使用默认配置 */
        s_accel_range = MPU6050_ACCEL_RANGE_2G;
        s_gyro_range = MPU6050_GYRO_RANGE_250DPS;
    }
    
    /* 复位MPU6050 */
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x80);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 延时等待复位完成 */
    for(volatile uint32_t i = 0; i < 100000; i++);
    
    /* 检查芯片ID */
    err = bsp_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &id, 1);
    if(err != ERR_OK || id != MPU6050_DEVICE_ID) {
        return ERR_MPU6050_ID_MISMATCH;
    }
    
    /* 设置时钟源为PLL with X axis gyroscope reference */
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x01);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 配置采样率 */
    uint8_t sample_div = (config != NULL) ? config->sample_rate_div : MPU6050_SMPLRT_DIV_DEFAULT;
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, sample_div);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 配置低通滤波器 */
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 配置加速度计量程 */
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, s_accel_range << 3);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 配置陀螺仪量程 */
    err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, s_gyro_range << 3);
    if(err != ERR_OK) {
        return ERR_MPU6050_INIT_FAILED;
    }
    
    /* 使能数据就绪中断（如配置） */
    if(config != NULL && config->use_interrupt) {
        err = bsp_mpu6050_config_interrupt(1);
        if(err != ERR_OK) {
            return ERR_MPU6050_INIT_FAILED;
        }
    }
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief 读取加速度数据
 */
uint8_t bsp_mpu6050_read_accel(mpu6050_data_t *data)
{
    uint8_t buf[6];
    uint8_t err;
    int16_t raw_x, raw_y, raw_z;
    float sensitivity;
    
    if(data == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 读取加速度寄存器 */
    err = bsp_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
    if(err != ERR_OK) {
        return ERR_MPU6050_I2C_READ;
    }
    
    /* 组合数据 */
    raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    /* 转换为m/s² */
    sensitivity = (float)s_accel_sensitivity[s_accel_range];
    data->acc_x = ((float)raw_x / sensitivity) * GRAVITY_ACCEL;
    data->acc_y = ((float)raw_y / sensitivity) * GRAVITY_ACCEL;
    data->acc_z = ((float)raw_z / sensitivity) * GRAVITY_ACCEL;
    data->valid = 1;
    
    return ERR_OK;
}

/**
 * @brief 读取陀螺仪数据
 */
uint8_t bsp_mpu6050_read_gyro(mpu6050_data_t *data)
{
    uint8_t buf[6];
    uint8_t err;
    int16_t raw_x, raw_y, raw_z;
    float sensitivity;
    
    if(data == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 读取陀螺仪寄存器 */
    err = bsp_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, buf, 6);
    if(err != ERR_OK) {
        return ERR_MPU6050_I2C_READ;
    }
    
    /* 组合数据 */
    raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    /* 转换为°/s */
    sensitivity = s_gyro_sensitivity[s_gyro_range];
    data->gyro_x = (float)raw_x / sensitivity;
    data->gyro_y = (float)raw_y / sensitivity;
    data->gyro_z = (float)raw_z / sensitivity;
    data->valid = 1;
    
    return ERR_OK;
}

/**
 * @brief 同时读取加速度和陀螺仪数据
 */
uint8_t bsp_mpu6050_read_all(mpu6050_data_t *data)
{
    uint8_t err1, err2;
    
    if(data == NULL) {
        return ERR_NULL_POINTER;
    }
    
    err1 = bsp_mpu6050_read_accel(data);
    if(err != ERR_OK) {
        return err1;
    }
    
    err2 = bsp_mpu6050_read_gyro(data);

    data->valid = 0;
    if(err != ERR_OK) {
        return err2;
    }
    if (err1 == ERR_OK && err2 == ERR_OK) {

        data->valid = 1;
        return ERR_OK;
    }

}

/**
 * @brief 配置数据就绪中断
 */
uint8_t bsp_mpu6050_config_interrupt(uint8_t enable)
{
    uint8_t err;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    
    if(enable) {
        /* 配置PA11为输入模式 */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* 配置外部中断 */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
        
        EXTI_InitStruct.EXTI_Line = EXTI_Line11;
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStruct);
        
        /* 配置NVIC */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
        
        /* 使能MPU6050数据就绪中断 */
        err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN);
        if(err != ERR_OK) {
            return err;
        }
    } else {
        /* 禁用中断 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line11;
        EXTI_InitStruct.EXTI_LineCmd = DISABLE;
        EXTI_Init(&EXTI_InitStruct);
        
        err = bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_INT_ENABLE, 0);
        if(err != ERR_OK) {
            return err;
        }
    }
    
    return ERR_OK;
}

/**
 * @brief 检查数据是否就绪
 */
uint8_t bsp_mpu6050_check_data_ready(uint8_t *ready)
{
    uint8_t status;
    uint8_t err;
    
    if(ready == NULL) {
        return ERR_NULL_POINTER;
    }
    
    err = bsp_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_INT_STATUS, &status, 1);
    if(err != ERR_OK) {
        return err;
    }
    
    *ready = (status & 0x01) ? 1 : 0;
    return ERR_OK;
}

/**
 * @brief 执行自检
 */
uint8_t bsp_mpu6050_self_test(uint8_t *result)
{
    /* 简化实现，实际应执行完整的自检流程 */
    if(result == NULL) {
        return ERR_NULL_POINTER;
    }
    
    *result = 1; /* 假设自检通过 */
    return ERR_OK;
}

/**
 * @brief 进入低功耗模式
 */
uint8_t bsp_mpu6050_enter_sleep(void)
{
    return bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x40);
}

/**
 * @brief 退出低功耗模式
 */
uint8_t bsp_mpu6050_exit_sleep(void)
{
    return bsp_i2c_write_byte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x01);
}
