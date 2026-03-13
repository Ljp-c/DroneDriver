/**
 * @file error_code.h
 * @brief 全局错误码定义
 * @details 统一错误码系统,覆盖所有硬件模块和应用层
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 类型定义 - 错误码枚举
 *============================================================================*/
typedef enum {
    /* 通用错误码 (0x00-0x0F) */
    ERR_SUCCESS = 0,                    /**< 成功 */
    ERR_OK = 0,                         /**< 成功(别名) */
    ERR_NULL_POINTER = 0x01,            /**< 空指针错误 */
    ERR_INVALID_PARAM = 0x02,           /**< 无效参数 */
    ERR_TIMEOUT = 0x03,                 /**< 超时错误 */
    ERR_BUSY = 0x04,                    /**< 设备忙 */
    ERR_NOT_INITIALIZED = 0x05,         /**< 未初始化 */
    ERR_OUT_OF_RANGE = 0x06,            /**< 超出范围 */
    ERR_MEMORY_ALLOC = 0x07,            /**< 内存分配失败 */
    ERR_INVALID_STATE = 0x08,           /**< 无效状态 */
    
    /* MPU6050错误码 (0x10-0x1F) */
    ERR_MPU6050_INIT_FAILED = 0x10,     /**< MPU6050初始化失败 */
    ERR_MPU6050_ID_MISMATCH = 0x11,     /**< MPU6050芯片ID不匹配 */
    ERR_MPU6050_I2C_READ = 0x12,        /**< MPU6050 I2C读取失败 */
    ERR_MPU6050_I2C_WRITE = 0x13,       /**< MPU6050 I2C写入失败 */
    ERR_MPU6050_DATA_NOT_READY = 0x14,  /**< MPU6050数据未就绪 */
    ERR_MPU6050_SELF_TEST_FAILED = 0x15,/**< MPU6050自检失败 */
    
    /* BMP250错误码 (0x20-0x2F) */
    ERR_BMP250_INIT_FAILED = 0x20,      /**< BMP250初始化失败 */
    ERR_BMP250_ID_MISMATCH = 0x21,      /**< BMP250芯片ID不匹配 */
    ERR_BMP250_I2C_READ = 0x22,         /**< BMP250 I2C读取失败 */
    ERR_BMP250_I2C_WRITE = 0x23,        /**< BMP250 I2C写入失败 */
    ERR_BMP250_CALIB_READ = 0x24,       /**< BMP250校准参数读取失败 */
    ERR_BMP250_MEASURING = 0x25,        /**< BMP250正在测量中 */
    
    /* ADC错误码 (0x30-0x3F) */
    ERR_ADC_INIT_FAILED = 0x30,         /**< ADC初始化失败 */
    ERR_ADC_CHANNEL_INVALID = 0x31,     /**< ADC通道无效 */
    ERR_ADC_CONVERSION_TIMEOUT = 0x32,  /**< ADC转换超时 */
    ERR_ADC_CALIBRATION_FAILED = 0x33,  /**< ADC校准失败 */
    
    /* W25Q64错误码 (0x40-0x4F) */
    ERR_W25Q64_INIT_FAILED = 0x40,      /**< W25Q64初始化失败 */
    ERR_W25Q64_ID_MISMATCH = 0x41,      /**< W25Q64芯片ID不匹配 */
    ERR_W25Q64_SPI_READ = 0x42,         /**< W25Q64 SPI读取失败 */
    ERR_W25Q64_SPI_WRITE = 0x43,        /**< W25Q64 SPI写入失败 */
    ERR_W25Q64_ERASE_FAILED = 0x44,     /**< W25Q64擦除失败 */
    ERR_W25Q64_WRITE_PROTECTED = 0x45,  /**< W25Q64写保护 */
    ERR_W25Q64_ADDRESS_INVALID = 0x46,  /**< W25Q64地址无效 */
    ERR_FLASH_TIMEOUT = 0x47,           /**< Flash操作超时 */
    ERR_FLASH_WRITE_ENABLE_FAILED = 0x48, /**< Flash写使能失败 */
    ERR_FLASH_ID_MISMATCH = 0x49,       /**< Flash ID不匹配 */
    ERR_FLASH_CRC_ERROR = 0x4A,         /**< Flash CRC错误 */
    
    /* PWM错误码 (0x50-0x5F) */
    ERR_PWM_INIT_FAILED = 0x50,         /**< PWM初始化失败 */
    ERR_PWM_CHANNEL_INVALID = 0x51,     /**< PWM通道无效 */
    ERR_PWM_DUTY_OUT_OF_RANGE = 0x52,   /**< PWM占空比超出范围 */
    ERR_PWM_FREQUENCY_INVALID = 0x53,   /**< PWM频率无效 */
    
    /* NRF24L01错误码 (0x60-0x6F) */
    ERR_NRF24L01_INIT_FAILED = 0x60,    /**< NRF24L01初始化失败 */
    ERR_NRF24L01_SPI_READ = 0x61,       /**< NRF24L01 SPI读取失败 */
    ERR_NRF24L01_SPI_WRITE = 0x62,      /**< NRF24L01 SPI写入失败 */
    ERR_NRF24L01_TX_FAILED = 0x63,      /**< NRF24L01发送失败 */
    ERR_NRF24L01_RX_FAILED = 0x64,      /**< NRF24L01接收失败 */
    ERR_NRF24L01_IRQ_TIMEOUT = 0x65,    /**< NRF24L01中断超时 */
    ERR_NRF24L01_DATA_INVALID = 0x66,   /**< NRF24L01数据无效 */
    ERR_NRF24L01_MAX_RT = 0x67,         /**< NRF24L01达到最大重发次数 */
    ERR_NRF24L01_TIMEOUT = 0x68,        /**< NRF24L01超时 */
    ERR_INVALID_PACKET = 0x69,          /**< 无效数据包 */
    ERR_CRC_ERROR = 0x6A,               /**< CRC校验错误 */
    
    /* PID错误码 (0x70-0x7F) */
    ERR_PID_INIT_FAILED = 0x70,         /**< PID初始化失败 */
    ERR_PID_PARAM_INVALID = 0x71,       /**< PID参数无效 */
    ERR_PID_OUTPUT_SATURATION = 0x72,   /**< PID输出饱和 */
    
    /* 总线错误码 (0x80-0x8F) */
    ERR_I2C_BUS_BUSY = 0x80,            /**< I2C总线忙 */
    ERR_I2C_BUS_ERROR = 0x81,           /**< I2C总线错误 */
    ERR_I2C_NACK = 0x82,                /**< I2C无应答 */
    ERR_SPI_BUS_BUSY = 0x83,            /**< SPI总线忙 */
    ERR_SPI_BUS_ERROR = 0x84,           /**< SPI总线错误 */
    ERR_SPI_INIT_FAILED = 0x85,         /**< SPI初始化失败 */
    ERR_MUTEX_CREATE_FAILED = 0x86,     /**< 互斥锁创建失败 */
    
    /* 飞行控制错误码 (0x90-0x9F) */
    ERR_NOT_ARMED = 0x90,               /**< 未解锁 */
    ERR_ALREADY_FLYING = 0x91,          /**< 已在飞行中 */
    ERR_NOT_FLYING = 0x92,              /**< 未在飞行中 */
    ERR_SENSOR_NOT_READY = 0x93,        /**< 传感器未就绪 */
    ERR_BATTERY_LOW = 0x94,             /**< 电池电量低 */
    
    /* 系统错误码 (0xF0-0xFF) */
    ERR_SYSTEM_CRITICAL = 0xF0,         /**< 系统严重错误 */
    ERR_TASK_CREATE_FAILED = 0xF1,      /**< 任务创建失败 */
    ERR_SEMAPHORE_CREATE_FAILED = 0xF2, /**< 信号量创建失败 */
    ERR_QUEUE_CREATE_FAILED = 0xF3,     /**< 队列创建失败 */
} error_code_t;

/*==============================================================================
 * 宏定义 - 错误码检查
 *============================================================================*/
/**
 * @brief 检查错误码是否表示成功
 * @param[in] err_code 错误码
 * @return 1表示成功,0表示失败
 */
#define IS_ERROR_OK(err_code) ((err_code) == ERR_SUCCESS)

/*==============================================================================
 * 函数声明
 *============================================================================*/
/**
 * @brief 获取错误码描述字符串
 * @param[in] err_code 错误码
 * @return 错误描述字符串
 */
const char* error_code_get_string(error_code_t err_code);

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODE_H__ */
