/**
 * @file bsp_w25q64.h
 * @brief W25Q64 Flash存储驱动头文件
 * @details 提供W25Q64 Flash芯片的读写功能，支持日志记录功能
 *          使用SPI2接口(PB12-PB15)
 *          支持FreeRTOS多任务同步
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __BSP_W25Q64_H__
#define __BSP_W25Q64_H__

#include "stm32f10x.h"
#include "system_types.h"
#include "error_code.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 宏定义 - W25Q64指令
 *============================================================================*/
#define W25Q64_WRITE_ENABLE         0x06    /**< 写使能指令 */
#define W25Q64_WRITE_DISABLE        0x04    /**< 写禁止指令 */
#define W25Q64_READ_STATUS_REG1     0x05    /**< 读状态寄存器1 */
#define W25Q64_READ_STATUS_REG2     0x35    /**< 读状态寄存器2 */
#define W25Q64_READ_DATA            0x03    /**< 读数据指令 */
#define W25Q64_FAST_READ            0x0B    /**< 快速读指令 */
#define W25Q64_PAGE_PROGRAM         0x02    /**< 页编程指令 */
#define W25Q64_SECTOR_ERASE         0x20    /**< 扇区擦除指令(4KB) */
#define W25Q64_BLOCK_ERASE_32K      0x52    /**< 块擦除指令(32KB) */
#define W25Q64_BLOCK_ERASE_64K      0xD8    /**< 块擦除指令(64KB) */
#define W25Q64_CHIP_ERASE           0xC7    /**< 全片擦除指令 */
#define W25Q64_POWER_DOWN           0xB9    /**< 掉电指令 */
#define W25Q64_RELEASE_POWER_DOWN   0xAB    /**< 释放掉电指令 */
#define W25Q64_JEDEC_ID             0x9F    /**< 读JEDEC ID指令 */

/*==============================================================================
 * 宏定义 - W25Q64参数
 *============================================================================*/
#define W25Q64_PAGE_SIZE            256     /**< 页大小: 256字节 */
#define W25Q64_SECTOR_SIZE          4096    /**< 扇区大小: 4KB */
#define W25Q64_BLOCK_SIZE_32K       32768   /**< 块大小: 32KB */
#define W25Q64_BLOCK_SIZE_64K       65536   /**< 块大小: 64KB */
#define W25Q64_CHIP_SIZE            8388608 /**< 芯片容量: 8MB (64Mbit) */
#define W25Q64_SECTOR_COUNT         2048    /**< 扇区总数 */
#define W25Q64_PAGE_PER_SECTOR      16      /**< 每扇区页数 */

/*==============================================================================
 * 宏定义 - 日志区域配置
 *============================================================================*/
#define W25Q64_LOG_START_SECTOR     0       /**< 日志起始扇区 */
#define W25Q64_LOG_SECTOR_COUNT     128     /**< 日志占用扇区数: 512KB */
#define W25Q64_LOG_START_ADDR       (W25Q64_LOG_START_SECTOR * W25Q64_SECTOR_SIZE)
#define W25Q64_LOG_END_ADDR         (W25Q64_LOG_START_ADDR + (W25Q64_LOG_SECTOR_COUNT * W25Q64_SECTOR_SIZE))

/*==============================================================================
 * 宏定义 - SPI引脚定义 (软件SPI)
 *============================================================================*/
/* 
 * 硬件连接要求:
 * SCK  -> PB12
 * MISO -> PB13
 * MOSI -> PB14
 * CS   -> PB15
 * 注意：这些引脚与STM32F103 SPI2标准引脚定义不一致，因此使用软件SPI模拟
 */
#define W25Q64_GPIO_CLK             RCC_APB2Periph_GPIOB

#define W25Q64_SCK_PIN              GPIO_Pin_12
#define W25Q64_SCK_PORT             GPIOB

#define W25Q64_MISO_PIN             GPIO_Pin_13
#define W25Q64_MISO_PORT            GPIOB

#define W25Q64_MOSI_PIN             GPIO_Pin_14
#define W25Q64_MOSI_PORT            GPIOB

#define W25Q64_CS_PIN               GPIO_Pin_15
#define W25Q64_CS_PORT              GPIOB

/* 软件SPI操作宏 */
#define W25Q64_SCK_LOW()            GPIO_ResetBits(W25Q64_SCK_PORT, W25Q64_SCK_PIN)
#define W25Q64_SCK_HIGH()           GPIO_SetBits(W25Q64_SCK_PORT, W25Q64_SCK_PIN)

#define W25Q64_MOSI_LOW()           GPIO_ResetBits(W25Q64_MOSI_PORT, W25Q64_MOSI_PIN)
#define W25Q64_MOSI_HIGH()          GPIO_SetBits(W25Q64_MOSI_PORT, W25Q64_MOSI_PIN)

#define W25Q64_MISO_READ()          GPIO_ReadInputDataBit(W25Q64_MISO_PORT, W25Q64_MISO_PIN)

#define W25Q64_CS_LOW()             GPIO_ResetBits(W25Q64_CS_PORT, W25Q64_CS_PIN)
#define W25Q64_CS_HIGH()            GPIO_SetBits(W25Q64_CS_PORT, W25Q64_CS_PIN)

/*==============================================================================
 * 宏定义 - 超时时间
 *============================================================================*/
#define W25Q64_TIMEOUT_MS           1000    /**< 操作超时时间: 1000ms */
#define W25Q64_ERASE_TIMEOUT_MS     5000    /**< 擦除超时时间: 5000ms */

/*==============================================================================
 * 宏定义 - 状态寄存器位
 *============================================================================*/
#define W25Q64_SR1_BUSY             0x01    /**< 忙标志位 */
#define W25Q64_SR1_WEL              0x02    /**< 写使能锁存位 */

/*==============================================================================
 * 类型定义 - 日志条目结构
 *============================================================================*/
/**
 * @brief 日志条目结构体
 * @details 每条日志包含时间戳、控制模式、传感器数据和控制输出
 */
typedef struct {
    uint32_t timestamp;          /**< 时间戳(ms) */
    control_mode_t mode;         /**< 当前控制模式 */
    float accel_x;               /**< X轴加速度(m/s²) */
    float accel_y;               /**< Y轴加速度(m/s²) */
    float accel_z;               /**< Z轴加速度(m/s²) */
    float gyro_x;                /**< X轴角速度(度/s) */
    float gyro_y;                /**< Y轴角速度(度/s) */
    float gyro_z;                /**< Z轴角速度(度/s) */
    float pitch;                 /**< 俯仰角(度) */
    float roll;                  /**< 横滚角(度) */
    float yaw;                   /**< 偏航角(度) */
    float altitude;              /**< 高度(m) */
    float temperature;           /**< 温度(°C) */
    float battery_voltage;       /**< 电池电压(V) */
    float pwm_out[4];            /**< 四个电机PWM输出(0-100%) */
    float pid_output[3];         /**< PID输出(俯仰、横滚、偏航) */
    uint16_t crc16;              /**< CRC16校验 */
} log_entry_t;

/**
 * @brief 日志头结构体
 * @details 存储最新日志的起始位置，用于快速定位日志
 */
typedef struct {
    uint32_t magic;              /**< 魔数: 0x4C4F4701 (LOG\x01) */
    uint32_t version;            /**< 日志格式版本 */
    uint32_t write_addr;         /**< 当前写入地址 */
    uint32_t entry_count;        /**< 日志条目数量 */
    uint32_t wrap_count;         /**< 循环覆盖次数 */
    uint32_t start_time;         /**< 首次记录时间 */
    uint16_t crc16;              /**< CRC16校验 */
} log_header_t;

#define LOG_MAGIC_NUMBER    0x4C4F4701  /**< 日志魔数 "LOG\x01" */
#define LOG_VERSION         0x00010000  /**< 日志版本 1.0.0 */
#define LOG_ENTRY_SIZE      sizeof(log_entry_t)

/*==============================================================================
 * 函数声明 - 初始化和基础操作
 *============================================================================*/
/**
 * @brief 初始化W25Q64 SPI接口
 * @details 配置SPI2 GPIO和SPI参数，初始化互斥锁
 * @return error_code_t 错误码
 *         - ERR_SUCCESS: 初始化成功
 *         - ERR_SPI_INIT_FAILED: SPI初始化失败
 *         - ERR_MUTEX_CREATE_FAILED: 创建互斥锁失败
 */
error_code_t bsp_w25q64_init(void);

/**
 * @brief 读取W25Q64 JEDEC ID
 * @param[out] manufacturer_id 厂商ID
 * @param[out] memory_type 存储器类型
 * @param[out] capacity 容量代码
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_read_jedec_id(uint8_t *manufacturer_id, 
                                       uint8_t *memory_type, 
                                       uint8_t *capacity);

/**
 * @brief 检查W25Q64是否就绪
 * @return bool true: 就绪, false: 忙
 */
bool bsp_w25q64_is_ready(void);

/*==============================================================================
 * 函数声明 - 数据读写
 *============================================================================*/
/**
 * @brief 从W25Q64读取数据
 * @param[in] addr 起始地址(0 - 0x7FFFFF)
 * @param[out] buffer 数据缓冲区
 * @param[in] length 读取长度
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_read(uint32_t addr, uint8_t *buffer, uint32_t length);

/**
 * @brief 向W25Q64写入数据
 * @param[in] addr 起始地址(0 - 0x7FFFFF)
 * @param[in] data 数据缓冲区
 * @param[in] length 写入长度(最大256字节)
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_write(uint32_t addr, const uint8_t *data, uint32_t length);

/**
 * @brief 擦除扇区
 * @param[in] sector_addr 扇区地址(任意地址，自动对齐)
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_erase_sector(uint32_t sector_addr);

/**
 * @brief 擦除块(32KB)
 * @param[in] block_addr 块地址
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_erase_block_32k(uint32_t block_addr);

/**
 * @brief 擦除块(64KB)
 * @param[in] block_addr 块地址
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_erase_block_64k(uint32_t block_addr);

/**
 * @brief 擦除整片
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_erase_chip(void);

/*==============================================================================
 * 函数声明 - 日志功能
 *============================================================================*/
/**
 * @brief 初始化日志系统
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_log_init(void);

/**
 * @brief 写入日志条目
 * @param[in] entry 日志条目指针
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_log_write(const log_entry_t *entry);

/**
 * @brief 读取日志条目
 * @param[in] index 日志索引(0为最新)
 * @param[out] entry 日志条目指针
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_log_read(uint32_t index, log_entry_t *entry);

/**
 * @brief 获取日志条目数量
 * @return uint32_t 日志条目数量
 */
uint32_t bsp_w25q64_log_get_count(void);

/**
 * @brief 清空日志
 * @return error_code_t 错误码
 */
error_code_t bsp_w25q64_log_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_W25Q64_H__ */
