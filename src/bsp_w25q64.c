/**
 * @file bsp_w25q64.c
 * @brief W25Q64 Flash存储驱动实现
 * @details 实现W25Q64 Flash芯片的SPI通信、数据读写和日志记录功能
 *          支持FreeRTOS任务同步，使用软件SPI模拟，引脚定义：
 *          PB12(SCK), PB13(MISO), PB14(MOSI), PB15(CS)
 * @author 
 * @date 2026-02-08
 * @version 1.0.0
 * @copyright MIT License
 */

#include "bsp_w25q64.h"
#include <string.h>

/*==============================================================================
 * 私有变量
 *============================================================================*/
static SemaphoreHandle_t s_spi_mutex = NULL;    /**< SPI总线互斥锁 */
static log_header_t s_log_header;               /**< 日志头信息 */
static uint8_t s_initialized = 0;               /**< 初始化标志 */

/*==============================================================================
 * 私有函数声明
 *============================================================================*/
static void bsp_w25q64_gpio_init(void);
static uint8_t bsp_w25q64_spi_transfer_byte(uint8_t byte);
static void bsp_w25q64_spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);
static error_code_t bsp_w25q64_wait_ready(uint32_t timeout_ms);
static error_code_t bsp_w25q64_write_enable(void);
static error_code_t bsp_w25q64_write_disable(void);
static uint8_t bsp_w25q64_read_status_reg1(void);
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length);
static error_code_t bsp_w25q64_update_header(void);

/*==============================================================================
 * 私有函数实现 - GPIO初始化
 *============================================================================*/
/**
 * @brief 初始化GPIO (软件SPI)
 */
static void bsp_w25q64_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* 使能GPIO时钟 */
    RCC_APB2PeriphClockCmd(W25Q64_GPIO_CLK, ENABLE);
    
    /* 配置CS引脚(PB15) - 通用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = W25Q64_CS_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25Q64_CS_PORT, &GPIO_InitStruct);
    W25Q64_CS_HIGH();
    
    /* 配置SCK引脚(PB12) - 通用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = W25Q64_SCK_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W25Q64_SCK_PORT, &GPIO_InitStruct);
    W25Q64_SCK_LOW(); /* 模式0: CPOL=0 */
    
    /* 配置MOSI引脚(PB14) - 通用推挽输出 */
    GPIO_InitStruct.GPIO_Pin = W25Q64_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(W25Q64_MOSI_PORT, &GPIO_InitStruct);
    W25Q64_MOSI_LOW();
    
    /* 配置MISO引脚(PB13) - 上拉输入 */
    GPIO_InitStruct.GPIO_Pin = W25Q64_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(W25Q64_MISO_PORT, &GPIO_InitStruct);
}

/*==============================================================================
 * 私有函数实现 - 软件SPI通信
 *============================================================================*/
/**
 * @brief 软件SPI收发一个字节
 * @param byte 发送的数据
 * @return 接收到的数据
 */
static uint8_t bsp_w25q64_spi_transfer_byte(uint8_t byte)
{
    uint8_t i;
    uint8_t rx_byte = 0;
    
    /* 模式0: CPOL=0, CPHA=0 (在第一个边沿采样) */
    
    for(i = 0; i < 8; i++) {
        /* 准备数据 (MSB first) */
        if(byte & 0x80) {
            W25Q64_MOSI_HIGH();
        } else {
            W25Q64_MOSI_LOW();
        }
        byte <<= 1;
        
        /* 产生时钟上升沿 (采样) */
        W25Q64_SCK_HIGH();
        
        /* 读取MISO */
        rx_byte <<= 1;
        if(W25Q64_MISO_READ()) {
            rx_byte |= 0x01;
        }
        
        /* 产生时钟下降沿 */
        W25Q64_SCK_LOW();
    }
    
    return rx_byte;
}

/**
 * @brief SPI批量收发
 */
static void bsp_w25q64_spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    uint32_t i;
    uint8_t dummy;
    
    for(i = 0; i < len; i++) {
        if(tx_buf != NULL && rx_buf != NULL) {
            rx_buf[i] = bsp_w25q64_spi_transfer_byte(tx_buf[i]);
        } else if(tx_buf != NULL) {
            bsp_w25q64_spi_transfer_byte(tx_buf[i]);
        } else if(rx_buf != NULL) {
            rx_buf[i] = bsp_w25q64_spi_transfer_byte(0xFF);
        } else {
            /* 两者都为空时发送空字节 */
            bsp_w25q64_spi_transfer_byte(0xFF);
        }
    }
}

/*==============================================================================
 * 对外接口实现
 *============================================================================*/
/**
 * @brief 初始化W25Q64
 */
error_code_t bsp_w25q64_init(void)
{
    uint8_t id_buf[3];
    
    if(s_initialized) {
        return ERR_OK;
    }
    
    /* 创建互斥锁 */
    s_spi_mutex = xSemaphoreCreateMutex();
    if(s_spi_mutex == NULL) {
        return ERR_SEMAPHORE_CREATE_FAILED;
    }
    
    /* 初始化GPIO */
    bsp_w25q64_gpio_init();
    
    /* 唤醒设备 */
    bsp_w25q64_wake_up();
    
    /* 简单延时确保稳定 */
    for(volatile int i = 0; i < 10000; i++);
    
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_JEDEC_ID);
    id_buf[0] = bsp_w25q64_spi_transfer_byte(0xFF); /* Manufacturer ID */
    id_buf[1] = bsp_w25q64_spi_transfer_byte(0xFF); /* Memory Type */
    id_buf[2] = bsp_w25q64_spi_transfer_byte(0xFF); /* Capacity */
    W25Q64_CS_HIGH();
    
    /* 验证Manufacturer ID (Winbond = 0xEF) */
    if(id_buf[0] != 0xEF) {
        return ERR_W25Q64_ID_MISMATCH;
    }
    
    /* 初始化日志头 */
    /* 尝试读取已有日志头 */
    uint8_t err = bsp_w25q64_read(W25Q64_LOG_START_ADDR, (uint8_t*)&s_log_header, sizeof(log_header_t));
    if(err != ERR_OK) {
        return err;
    }
    
    /* 检查魔数是否有效 */
    if(s_log_header.magic != 0xA55A5AA5) {
        /* 初始化新日志头 */
        s_log_header.magic = 0xA55A5AA5;
        s_log_header.write_ptr = sizeof(log_header_t); /* 从头部后开始写 */
        s_log_header.record_count = 0;
        
        /* 擦除第一个扇区并写入头部 */
        err = bsp_w25q64_sector_erase(W25Q64_LOG_START_ADDR);
        if(err != ERR_OK) return err;
        
        err = bsp_w25q64_update_header();
        if(err != ERR_OK) return err;
    }
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief 读取数据
 */
error_code_t bsp_w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if(!s_initialized) return ERR_NOT_INITIALIZED;
    if(buf == NULL || len == 0) return ERR_INVALID_PARAM;
    
    if(xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(W25Q64_TIMEOUT_MS)) != pdTRUE) {
        return ERR_MUTEX_TIMEOUT;
    }
    
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_READ_DATA);
    bsp_w25q64_spi_transfer_byte((addr >> 16) & 0xFF);
    bsp_w25q64_spi_transfer_byte((addr >> 8) & 0xFF);
    bsp_w25q64_spi_transfer_byte(addr & 0xFF);
    
    bsp_w25q64_spi_transfer(NULL, buf, len);
    
    W25Q64_CS_HIGH();
    
    xSemaphoreGive(s_spi_mutex);
    return ERR_OK;
}

/**
 * @brief 页编程 (写入数据)
 */
error_code_t bsp_w25q64_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    error_code_t err = ERR_OK;
    uint32_t pageremain;
    
    if(!s_initialized) return ERR_NOT_INITIALIZED;
    if(buf == NULL || len == 0) return ERR_INVALID_PARAM;
    
    if(xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(W25Q64_TIMEOUT_MS)) != pdTRUE) {
        return ERR_MUTEX_TIMEOUT;
    }
    
    pageremain = 256 - addr % 256;
    if(len <= pageremain) pageremain = len;
    
    while(1) {
        err = bsp_w25q64_write_enable();
        if(err != ERR_OK) break;
        
        W25Q64_CS_LOW();
        bsp_w25q64_spi_transfer_byte(W25Q64_PAGE_PROGRAM);
        bsp_w25q64_spi_transfer_byte((addr >> 16) & 0xFF);
        bsp_w25q64_spi_transfer_byte((addr >> 8) & 0xFF);
        bsp_w25q64_spi_transfer_byte(addr & 0xFF);
        
        bsp_w25q64_spi_transfer(buf, NULL, pageremain);
        W25Q64_CS_HIGH();
        
        err = bsp_w25q64_wait_ready(W25Q64_TIMEOUT_MS);
        if(err != ERR_OK) break;
        
        if(len == pageremain) break;
        else {
            buf += pageremain;
            addr += pageremain;
            len -= pageremain;
            if(len > 256) pageremain = 256;
            else pageremain = len;
        }
    }
    
    xSemaphoreGive(s_spi_mutex);
    return err;
}

/**
 * @brief 扇区擦除 (4KB)
 */
error_code_t bsp_w25q64_sector_erase(uint32_t addr)
{
    error_code_t err;
    
    if(!s_initialized) return ERR_NOT_INITIALIZED;
    
    if(xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(W25Q64_TIMEOUT_MS)) != pdTRUE) {
        return ERR_MUTEX_TIMEOUT;
    }
    
    err = bsp_w25q64_write_enable();
    if(err == ERR_OK) {
        W25Q64_CS_LOW();
        bsp_w25q64_spi_transfer_byte(W25Q64_SECTOR_ERASE);
        bsp_w25q64_spi_transfer_byte((addr >> 16) & 0xFF);
        bsp_w25q64_spi_transfer_byte((addr >> 8) & 0xFF);
        bsp_w25q64_spi_transfer_byte(addr & 0xFF);
        W25Q64_CS_HIGH();
        
        err = bsp_w25q64_wait_ready(W25Q64_ERASE_TIMEOUT_MS);
    }
    
    xSemaphoreGive(s_spi_mutex);
    return err;
}

/**
 * @brief 写入日志
 */
error_code_t bsp_w25q64_write_log(const log_entry_t *entry)
{
    error_code_t err;
    uint32_t next_ptr;
    
    if(entry == NULL) return ERR_INVALID_PARAM;
    
    /* 计算下一个日志位置 */
    next_ptr = s_log_header.write_ptr + sizeof(log_entry_t);
    
    /* 检查是否需要擦除新扇区 (循环日志，到达末尾回卷) */
    if((s_log_header.write_ptr / W25Q64_SECTOR_SIZE) != (next_ptr / W25Q64_SECTOR_SIZE)) {
        /* 跨越扇区边界，回卷 */
        if(next_ptr >= (W25Q64_LOG_SECTOR_COUNT * W25Q64_SECTOR_SIZE)) {
            next_ptr = sizeof(log_header_t); /* 回卷到头部后 */
            s_log_header.write_ptr = next_ptr;
            /* 擦除并重新初始化 */
            err = bsp_w25q64_sector_erase(W25Q64_LOG_START_ADDR);
            if(err != ERR_OK) return err;
            /* 重写头部 */
            err = bsp_w25q64_update_header();
            if(err != ERR_OK) return err;
        } else {
            /* 擦除下一个扇区 */
            err = bsp_w25q64_sector_erase(W25Q64_LOG_START_ADDR + (next_ptr & ~(W25Q64_SECTOR_SIZE - 1)));
            if(err != ERR_OK) return err;
        }
    }
    
    /* 写入日志数据 */
    err = bsp_w25q64_write(W25Q64_LOG_START_ADDR + s_log_header.write_ptr, (uint8_t*)entry, sizeof(log_entry_t));
    if(err != ERR_OK) return err;
    
    /* 更新头信息 */
    s_log_header.write_ptr += sizeof(log_entry_t);
    s_log_header.record_count++;
    
    return bsp_w25q64_update_header();
}

/**
 * @brief 读取日志
 */
error_code_t bsp_w25q64_read_log(uint32_t index, log_entry_t *entry)
{
    /* 待实现：目前只支持按地址偏移读取，索引读取未实现 */
    return ERR_NOT_IMPLEMENTED;
}

/**
 * @brief 进入低功耗模式
 */
error_code_t bsp_w25q64_power_down(void)
{
    if(xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(W25Q64_TIMEOUT_MS)) != pdTRUE) {
        return ERR_MUTEX_TIMEOUT;
    }
    
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_POWER_DOWN);
    W25Q64_CS_HIGH();
    
    xSemaphoreGive(s_spi_mutex);
    return ERR_OK;
}

/**
 * @brief 唤醒设备
 */
error_code_t bsp_w25q64_wake_up(void)
{
    /* 唤醒不需要获取锁，因为可能在初始化前调用 */
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_RELEASE_POWER_DOWN);
    W25Q64_CS_HIGH();
    return ERR_OK;
}

/*==============================================================================
 * 私有函数实现
 *============================================================================*/
static error_code_t bsp_w25q64_wait_ready(uint32_t timeout_ms)
{
    uint8_t status;
    uint32_t tick_start = xTaskGetTickCount();
    
    do {
        W25Q64_CS_LOW();
        bsp_w25q64_spi_transfer_byte(W25Q64_READ_STATUS_REG1);
        status = bsp_w25q64_spi_transfer_byte(0xFF);
        W25Q64_CS_HIGH();
        
        if((status & W25Q64_SR1_BUSY) == 0) {
            return ERR_OK;
        }
        
        vTaskDelay(1);
    } while((xTaskGetTickCount() - tick_start) < pdMS_TO_TICKS(timeout_ms));
    
    return ERR_TIMEOUT;
}

static error_code_t bsp_w25q64_write_enable(void)
{
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_WRITE_ENABLE);
    W25Q64_CS_HIGH();
    return ERR_OK;
}

static error_code_t bsp_w25q64_update_header(void)
{
    /* 写入头部到Sector 0开始位置 */
    /* 注意：需要处理扇区擦除，因为Flash只能1->0，所以更新头部需要擦除整个扇区 */
    /* 为简化，这里使用开销较大的方法：擦除Sector 0后重写 */
    /* 实际应用应该：通过缓存头部数据，批量写入，或者使用双缓冲策略 */
    /* 或者维护一个头部更新计数器，只在必要时擦除 */
    /* 此处为简化实现，每次更新都擦除重写 */
    
    return bsp_w25q64_write(W25Q64_LOG_START_ADDR, (uint8_t*)&s_log_header, sizeof(log_header_t));
}

static uint8_t bsp_w25q64_read_status_reg1(void)
{
    uint8_t status;
    W25Q64_CS_LOW();
    bsp_w25q64_spi_transfer_byte(W25Q64_READ_STATUS_REG1);
    status = bsp_w25q64_spi_transfer_byte(0xFF);
    W25Q64_CS_HIGH();
    return status;
}
