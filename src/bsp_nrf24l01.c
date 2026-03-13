/**
 * @file bsp_nrf24l01.c
 * @brief NRF24L01无线通信驱动实现
 * @details 实数NRF24L01 2.4GHz无线收发功能
 *          使用SPI1接口,支持Enhanced ShockBurst模式
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#include "bsp_nrf24l01.h"
#include <string.h>

/*==============================================================================
 * 私有变量
 *============================================================================*/
static SemaphoreHandle_t s_spi_mutex = NULL;        /**< SPI总线互斥锁 */
static nrf24l01_state_t s_state = NRF24L01_STATE_IDLE;  /**< 当前状态 */
static nrf24l01_stats_t s_stats;                    /**< 统计信息 */
static bool s_initialized = false;                  /**< 初始化标志 */

/* 默认地址 */
static const uint8_t s_default_address[NRF24L01_ADDR_WIDTH] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

/*==============================================================================
 * 私有函数声明
 *============================================================================*/
static void bsp_nrf24l01_spi_gpio_init(void);
static void bsp_nrf24l01_spi_periph_init(void);
static uint8_t bsp_nrf24l01_spi_transfer_byte(uint8_t byte);
static uint8_t bsp_nrf24l01_read_reg(uint8_t reg);
static error_code_t bsp_nrf24l01_write_reg(uint8_t reg, uint8_t value);
static error_code_t bsp_nrf24l01_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t length);
static error_code_t bsp_nrf24l01_write_buffer(uint8_t reg, const uint8_t *buffer, uint8_t length);
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length);

/*==============================================================================
 * 私有函数实现 - GPIO数SPI初始化
 *============================================================================*/
/**
 * @brief 初始化SPI1 GPIO
 */
static void bsp_nrf24l01_spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* 使能GPIO时钟 */
    RCC_APB2PeriphClockCmd(NRF24L01_SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    
    /* 配置CE引脚(PA4) - 错误码?*/
    GPIO_InitStruct.GPIO_Pin = NRF24L01_CE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);
    NRF24L01_CE_LOW();
    
    /* 配置CS引脚(PA8) - 错误码?*/
    GPIO_InitStruct.GPIO_Pin = NRF24L01_CS_PIN;
    GPIO_Init(NRF24L01_CS_PORT, &GPIO_InitStruct);
    NRF24L01_CS_HIGH();
    
    /* 配置SCK引脚(PA5) - 读取数据数?*/
    GPIO_InitStruct.GPIO_Pin = NRF24L01_SCK_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(NRF24L01_SCK_PORT, &GPIO_InitStruct);
    
    /* 配置MISO引脚(PA6) - 读取数据 */
    GPIO_InitStruct.GPIO_Pin = NRF24L01_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(NRF24L01_MISO_PORT, &GPIO_InitStruct);
    
    /* 配置MOSI引脚(PA7) - 读取数据数?*/
    GPIO_InitStruct.GPIO_Pin = NRF24L01_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(NRF24L01_MOSI_PORT, &GPIO_InitStruct);
    
    /* 配置IRQ引脚(PB0) - 读取数据 */
    /* 使能GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin = NRF24L01_IRQ_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);
}

/**
 * @brief 初始化SPI1外设
 */
static void bsp_nrf24l01_spi_periph_init(void)
{
    SPI_InitTypeDef SPI_InitStruct;
    
    /* 使能SPI1时钟 */
    RCC_APB2PeriphClockCmd(NRF24L01_SPI_CLK, ENABLE);
    
    /* 配置SPI参数 */
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;         /* 空闲时钟低电平 */
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;       /* 在第一个边沿采样 */
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;          /* 软件NSS */
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; /* 72MHz/8 = 9MHz */
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    
    SPI_Init(NRF24L01_SPI, &SPI_InitStruct);
    SPI_Cmd(NRF24L01_SPI, ENABLE);
}

/*==============================================================================
 * 私有函数实现 - SPI通信
 *============================================================================*/
/**
 * @brief SPI字节传输
 * @param[in] byte 接收到的字节
 * @return uint8_t 接收到的字节
 */
static uint8_t bsp_nrf24l01_spi_transfer_byte(uint8_t byte)
{
    /* 等待发送缓冲区为空 */
    while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_TXE) == RESET);
    
    /* 读取数据 */
    SPI_I2S_SendData(NRF24L01_SPI, byte);
    
    /* 等待接收缓冲区非空 */
    while (SPI_I2S_GetFlagStatus(NRF24L01_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    
    /* 数取读取数据 */
    return SPI_I2S_ReceiveData(NRF24L01_SPI);
}

/**
 * @brief 读寄存器
 * @param[in] reg 寄存器地址
 * @return uint8_t 寄存器值
 */
static uint8_t bsp_nrf24l01_read_reg(uint8_t reg)
{
    uint8_t value;
    
    NRF24L01_CS_LOW();
    bsp_nrf24l01_spi_transfer_byte(NRF24L01_CMD_R_REGISTER | reg);
    value = bsp_nrf24l01_spi_transfer_byte(0xFF);
    NRF24L01_CS_HIGH();
    
    return value;
}

/**
 * @brief 写寄存器
 * @param[in] reg 寄存器地址
 * @param[in] value 寄存器值
 * @return error_code_t 错误码
 */
static error_code_t bsp_nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    NRF24L01_CS_LOW();
    bsp_nrf24l01_spi_transfer_byte(NRF24L01_CMD_W_REGISTER | reg);
    bsp_nrf24l01_spi_transfer_byte(value);
    NRF24L01_CS_HIGH();
    
    return ERR_SUCCESS;
}

/**
 * @brief 读取数据
 * @param[in] reg 寄存器地址
 * @param[out] buffer 数据缓冲区
 * @param[in] length 数据长度
 * @return error_code_t 错误码
 */
static error_code_t bsp_nrf24l01_read_buffer(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (buffer == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    NRF24L01_CS_LOW();
    bsp_nrf24l01_spi_transfer_byte(reg);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = bsp_nrf24l01_spi_transfer_byte(0xFF);
    }
    NRF24L01_CS_HIGH();
    
    return ERR_SUCCESS;
}

/**
 * @brief 写错误码
 * @param[in] reg 寄存器地址
 * @param[in] buffer 数据缓冲区
 * @param[in] length 数据长度
 * @return error_code_t 错误码
 */
static error_code_t bsp_nrf24l01_write_buffer(uint8_t reg, const uint8_t *buffer, uint8_t length)
{
    if (buffer == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    NRF24L01_CS_LOW();
    bsp_nrf24l01_spi_transfer_byte(reg);
    for (uint8_t i = 0; i < length; i++) {
        bsp_nrf24l01_spi_transfer_byte(buffer[i]);
    }
    NRF24L01_CS_HIGH();
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 私有函数实现 - CRC16校验
 *============================================================================*/
/**
 * @brief CRC16-CCITT配置
 * @param[in] data 数据指针
 * @param[in] length 数据长度
 * @return uint16_t CRC16值
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/*==============================================================================
 * 对外接口实现 - 初始化
 *============================================================================*/
error_code_t bsp_nrf24l01_init(void)
{
    if (s_initialized) {
        return ERR_SUCCESS;
    }
    
    /* 初始化GPIO */
    bsp_nrf24l01_spi_gpio_init();
    
    /* 初始化SPI */
    bsp_nrf24l01_spi_periph_init();
    
    /* 读取数据数 */
    s_spi_mutex = xSemaphoreCreateMutex();
    if (s_spi_mutex == NULL) {
        return ERR_MUTEX_CREATE_FAILED;
    }
    
    /* 延时等待NRF24L01上电 */
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置NRF24L01寄存器 */
    
    /* 配置: 使能CRC(2字节),上电 */
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, 
                           NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    /* 使能自动应答(所有通道) */
    bsp_nrf24l01_write_reg(NRF24L01_REG_EN_AA, 0x3F);
    
    /* 使能接收地址(通道0和通道1) */
    bsp_nrf24l01_write_reg(NRF24L01_REG_EN_RXADDR, 0x03);
    
    /* 数矫碉址配置为5街 */
    bsp_nrf24l01_write_reg(NRF24L01_REG_SETUP_AW, 0x03);
    
    /* 配置皆讹截凤: 重传间隔500us,配置胤?0数 */
    bsp_nrf24l01_write_reg(NRF24L01_REG_SETUP_RETR, 0x1A);
    
    /* 配置RF通数 */
    bsp_nrf24l01_write_reg(NRF24L01_REG_RF_CH, NRF24L01_RF_CHANNEL);
    
    /* 配置RF配置: 1Mbps, 0dBm */
    uint8_t rf_setup = 0x06;  /* 1Mbps, 0dBm */
    bsp_nrf24l01_write_reg(NRF24L01_REG_RF_SETUP, rf_setup);
    
    /* 数矫配置捷匡数 */
    bsp_nrf24l01_write_reg(NRF24L01_REG_RX_PW_P0, NRF24L01_PAYLOAD_SIZE);
    bsp_nrf24l01_write_reg(NRF24L01_REG_RX_PW_P1, NRF24L01_PAYLOAD_SIZE);
    
    /* 配置默认地址 */
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_TX_ADDR, 
                              s_default_address, NRF24L01_ADDR_WIDTH);
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_RX_ADDR_P0, 
                              s_default_address, NRF24L01_ADDR_WIDTH);
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_RX_ADDR_P1, 
                              s_default_address, NRF24L01_ADDR_WIDTH);
    
    /* 清FIFO */
    bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
    bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
    
    /* 配置卸媳街?*/
    bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, 0x70);
    
    xSemaphoreGive(s_spi_mutex);
    
    /* 初始化为配置模式 */
    bsp_nrf24l01_set_rx_mode();
    
    /* 初始化统计信息 */
    memset(&s_stats, 0, sizeof(s_stats));
    
    s_initialized = true;
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_set_rx_mode(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置为配置模式 */
    uint8_t config = bsp_nrf24l01_read_reg(NRF24L01_REG_CONFIG);
    config |= NRF24L01_CONFIG_PRIM_RX;
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, config);
    
    /* 配置CE */
    NRF24L01_CE_HIGH();
    vTaskDelay(pdMS_TO_TICKS(1));
    
    s_state = NRF24L01_STATE_RX;
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_set_tx_mode(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置CE */
    NRF24L01_CE_LOW();
    
    /* 配置为配置模式 */
    uint8_t config = bsp_nrf24l01_read_reg(NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_CONFIG_PRIM_RX;
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, config);
    
    s_state = NRF24L01_STATE_TX;
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_power_down(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置CE */
    NRF24L01_CE_LOW();
    
    /* 清除PWR_UP位 */
    uint8_t config = bsp_nrf24l01_read_reg(NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_CONFIG_PWR_UP;
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, config);
    
    s_state = NRF24L01_STATE_IDLE;
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_power_up(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置PWR_UP位 */
    uint8_t config = bsp_nrf24l01_read_reg(NRF24L01_REG_CONFIG);
    config |= NRF24L01_CONFIG_PWR_UP;
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, config);
    
    vTaskDelay(pdMS_TO_TICKS(5));
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 对外接口实现 - 配置秸凤
 *============================================================================*/
error_code_t bsp_nrf24l01_send_packet(const uint8_t *data, uint8_t length)
{
    error_code_t err;
    
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (data == NULL || length == 0 || length > NRF24L01_PAYLOAD_SIZE) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 配置为配置模式 */
    NRF24L01_CE_LOW();
    uint8_t config = bsp_nrf24l01_read_reg(NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_CONFIG_PRIM_RX;
    bsp_nrf24l01_write_reg(NRF24L01_REG_CONFIG, config);
    
    /* 数絋X FIFO */
    bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
    
    /* 配置卸媳街?*/
    bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, 0x70);
    
    /* 写错误码 */
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_TX_PAYLOAD, data, length);
    
    /* 配置CE初始化数 */
    NRF24L01_CE_HIGH();
    vTaskDelay(pdMS_TO_TICKS(1));
    NRF24L01_CE_LOW();
    
    /* 饺达错误码?*/
    uint32_t start_tick = xTaskGetTickCount();
    bool tx_done = false;
    
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) {
        uint8_t status = bsp_nrf24l01_read_reg(NRF24L01_REG_STATUS);
        
        if (status & NRF24L01_STATUS_TX_DS) {
            /* 发送成功 */
            bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_TX_DS);
            s_stats.tx_packets++;
            tx_done = true;
            break;
        }
        
        if (status & NRF24L01_STATUS_MAX_RT) {
            /* 到配置胤配置?*/
            bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_MAX_RT);
            bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
            s_stats.lost_packets++;
            s_stats.tx_errors++;
            xSemaphoreGive(s_spi_mutex);
            return ERR_NRF24L01_MAX_RT;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    xSemaphoreGive(s_spi_mutex);
    
    if (!tx_done) {
        s_stats.tx_errors++;
        return ERR_NRF24L01_TIMEOUT;
    }
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_receive_packet(uint8_t *data, uint8_t *length, uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (data == NULL || length == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    /* 确配置节数模式 */
    if (s_state != NRF24L01_STATE_RX) {
        bsp_nrf24l01_set_rx_mode();
    }
    
    /* 配置欠错误码?*/
    uint32_t start_tick = xTaskGetTickCount();
    bool rx_done = false;
    
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t status = bsp_nrf24l01_read_reg(NRF24L01_REG_STATUS);
        
        if (status & NRF24L01_STATUS_RX_DR) {
            /* 叫错误码 */
            uint8_t pipe = (status >> 1) & 0x07;
            
            /* 数取配置 */
            bsp_nrf24l01_read_buffer(NRF24L01_CMD_R_RX_PAYLOAD, data, NRF24L01_PAYLOAD_SIZE);
            *length = NRF24L01_PAYLOAD_SIZE;
            
            /* 配置卸媳街?*/
            bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);
            
            s_stats.rx_packets++;
            rx_done = true;
            break;
        }
        
        xSemaphoreGive(s_spi_mutex);
        vTaskDelay(pdMS_TO_TICKS(1));
        
        if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            return ERR_SPI_BUS_BUSY;
        }
    }
    
    xSemaphoreGive(s_spi_mutex);
    
    if (!rx_done) {
        return ERR_NRF24L01_TIMEOUT;
    }
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_send_control(const control_packet_t *packet)
{
    if (packet == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    control_packet_t tx_packet;
    memcpy(&tx_packet, packet, sizeof(control_packet_t));
    
    /* 配置帧头错误码 */
    tx_packet.header = 0xAA;
    tx_packet.type = PACKET_TYPE_CONTROL;
    
    /* 配置CRC */
    tx_packet.crc16 = crc16_ccitt((uint8_t *)&tx_packet, sizeof(control_packet_t) - sizeof(uint16_t));
    
    return bsp_nrf24l01_send_packet((uint8_t *)&tx_packet, sizeof(control_packet_t));
}

error_code_t bsp_nrf24l01_receive_control(control_packet_t *packet, uint32_t timeout_ms)
{
    error_code_t err;
    uint8_t length;
    
    if (packet == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    err = bsp_nrf24l01_receive_packet((uint8_t *)packet, &length, timeout_ms);
    if (err != ERR_SUCCESS) {
        return err;
    }
    
    /* 验证帧头 */
    if (packet->header != 0xAA || packet->type != PACKET_TYPE_CONTROL) {
        s_stats.rx_errors++;
        return ERR_INVALID_PACKET;
    }
    
    /* 验证CRC */
    uint16_t calc_crc = crc16_ccitt((uint8_t *)packet, sizeof(control_packet_t) - sizeof(uint16_t));
    if (calc_crc != packet->crc16) {
        s_stats.rx_errors++;
        return ERR_CRC_ERROR;
    }
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_send_telemetry(const telemetry_packet_t *packet)
{
    if (packet == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    telemetry_packet_t tx_packet;
    memcpy(&tx_packet, packet, sizeof(telemetry_packet_t));
    
    /* 配置帧头错误码 */
    tx_packet.header = 0xBB;
    tx_packet.type = PACKET_TYPE_TELEMETRY;
    
    /* 配置CRC */
    tx_packet.crc16 = crc16_ccitt((uint8_t *)&tx_packet, sizeof(telemetry_packet_t) - sizeof(uint16_t));
    
    return bsp_nrf24l01_send_packet((uint8_t *)&tx_packet, sizeof(telemetry_packet_t));
}

bool bsp_nrf24l01_data_available(void)
{
    if (!s_initialized) {
        return false;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    
    uint8_t status = bsp_nrf24l01_read_reg(NRF24L01_REG_STATUS);
    bool available = (status & NRF24L01_STATUS_RX_DR) != 0;
    
    xSemaphoreGive(s_spi_mutex);
    
    return available;
}

/*==============================================================================
 * 对外接口实现 - 数址数通错误码
 *============================================================================*/
error_code_t bsp_nrf24l01_set_tx_address(const uint8_t *addr)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (addr == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_TX_ADDR, 
                              addr, NRF24L01_ADDR_WIDTH);
    /* 杰碉0也配置为数同数址(数节数ACK) */
    bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | NRF24L01_REG_RX_ADDR_P0, 
                              addr, NRF24L01_ADDR_WIDTH);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_set_rx_address(uint8_t pipe, const uint8_t *addr)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (addr == NULL || pipe > 5) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    uint8_t reg = NRF24L01_REG_RX_ADDR_P0 + pipe;
    
    if (pipe < 2) {
        /* 杰碉0数1使数5街节碉址 */
        bsp_nrf24l01_write_buffer(NRF24L01_CMD_W_REGISTER | reg, addr, NRF24L01_ADDR_WIDTH);
    } else {
        /* 杰碉2-5只使数1街节碉址(数街) */
        bsp_nrf24l01_write_reg(reg, addr[0]);
    }
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_set_channel(uint8_t channel)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (channel > 125) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    bsp_nrf24l01_write_reg(NRF24L01_REG_RF_CH, channel);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_set_power(uint8_t power)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    if (power > 3) {
        return ERR_INVALID_PARAM;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    uint8_t rf_setup = bsp_nrf24l01_read_reg(NRF24L01_REG_RF_SETUP);
    rf_setup &= ~0x06;  /* 错误码轿?*/
    rf_setup |= (power << 1);
    bsp_nrf24l01_write_reg(NRF24L01_REG_RF_SETUP, rf_setup);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 对外接口实现 - 叫断猴状态
 *============================================================================*/
uint8_t bsp_nrf24l01_get_status(void)
{
    if (!s_initialized) {
        return 0;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return 0;
    }
    
    uint8_t status = bsp_nrf24l01_read_reg(NRF24L01_REG_STATUS);
    
    xSemaphoreGive(s_spi_mutex);
    
    return status;
}

error_code_t bsp_nrf24l01_flush_tx(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_flush_rx(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    bsp_nrf24l01_write_reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_clear_irq_flags(uint8_t flags)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    bsp_nrf24l01_write_reg(NRF24L01_REG_STATUS, flags & 0x70);
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_get_stats(nrf24l01_stats_t *stats)
{
    if (stats == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    memcpy(stats, &s_stats, sizeof(nrf24l01_stats_t));
    
    /* 数取数酵观诧拇配置饺★胤配置?*/
    uint8_t observe = bsp_nrf24l01_read_reg(NRF24L01_REG_OBSERVE_TX);
    s_stats.retransmissions += (observe >> 4) & 0x0F;
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}

error_code_t bsp_nrf24l01_reset_stats(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 数取错误码 */
    if (xSemaphoreTake(s_spi_mutex, pdMS_TO_TICKS(NRF24L01_TIMEOUT_MS)) != pdTRUE) {
        return ERR_SPI_BUS_BUSY;
    }
    
    memset(&s_stats, 0, sizeof(s_stats));
    
    xSemaphoreGive(s_spi_mutex);
    
    return ERR_SUCCESS;
}
