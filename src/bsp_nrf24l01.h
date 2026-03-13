/**
 * @file bsp_nrf24l01.h
 * @brief NRF24L01无线通信驱动头文件
 * @details 提供NRF24L01 2.4GHz无线收发功能
 *          使用SPI1接口(PA5-PA7),支持Enhanced ShockBurst模式
 *          支持FreeRTOS多任务同步
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __BSP_NRF24L01_H__
#define __BSP_NRF24L01_H__

#include "stm32f10x.h"
#include "system_types.h"
#include "error_code.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 宏定义 - NRF24L01寄存器地址
 *============================================================================*/
#define NRF24L01_REG_CONFIG         0x00    /**< 配置寄存器 */
#define NRF24L01_REG_EN_AA          0x01    /**< 自动应答使能 */
#define NRF24L01_REG_EN_RXADDR      0x02    /**< 接收地址使能 */
#define NRF24L01_REG_SETUP_AW       0x03    /**< 地址宽度配置 */
#define NRF24L01_REG_SETUP_RETR     0x04    /**< 自动重发配置 */
#define NRF24L01_REG_RF_CH          0x05    /**< RF通道 */
#define NRF24L01_REG_RF_SETUP       0x06    /**< RF设置 */
#define NRF24L01_REG_STATUS         0x07    /**< 状态寄存器 */
#define NRF24L01_REG_OBSERVE_TX     0x08    /**< 发送观察 */
#define NRF24L01_REG_CD             0x09    /**< 载波检测 */
#define NRF24L01_REG_RX_ADDR_P0     0x0A    /**< 数据管道0接收地址 */
#define NRF24L01_REG_RX_ADDR_P1     0x0B    /**< 数据管道1接收地址 */
#define NRF24L01_REG_RX_ADDR_P2     0x0C    /**< 数据管道2接收地址 */
#define NRF24L01_REG_RX_ADDR_P3     0x0D    /**< 数据管道3接收地址 */
#define NRF24L01_REG_RX_ADDR_P4     0x0E    /**< 数据管道4接收地址 */
#define NRF24L01_REG_RX_ADDR_P5     0x0F    /**< 数据管道5接收地址 */
#define NRF24L01_REG_TX_ADDR        0x10    /**< 发送地址 */
#define NRF24L01_REG_RX_PW_P0       0x11    /**< 数据管道0接收数据宽度 */
#define NRF24L01_REG_RX_PW_P1       0x12    /**< 数据管道1接收数据宽度 */
#define NRF24L01_REG_RX_PW_P2       0x13    /**< 数据管道2接收数据宽度 */
#define NRF24L01_REG_RX_PW_P3       0x14    /**< 数据管道3接收数据宽度 */
#define NRF24L01_REG_RX_PW_P4       0x15    /**< 数据管道4接收数据宽度 */
#define NRF24L01_REG_RX_PW_P5       0x16    /**< 数据管道5接收数据宽度 */
#define NRF24L01_REG_FIFO_STATUS    0x17    /**< FIFO状态 */
#define NRF24L01_REG_DYNPD          0x1C    /**< 动态载荷长度 */
#define NRF24L01_REG_FEATURE        0x1D    /**< 特性寄存器 */

/*==============================================================================
 * 宏定义 - NRF24L01指令
 *============================================================================*/
#define NRF24L01_CMD_R_REGISTER     0x00    /**< 读寄存器 */
#define NRF24L01_CMD_W_REGISTER     0x20    /**< 写寄存器 */
#define NRF24L01_CMD_R_RX_PAYLOAD   0x61    /**< 读RX载荷 */
#define NRF24L01_CMD_W_TX_PAYLOAD   0xA0    /**< 写TX载荷 */
#define NRF24L01_CMD_FLUSH_TX       0xE1    /**< 清TX FIFO */
#define NRF24L01_CMD_FLUSH_RX       0xE2    /**< 清RX FIFO */
#define NRF24L01_CMD_REUSE_TX_PL    0xE3    /**< 重用TX载荷 */
#define NRF24L01_CMD_R_RX_PL_WID    0x60    /**< 读RX载荷宽度 */
#define NRF24L01_CMD_W_ACK_PAYLOAD  0xA8    /**< 写ACK载荷 */
#define NRF24L01_CMD_W_TX_PAYLOAD_NOACK 0xB0 /**< 写TX载荷(无应答) */
#define NRF24L01_CMD_NOP            0xFF    /**< 空操作 */

/*==============================================================================
 * 宏定义 - NRF24L01配置参数
 *============================================================================*/
#define NRF24L01_ADDR_WIDTH         5       /**< 地址宽度: 5字节 */
#define NRF24L01_PAYLOAD_SIZE       32      /**< 载荷大小: 32字节 */
#define NRF24L01_RF_CHANNEL         76      /**< RF通道: 2476MHz */
#define NRF24L01_RF_DATARATE        0       /**< 数据率: 1Mbps (0=1Mbps, 1=2Mbps, 2=250kbps) */
#define NRF24L01_RF_PWR             3       /**< 发射功率: 0dBm (0=-18dBm, 1=-12dBm, 2=-6dBm, 3=0dBm) */

/*==============================================================================
 * 宏定义 - SPI1引脚定义
 *============================================================================*/
#define NRF24L01_SPI                SPI1
#define NRF24L01_SPI_CLK            RCC_APB2Periph_SPI1
#define NRF24L01_SPI_GPIO_CLK       RCC_APB2Periph_GPIOA

#define NRF24L01_CE_PIN             GPIO_Pin_4
#define NRF24L01_CE_PORT            GPIOA
#define NRF24L01_CS_PIN             GPIO_Pin_8
#define NRF24L01_CS_PORT            GPIOA
#define NRF24L01_SCK_PIN            GPIO_Pin_5
#define NRF24L01_SCK_PORT           GPIOA
#define NRF24L01_MISO_PIN           GPIO_Pin_6
#define NRF24L01_MISO_PORT          GPIOA
#define NRF24L01_MOSI_PIN           GPIO_Pin_7
#define NRF24L01_MOSI_PORT          GPIOA
#define NRF24L01_IRQ_PIN            GPIO_Pin_0
#define NRF24L01_IRQ_PORT           GPIOB

#define NRF24L01_CE_LOW()           GPIO_ResetBits(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
#define NRF24L01_CE_HIGH()          GPIO_SetBits(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
#define NRF24L01_CS_LOW()           GPIO_ResetBits(NRF24L01_CS_PORT, NRF24L01_CS_PIN)
#define NRF24L01_CS_HIGH()          GPIO_SetBits(NRF24L01_CS_PORT, NRF24L01_CS_PIN)

/*==============================================================================
 * 宏定义 - 超时时间和队列大小
 *============================================================================*/
#define NRF24L01_TIMEOUT_MS         100     /**< 操作超时时间: 100ms */
#define NRF24L01_TX_QUEUE_SIZE      8       /**< 发送队列大小 */
#define NRF24L01_RX_QUEUE_SIZE      16      /**< 接收队列大小 */

/*==============================================================================
 * 宏定义 - 状态寄存器位
 *============================================================================*/
#define NRF24L01_STATUS_RX_DR       0x40    /**< 接收数据就绪 */
#define NRF24L01_STATUS_TX_DS       0x20    /**< 发送完成 */
#define NRF24L01_STATUS_MAX_RT      0x10    /**< 达到最大重发次数 */
#define NRF24L01_STATUS_RX_P_NO     0x0E    /**< 接收数据管道编号 */
#define NRF24L01_STATUS_TX_FULL     0x01    /**< TX FIFO满 */

/*==============================================================================
 * 宏定义 - 配置寄存器位
 *============================================================================*/
#define NRF24L01_CONFIG_PRIM_RX     0x01    /**< 接收模式 */
#define NRF24L01_CONFIG_PWR_UP      0x02    /**< 上电 */
#define NRF24L01_CONFIG_CRCO        0x04    /**< CRC编码方式(0=1字节, 1=2字节) */
#define NRF24L01_CONFIG_EN_CRC      0x08    /**< 使能CRC */
#define NRF24L01_CONFIG_MASK_MAX_RT 0x10    /**< 屏蔽MAX_RT中断 */
#define NRF24L01_CONFIG_MASK_TX_DS  0x20    /**< 屏蔽TX_DS中断 */
#define NRF24L01_CONFIG_MASK_RX_DR  0x40    /**< 屏蔽RX_DR中断 */

/*==============================================================================
 * 类型定义 - 通信协议
 *============================================================================*/
/**
 * @brief 数据包类型枚举
 */
typedef enum {
    PACKET_TYPE_CONTROL = 0x01,     /**< 控制指令包 */
    PACKET_TYPE_TELEMETRY = 0x02,   /**< 遥测数据包 */
    PACKET_TYPE_CONFIG = 0x03,      /**< 配置包 */
    PACKET_TYPE_LOG = 0x04,         /**< 日志包 */
    PACKET_TYPE_ACK = 0x05,         /**< 应答包 */
    PACKET_TYPE_HEARTBEAT = 0x06,   /**< 心跳包 */
} packet_type_t;

/**
 * @brief 控制指令结构体
 * @details 遥控器发送的控制指令
 */
typedef struct {
    uint8_t header;                 /**< 帧头: 0xAA */
    uint8_t type;                   /**< 数据类型: PACKET_TYPE_CONTROL */
    uint16_t seq;                   /**< 序列号 */
    control_mode_t mode;            /**< 控制模式 */
    float throttle;                 /**< 油门(0-100%) */
    float pitch_setpoint;           /**< 俯仰角设定值(度) */
    float roll_setpoint;            /**< 横滚角设定值(度) */
    float yaw_rate;                 /**< 偏航角速度(度/s) */
    float altitude_setpoint;        /**< 高度设定值(m) */
    uint16_t crc16;                 /**< CRC16校验 */
} control_packet_t;

/**
 * @brief 遥测数据结构体
 * @details 飞机回传的遥测数据
 */
typedef struct {
    uint8_t header;                 /**< 帧头: 0xBB */
    uint8_t type;                   /**< 数据类型: PACKET_TYPE_TELEMETRY */
    uint16_t seq;                   /**< 序列号 */
    uint32_t timestamp;             /**< 时间戳(ms) */
    control_mode_t mode;            /**< 当前模式 */
    float altitude;                 /**< 高度(m) */
    float battery_voltage;          /**< 电池电压(V) */
    float motor_rpm[4];             /**< 电机转速(RPM) */
    uint8_t status;                 /**< 状态标志 */
    uint16_t crc16;                 /**< CRC16校验 */
} telemetry_packet_t;

/**
 * @brief 配置包数据结构体
 */
typedef struct {
    uint8_t header;                 /**< 帧头: 0xCC */
    uint8_t type;                   /**< 数据类型: PACKET_TYPE_CONFIG */
    uint16_t seq;                   /**< 序列号 */
    uint8_t param_id;               /**< 参数ID */
    float param_value;              /**< 参数值 */
    uint16_t crc16;                 /**< CRC16校验 */
} config_packet_t;

/**
 * @brief NRF24L01工作状态枚举
 */
typedef enum {
    NRF24L01_STATE_IDLE = 0,        /**< 空闲状态 */
    NRF24L01_STATE_TX,              /**< 发送状态 */
    NRF24L01_STATE_RX,              /**< 接收状态 */
    NRF24L01_STATE_ERROR            /**< 错误状态 */
} nrf24l01_state_t;

/**
 * @brief NRF24L01统计信息结构体
 */
typedef struct {
    uint32_t tx_packets;            /**< 发送包数量 */
    uint32_t rx_packets;            /**< 接收包数量 */
    uint32_t tx_errors;             /**< 发送错误数 */
    uint32_t rx_errors;             /**< 接收错误数 */
    uint32_t retransmissions;       /**< 重发次数 */
    uint32_t lost_packets;          /**< 丢包数 */
    int8_t last_rssi;               /**< 上次信号强度 */
} nrf24l01_stats_t;

/*==============================================================================
 * 函数声明 - 初始化和电源管理
 *============================================================================*/
/**
 * @brief 初始化NRF24L01
 * @details 配置SPI1 GPIO和SPI参数,初始化NRF24L01寄存器
 * @return error_code_t 错误码
 *         - ERR_SUCCESS: 初始化成功
 *         - ERR_SPI_INIT_FAILED: SPI初始化失败
 *         - ERR_MUTEX_CREATE_FAILED: 创建互斥锁失败
 */
error_code_t bsp_nrf24l01_init(void);

/**
 * @brief 设置NRF24L01为接收模式
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_rx_mode(void);

/**
 * @brief 设置NRF24L01为发送模式
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_tx_mode(void);

/**
 * @brief 进入低功耗模式
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_power_down(void);

/**
 * @brief 退出低功耗模式
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_power_up(void);

/*==============================================================================
 * 函数声明 - 数据收发
 *============================================================================*/
/**
 * @brief 发送数据包
 * @param[in] data 数据指针
 * @param[in] length 数据长度(最大32字节)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_send_packet(const uint8_t *data, uint8_t length);

/**
 * @brief 接收数据包
 * @param[out] data 数据缓冲区
 * @param[out] length 接收长度
 * @param[in] timeout_ms 超时时间(ms)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_receive_packet(uint8_t *data, uint8_t *length, uint32_t timeout_ms);

/**
 * @brief 发送控制指令包
 * @param[in] packet 控制指令包指针
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_send_control(const control_packet_t *packet);

/**
 * @brief 接收控制指令包
 * @param[out] packet 控制指令包缓冲区
 * @param[in] timeout_ms 超时时间(ms)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_receive_control(control_packet_t *packet, uint32_t timeout_ms);

/**
 * @brief 发送遥测数据包
 * @param[in] packet 遥测数据包指针
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_send_telemetry(const telemetry_packet_t *packet);

/**
 * @brief 检查是否有接收数据
 * @return bool true: 有数据, false: 无数据
 */
bool bsp_nrf24l01_data_available(void);

/*==============================================================================
 * 函数声明 - 地址和通道配置
 *============================================================================*/
/**
 * @brief 设置TX地址
 * @param[in] addr 地址指针(5字节)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_tx_address(const uint8_t *addr);

/**
 * @brief 设置RX地址
 * @param[in] pipe 管道号(0-5)
 * @param[in] addr 地址指针(5字节)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_rx_address(uint8_t pipe, const uint8_t *addr);

/**
 * @brief 设置RF通道
 * @param[in] channel 通道号(0-125)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_channel(uint8_t channel);

/**
 * @brief 设置发射功率
 * @param[in] power 功率等级(0-3)
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_set_power(uint8_t power);

/*==============================================================================
 * 函数声明 - 中断和状态
 *============================================================================*/
/**
 * @brief 获取状态寄存器值
 * @return uint8_t 状态寄存器值
 */
uint8_t bsp_nrf24l01_get_status(void);

/**
 * @brief 清TX FIFO
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_flush_tx(void);

/**
 * @brief 清RX FIFO
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_flush_rx(void);

/**
 * @brief 清除中断标志
 * @param[in] flags 要清除的标志位
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_clear_irq_flags(uint8_t flags);

/**
 * @brief 获取统计信息
 * @param[out] stats 统计信息结构体指针
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_get_stats(nrf24l01_stats_t *stats);

/**
 * @brief 重置统计信息
 * @return error_code_t 错误码
 */
error_code_t bsp_nrf24l01_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_NRF24L01_H__ */
