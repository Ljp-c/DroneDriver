/**
 * @file global_data.h
 * @brief 全局数据声明
 * @details 定义飞行状态全局数据结构，并通过互斥锁保护并发访问
 * @author 
 * @date 2026
 * @version 2.0.0
 * @copyright MIT License
 */

#ifndef __GLOBAL_DATA_H__
#define __GLOBAL_DATA_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

/*==============================================================================
 * 飞行状态数据结构
 *============================================================================*/

/**
 * @brief 飞行状态数据结构体
 * @details 包含所有飞控系统需要的传感器数据和计算结果
 */
typedef struct {
    /* === 姿态角（欧拉角，单位：度） === */
    float pitch;        /**< 俯仰角 (Pitch) */
    float roll;         /**< 横滚角 (Roll) */
    float yaw;          /**< 偏航角 (Yaw) */
    
    /* === 位置和高度（单位：米） === */
    float altitude;     /**< 对地绝对高度 (m) */
    float height;       /**< 相对高度 (m) */
    
    /* === 速度（单位：m/s） === */
    float velocity_x;   /**< X轴速度 */
    float velocity_y;   /**< Y轴速度 */
    float velocity_z;   /**< Z轴速度（竖直向上为正） */
    
    /* === 加速度（单位：m/s²） === */
    float accel_x;      /**< X轴加速度 */
    float accel_y;      /**< Y轴加速度 */
    float accel_z;      /**< Z轴加速度 */
    
    /* === 角速度（单位：rad/s） === */
    float gyro_x;       /**< X轴角速度 */
    float gyro_y;       /**< Y轴角速度 */
    float gyro_z;       /**< Z轴角速度 */
    
    /* === 环境数据 === */
    float temperature;  /**< 温度（℃） */
    float pressure;     /**< 气压（Pa） */
    float gravity;      /**< 重力加速度（m/s²），标准值9.8 */
    
    /* === 数据有效性和时间戳 === */
    uint32_t timestamp; /**< 数据时间戳（ms） */
    uint8_t mpu_valid;  /**< MPU6050数据有效标志（1=有效，0=无效） */
    uint8_t bmp_valid;  /**< BMP280数据有效标志 */
    uint8_t battery_valid; /**< 电池数据有效标志 */
    
} flight_state_t;

/*==============================================================================
 * 兼容性支持（用于过渡期）
 *============================================================================*/

/* 用于兼容旧代码中使用Global_message[index]的方式 */
typedef enum {
    PITCH_INDEX = 0,          /**< 俯仰角索引 */
    ROLL_INDEX,               /**< 横滚角索引 */
    PRESSURE_INDEX = 2,       /**< 气压索引 */
    ALTITUDE_INDEX,           /**< 对地高度索引 */
    X_ACCLE_INDEX,            /**< X加速度索引 */
    Y_ACCLE_INDEX,            /**< Y加速度索引 */
    Z_ACCLE_INDEX,            /**< Z加速度索引 */
    X_GYRO_INDEX,             /**< X角速度索引 */
    Y_GYRO_INDEX,             /**< Y角速度索引 */
    Z_GYRO_INDEX,             /**< Z角速度索引 */
    TEMPERATURE_INDEX,        /**< 温度索引 */
    
    GRAVITY_INDEX = 14        /**< 重力加速度索引 */
} globedata;

/*==============================================================================
 * 全局数据声明
 *============================================================================*/

/** @brief 全局飞行状态 */
extern flight_state_t g_flight_state;

/** @brief 飞行状态的互斥锁（保护共享访问） */
extern SemaphoreHandle_t g_flight_state_mutex;

/* 兼容旧代码的全局数组宏 */
extern float Global_message[15];
extern globedata globedata_index;
extern globedata globedata_vel;
extern globedata globedata_acc;

#endif /* __GLOBAL_DATA_H__ */
