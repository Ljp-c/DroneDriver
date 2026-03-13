/**
 * @file global_data.h
 * @brief 全局数据声明
 * @details 声明全局数组Global_message,用于存储角度和距离核心数据
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __GLOBAL_DATA_H__
#define __GLOBAL_DATA_H__




typedef enum {
    PITCH_INDEX = 0,
    ROLL_INDEX,
    PRESSURE,
    ALTITUDE_INDEX,
    X_ACCLE_INDEX,
    Y_ACCLE_INDEX,
    Z_ACCLE_INDEX,
    X_GYRO_INDEX,
    Y_GYRO_INDEX,
    Z_GYRO_INDEX,
    TEMPERATURE_INDEX,

    GRAVITY_INDEX=14 /**< 重力加速度索引 */
}  globedata;

/**
 * @brief 全局核心数据数组
 * @details 存储角度、距离相关核心数据,供各模块接口调用
 * 
 * 索引定义:
 * - [0]: 俯仰角 (pitch, 单位: °)
 * - [1]: 横滚角 (roll, 单位: °)
 * - [2]: 偏航角 (yaw, 单位: °)
 * - [3]: 对地高度 (单位: m)
 * - [4]: X轴加速度 (单位: m)
 * - [5]: Y轴加速度 (单位: m)
 * - [6]: Z轴加速度 (单位: m/s²)
 * - [7]: X轴角加速度 (单位: m/s²)
 * - [8]: Y轴角加速度 (单位: m/s²)
 * - [9]: Z轴角加速度 (单位: m/s²)
 */
extern float Global_message[15]; /**< 全局核心数据数组 */
/**
 * @brief 全局核心数据索引
 * @details 定义全局核心数据数组的索引,方便模块调用
 */
extern globedata globedata_index; /**< 全局核心数据索引 */

#endif /* __GLOBAL_DATA_H__ */
