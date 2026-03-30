/**
 * @file global_data.c
 * @brief 全局数据定义
 * @version 2.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 全局飞行状态数据定义和管理
 */

#include "global_data.h"
#include "FreeRTOS.h"
#include "semphr.h"

/*==============================================================================
 * 全局变量定义
 *============================================================================*/

/** @brief 飞行状态数据结构 */
flight_state_t g_flight_state = {
    .pitch = 0.0f,
    .roll = 0.0f,
    .yaw = 0.0f,
    .altitude = 0.0f,
    .height = 0.0f,
    .velocity_x = 0.0f,
    .velocity_y = 0.0f,
    .velocity_z = 0.0f,
    .accel_x = 0.0f,
    .accel_y = 0.0f,
    .accel_z = 0.0f,
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .temperature = 25.0f,
    .pressure = 101325.0f,
    .gravity = 9.8f,
    .timestamp = 0,
    .mpu_valid = 0,
    .bmp_valid = 0,
    .battery_valid = 0
};

/** @brief 飞行状态互斥锁 */
SemaphoreHandle_t g_flight_state_mutex = NULL;

/*==============================================================================
 * 兼容性支持：全局数组和变量（用于过渡期）
 *============================================================================*/

/** @brief 全局核心数据数组（兼容旧代码） */
float Global_message[15] = {0.0f};

/** @brief 全局索引变量 */
globedata globedata_index = ALTITUDE_INDEX;
globedata globedata_vel = Z_ACCLE_INDEX;
globedata globedata_acc = Z_ACCLE_INDEX;

/*==============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief 初始化全局数据管理系统
 * @details 创建互斥锁并初始化数据结构
 * @return 错误码（ERR_OK表示成功）
 */
int global_data_init(void)
{
    /* 创建互斥锁 */
    g_flight_state_mutex = xSemaphoreCreateMutex();
    if (g_flight_state_mutex == NULL) {
        return -1;  /* 内存分配失败 */
    }
    
    /* 初始化飞行状态 */
    g_flight_state.gravity = 9.8f;
    g_flight_state.timestamp = 0;
    g_flight_state.mpu_valid = 0;
    g_flight_state.bmp_valid = 0;
    
    return 0;  /* 成功 */
}

/**
 * @brief 原子性地读取飞行状态
 * @param state_copy 用于接收数据的指针
 * @return 错误码
 * @note 调用此函数会复制整个飞行状态，可能需要10ms左右的时间
 */
int get_flight_state(flight_state_t *state_copy)
{
    if (state_copy == NULL) {
        return -1;  /* 空指针 */
    }
    
    if (g_flight_state_mutex == NULL) {
        return -2;  /* 互斥锁未初始化 */
    }
    
    /* 获取互斥锁，最多等待10ms */
    if (xSemaphoreTake(g_flight_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -3;  /* 超时 */
    }
    
    /* 原子复制 */
    *state_copy = g_flight_state;
    
    /* 释放互斥锁 */
    xSemaphoreGive(g_flight_state_mutex);
    
    return 0;  /* 成功 */
}

/**
 * @brief 原子性地写入飞行状态
 * @param new_state 新的飞行状态
 * @return 错误码
 */
int set_flight_state(const flight_state_t *new_state)
{
    if (new_state == NULL) {
        return -1;  /* 空指针 */
    }
    
    if (g_flight_state_mutex == NULL) {
        return -2;  /* 互斥锁未初始化 */
    }
    
    /* 获取互斥锁，最多等待10ms */
    if (xSemaphoreTake(g_flight_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -3;  /* 超时 */
    }
    
    /* 原子复制 */
    g_flight_state = *new_state;
    
    /* 释放互斥锁 */
    xSemaphoreGive(g_flight_state_mutex);
    
    return 0;  /* 成功 */
}

/**
 * @brief 更新飞行状态的单个字段（姿态角）
 * @param pitch 俯仰角（度）
 * @param roll 横滚角（度）
 * @param yaw 偏航角（度）
 * @return 错误码
 */
int update_flight_attitude(float pitch, float roll, float yaw)
{
    if (g_flight_state_mutex == NULL) return -2;
    
    if (xSemaphoreTake(g_flight_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -3;
    }
    
    g_flight_state.pitch = pitch;
    g_flight_state.roll = roll;
    g_flight_state.yaw = yaw;
    
    xSemaphoreGive(g_flight_state_mutex);
    return 0;
}

/**
 * @brief 更新飞行状态的单个字段（加速度）
 * @param x X轴加速度
 * @param y Y轴加速度
 * @param z Z轴加速度
 * @return 错误码
 */
int update_flight_accel(float x, float y, float z)
{
    if (g_flight_state_mutex == NULL) return -2;
    
    if (xSemaphoreTake(g_flight_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -3;
    }
    
    g_flight_state.accel_x = x;
    g_flight_state.accel_y = y;
    g_flight_state.accel_z = z;
    
    xSemaphoreGive(g_flight_state_mutex);
    return 0;
}

/**
 * @brief 更新飞行状态的单个字段（角速度）
 * @param x X轴角速度
 * @param y Y轴角速度
 * @param z Z轴角速度
 * @return 错误码
 */
int update_flight_gyro(float x, float y, float z)
{
    if (g_flight_state_mutex == NULL) return -2;
    
    if (xSemaphoreTake(g_flight_state_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return -3;
    }
    
    g_flight_state.gyro_x = x;
    g_flight_state.gyro_y = y;
    g_flight_state.gyro_z = z;
    
    xSemaphoreGive(g_flight_state_mutex);
    return 0;
}
