/**
 * @file system_types.h
 * @brief 系统全局类型定义
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 此文件定义系统全局使用的数据类型和结构体
 * 仅允许定义全局数组Global_message，其他变量均本地化
 */

#ifndef __SYSTEM_TYPES_H
#define __SYSTEM_TYPES_H

#include <stdint.h>
#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 控制模式枚举 ===================== */
/**
 * @brief 飞行控制模式枚举
 * 共9种控制模式，对应不同的PID控制策略
 */
typedef enum {
    CTRL_MODE_NONE = 0,                 /* 无控制模式 */
    CTRL_MODE_UP_ACCEL_CONST = 1,       /* 向上恒定加速度飞行 */
    CTRL_MODE_UP_SPEED_CONST = 2,       /* 向上匀速飞行 */
    CTRL_MODE_DOWN_ACCEL_CONST = 3,     /* 向下恒定加速度飞行 */
    CTRL_MODE_DOWN_SPEED_CONST = 4,     /* 向下匀速飞行 */
    CTRL_MODE_HOVER = 5,                /* 浮空悬停 */
    CTRL_MODE_X_SPEED_CONST = 6,        /* X轴水平匀速飞行 */
    CTRL_MODE_X_ACCEL_CONST = 7,        /* X轴水平加速飞行 */
    CTRL_MODE_Y_SPEED_CONST = 8,        /* Y轴水平匀速飞行 */
    CTRL_MODE_Y_ACCEL_CONST = 9,        /* Y轴水平加速飞行 */
    CTRL_MODE_MAX = 10
} control_mode_t;

/* ===================== 传感器数据结构 ===================== */
/**
 * @brief MPU6050加速度数据结构
 * 单位：m/s²
 */
 
typedef struct {
    float acc_x;        /* X轴加速度，向上为正 */
    float acc_y;        /* Y轴加速度 */
    float acc_z;        /* Z轴加速度 */
    float gyro_x;       /* X轴角速度 */
    float gyro_y;       /* Y轴角速度 */
    float gyro_z;       /* Z轴角速度 */
    uint8_t valid;      /* 数据有效标志 */
} mpu6050_data_t;

/**
 * @brief BMP250高度数据结构
 * 单位：米(m)，帕斯卡(Pa)
 */
typedef struct {
    float altitude;     /* 对地高度 */
    float pressure;     /* 气压值 */
    float temperature;  /* 温度 */
    uint8_t valid;      /* 数据有效标志 */
} bmp250_data_t;

/**
 * @brief 电池电压数据结构
 * 单位：伏特(V)
 */
typedef struct {
    float voltage;      /* 电池电压 */
    uint8_t valid;      /* 数据有效标志 */
} battery_data_t;

/* ===================== 系统状态结构 ===================== */
/**
 * @brief 系统运行状态
 */
typedef enum {
    SYS_STATE_INIT = 0,         /* 初始化状态 */
    SYS_STATE_READY,            /* 就绪状态 */
    SYS_STATE_RUNNING,          /* 运行状态 */
    SYS_STATE_ERROR,            /* 错误状态 */
    SYS_STATE_EMERGENCY         /* 紧急状态 */
} system_state_t;

/**
 * @brief 系统状态数据结构
 */
typedef struct {
    system_state_t state;           /* 当前系统状态 */
    control_mode_t control_mode;    /* 当前控制模式 */
    uint32_t error_code;            /* 错误码 */
    uint32_t runtime_seconds;       /* 运行时间(秒) */
    uint8_t initialized;            /* 初始化完成标志 */
} system_status_t;

/* ===================== 控制数据结构 ===================== */
/**
 * @brief PID控制输出结构
 * 占空比范围：0-100%，对应0-5V输出
 */
typedef struct {
    float duty_ch1;     /* PA0通道占空比 */
    float duty_ch2;     /* PA1通道占空比 */
    float duty_ch3;     /* PA2通道占空比 */
    float duty_ch4;     /* PA3通道占空比 */
    uint8_t valid;      /* 输出有效标志 */
} pwm_output_t;

/**
 * @brief 遥控指令结构
 */
typedef struct {
    control_mode_t mode;        /* 控制模式 */
    float target_param1;        /* 目标参数1（速度/加速度/高度） */
    float target_param2;        /* 目标参数2 */
    uint8_t valid;              /* 指令有效标志 */
    uint32_t timestamp;         /* 时间戳 */
} remote_command_t;

/**
 * @brief 遥测数据结构
 */
typedef struct {
    mpu6050_data_t mpu6050;        /**< MPU6050数据 */
    bmp250_data_t bmp250;           /**< BMP250数据 */
    battery_data_t battery;         /**< 电池电压数据 */
    uint32_t error_code;            /**< 错误码 */
    system_state_t state;           /**< 系统状态 */    
} telemetry_data_t;

/* ===================== 全局数组定义 ===================== */
/**
 * @brief 全局核心数据数组
 * 用途：存储角度、距离相关核心数据，供各模块接口调用
 * 索引定义：
 * [0]: 俯仰角 (pitch)
 * [1]: 横滚角 (roll)
 * [2]: 偏航角 (yaw)
 * [3]: 对地高度
 * [4]: X轴水平距离
 * [5]: Y轴水平距离
 * [6-9]: 预留
 */
extern float Global_message[14];

/* ===================== 常量定义 ===================== */
/* 物理常量 */
#define GRAVITY_ACCEL           9.80665f    /* 重力加速度 m/s² */
#define SEA_LEVEL_PRESSURE      101325.0f   /* 海平面标准气压 Pa */

/* PWM参数 */
#define PWM_MIN_DUTY            0.0f        /* 最小占空比 */
#define PWM_MAX_DUTY            100.0f      /* 最大占空比 */
#define PWM_VOLTAGE_MIN         0.0f        /* 最小输出电压 V */
#define PWM_VOLTAGE_MAX         5.0f        /* 最大输出电压 V */

/* 控制参数 */
#define TARGET_UP_ACCEL         0.05f       /* 向上恒定加速度 m/s² */
#define TARGET_DOWN_ACCEL       (-0.05f)    /* 向下恒定加速度 m/s² */
#define TARGET_HORIZONTAL_ACCEL 0.05f       /* 水平恒定加速度 m/s² */
#define TARGET_UP_SPEED         1.0f        /* 向上目标速度 m/s */
#define TARGET_DOWN_SPEED       (-1.0f)     /* 向下目标速度 m/s */
#define TARGET_HORIZONTAL_SPEED 0.5f        /* 水平目标速度 m/s */

/* 时间参数 */
#define LOG_INTERVAL_SECONDS    120         /* 日志记录间隔：2分钟 */
#define TELEMETRY_INTERVAL_MS   100         /* 遥测发送间隔：100ms */
#define PID_CONTROL_INTERVAL_MS 10          /* PID控制间隔：10ms */

/* 电压参数 */
#define BATTERY_VOLTAGE_MIN     3.3f        /* 电池最低电压 V */
#define BATTERY_VOLTAGE_MAX     4.2f        /* 电池最高电压 V */

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_TYPES_H */
