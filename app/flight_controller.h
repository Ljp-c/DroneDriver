/**
 * @file flight_controller.h
 * @brief 飞行控制器应用层头文件
 * @details 实现飞行控制核心逻辑,支持9种控制模式切换
 *          协调传感器数据采集、PID控制、处理遥控指令和遥测数据
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "../common/system_types.h"
#include "../common/error_code.h"
#include "../src/bsp_nrf24l01.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 全局变量声明
 *============================================================================*/
/** @brief 全局系统状态 */
extern volatile system_state_t g_system_state;

/*==============================================================================
 * 宏定义 - 任务优先级
 *============================================================================*/
#define TASK_PRIORITY_CONTROL       5       /**< PID控制任务优先级(最高) */
#define TASK_PRIORITY_COMM          4       /**< 通信任务优先级 */
#define TASK_PRIORITY_SENSOR        3       /**< 传感器采集任务优先级 */
#define TASK_PRIORITY_LOG           2       /**< 日志记录任务优先级 */
#define TASK_PRIORITY_BATTERY       2       /**< 电池监控任务优先级 */

/*==============================================================================
 * 宏定义 - 任务栈大小(字) 
 *============================================================================*/
#define TASK_STACK_SIZE_CONTROL     768     /**< 控制任务栈大小 */
#define TASK_STACK_SIZE_COMM        512     /**< 通信任务栈大小 */
#define TASK_STACK_SIZE_SENSOR      512     /**< 传感器任务栈大小 */
#define TASK_STACK_SIZE_LOG         384     /**< 日志任务栈大小 */
#define TASK_STACK_SIZE_BATTERY     256     /**< 电池监控任务栈大小 */

/*==============================================================================
 * 宏定义 - 电池电压阈值
 *============================================================================*/
#define BATTERY_VOLTAGE_FULL        12.6f   /**< 满电电压(3S锂电池) */
#define BATTERY_VOLTAGE_LOW         10.5f   /**< 低电压警告阈值 */
#define BATTERY_VOLTAGE_CRITICAL    10.0f   /**< 临界电压阈值 */
/*==============================================================================
 * 类型定义 - 角度状态结构体
 *============================================================================*/
/**
 * @brief 角度状态结构体    
 * @details 用于存储姿态四元数和比例增益
 */
typedef struct {
    float q0, q1, q2, q3;      // 姿态四元数
    float twoKp;               // 比例增益（很小）
} MahonyGated;
/*==============================================================================
 * 类型定义 - 速度计算
 *============================================================================*/
/**
 * @brief 速度计算结构体
 * @details 用于存储速度积分和上次加速度值
 */
/* 状态结构体 */
typedef struct
{
    float velocity;        // 积分得到的速度
    float prev_acc;        // 上次加速度（用于梯形积分）
    bool is_first_run;     // 首次运行标志
} velocity_estimator_t;
/*==============================================================================
 * 类型定义 - 传感器状态结构体
 *============================================================================*/
/**
 * @brief 传感器状态结构体
 */
typedef struct
{
    float alpha;
    float output;
} lpf1_t;
/*==============================================================================
 * 类型定义 - 传感器状态结构体
 *============================================================================*/
/**
 * @brief 传感器状态结构体
 */
typedef struct {
    bool mpu6050_ok;                /**< MPU6050状态 */
    bool bmp250_ok;                 /**< BMP250状态 */
    bool adc_ok;                    /**< ADC状态 */
    bool pwm_ok;                    /**< PWM状态 */
    bool w25q64_ok;                 /**< W25Q64状态 */
    bool nrf24l01_ok;               /**< NRF24L01状态 */
} sensor_status_t;

/*==============================================================================
 * 类型定义 - 传感器数据结构体
 *============================================================================*/
/**
 * @brief 传感器数据结构体
 */
typedef struct {
    mpu6050_data_t accle_gyro;        /**< MPU6050数据 */
    bmp250_data_t bmp250;           /**< BMP250数据 */
} sensor_data_t;

/*==============================================================================
 * 类型定义 - 姿态控制结构体
 *============================================================================*/
/**
 * @brief 控制输出结构体
 */
typedef struct {
    float pitch;                /**< 俯仰输出 */
    float roll;                 /**< 横滚输出 */
    float yaw;                  /**< 偏航输出 */
} attitude_t;
/*==============================================================================
 * 类型定义 - 控制输出结构体
 *============================================================================*/
/**
 * @brief 控制输出结构体
 */
typedef struct {
    attitude_t attitude;            /**< 姿态输出 */
    float altitude_out;             /**< 高度输出 */
    pwm_output_t motor_pwm;         /**< 电机PWM输出(0-100%) */
} control_output_t;

/*==============================================================================
 * 类型定义 - 飞行控制器主结构体
 *============================================================================*/
/**
 * @brief 飞行控制器主数据结构体
 */
typedef struct {
    control_mode_t current_mode;    /**< 当前控制模式 */
    system_state_t system_state;    /**< 系统状态 */
    sensor_status_t sensor_status;  /**< 传感器状态 */
    
    /* 控制目标值 */

    float target_yaw_rate;          /**< 目标偏航角速度(度/s) */
    float target_altitude;          /**< 目标高度(m) */
    
    /* 控制状态 */
    float throttle;                 /**< 油门(0-100%) */
    bool armed;                     /**< 解锁标志 */
    bool flying;                    /**< 飞行标志 */
    
    /* 系统信息 */
    battery_data_t battery_sign;    /**< 电池电压(V) */
    bool battery_low;               /**< 低电压标志 */
    uint32_t takeoff_time_ms;       /**< 起飞时间(ms) */
    uint32_t last_cmd_time_ms;      /**< 最后指令时间(ms) */
} flight_controller_t;
/*==============================================================================
 * 辅助函数 - 低通滤波
 *============================================================================*/
/**
 * @brief 更新低通滤波输出
 * @return float 滤波输出
 */
static inline float lpf1_update(lpf1_t *f, float input);
/*==============================================================================
 * 辅助函数 - 姿态更新
 *============================================================================*/
/**
 * @brief 更新姿态(互补滤波)
 * @return error_code_t 错误码
 */
void MahonyGated_Init(MahonyGated *s);

static inline float fast_rsqrt(float x);

void MahonyGated_UpdateIMU(MahonyGated *s,float *array,float dt);

static inline float fast_atan2(float y, float x);

void Quaternion_ToEuler(const MahonyGated *s,
                        attitude_t *attitude);
/*==============================================================================
 * 函数声明 - 初始化函数
 *============================================================================*/
/**
 * @brief 初始化飞行控制器
 * @details 初始化PID控制器和飞行控制状态
 * @return error_code_t 错误码
 */
error_code_t flight_controller_init(void);

/**
 * @brief 启动飞行控制器
 * @details 初始化所有硬件模块并创建FreeRTOS任务
 * @return error_code_t 错误码
 */
error_code_t flight_controller_start(void);

/*==============================================================================
 * 函数声明 - 飞行控制
 *============================================================================*/
/**
 * @brief 解锁飞行控制器
 * @details 检查传感器和电池状态,启动电机输出
 * @return error_code_t 错误码
 */
error_code_t flight_controller_arm(void);

/**
 * @brief 锁定飞行控制器
 * @details 停止电机输出并进入待机
 * @return error_code_t 错误码
 */
error_code_t flight_controller_disarm(void);

/**
 * @brief 起飞
 * @details 从当前位置起飞到指定高度
 * @param[in] target_altitude 目标高度(m)
 * @return error_code_t 错误码
 */
error_code_t flight_controller_takeoff(float target_altitude);

/**
 * @brief 降落
 * @details 控制飞机缓慢下降并自动锁定
 * @return error_code_t 错误码
 */
error_code_t flight_controller_land(void);

/**
 * @brief 设置控制模式
 * @param[in] mode 控制模式
 * @return error_code_t 错误码
 */
error_code_t flight_controller_set_mode(control_mode_t mode);

/**
 * @brief 紧急停止
 * @details 紧急停止所有电机输出
 * @return error_code_t 错误码
 */
error_code_t flight_controller_emergency_stop(void);

/*==============================================================================
 * 函数声明 - 状态获取
 *============================================================================*/
/**
 * @brief 获取系统状态
 * @return system_state_t 系统状态
 */
system_state_t flight_controller_get_state(void);

/**
 * @brief 获取当前姿态
 * @param[out] attitude 姿态数据结构体指针
 * @return error_code_t 错误码
 */
error_code_t flight_controller_get_attitude(attitude_t *attitude);

/**
 * @brief 获取传感器数据
 * @param[out] data 传感器数据结构体指针
 * @return error_code_t 错误码
 */
error_code_t flight_controller_get_sensor_data(sensor_data_t *data);

/**
 * @brief 获取控制输出
 * @param[out] output 控制输出结构体指针
 * @return error_code_t 错误码
 */
error_code_t flight_controller_get_control_output(control_output_t *output);

/**
 * @brief 获取飞行时间
 * @return uint32_t 飞行时间(ms)
 */
uint32_t flight_controller_get_flight_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_CONTROLLER_H__ */
