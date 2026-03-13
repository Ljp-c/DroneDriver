/**
 * @file pid_controller.h
 * @brief PID控制器头文件
 * @details 提供9种不同控制模式的PID算法实现
 * @author 
 * @date 2026-02-08
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "stm32f10x.h"
#include "system_types.h"
#include "error_code.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 宏定义
 *============================================================================*/
#define PID_OUTPUT_MIN              (-100.0f)   /**< PID输出最小值 */
#define PID_OUTPUT_MAX              100.0f      /**< PID输出最大值 */
#define PID_INTEGRAL_MIN            (-50.0f)    /**< 积分限幅最小值 */
#define PID_INTEGRAL_MAX            50.0f       /**< 积分限幅最大值 */
#define PID_DT                      0.01f       /**< 控制周期 10ms */
#define CTRL_MODE_MAX               9           /**< 最大控制模式数量 */
/*==============================================================================
 * 类型定义
 *============================================================================*/
typedef struct
{
    float kp;
    float ki;
    float kd;

    float i_limit;     // 积分限幅
    float out_limit;   // 输出限幅
    float d_alpha;     // D滤波系数
} pid_param_t;

typedef struct
{
    float d_lpf;
    float integral;
    float last_error;
    float prev_input; /* 用于微分滤波 */
} pid_state_t;

/*==============================================================================
 * 函数声明 - 初始化
 *============================================================================*/
error_code_t pid_controller_init(void);
error_code_t pid_reset_all(void);


/*==============================================================================
 * 函数声明 - 内环PID控制器
 *============================================================================*/
static float innner_ring(float a_target, float dt, globedata index);
static float outer_ring(float v_target, globedata index, float dt)
/*==============================================================================
 * 函数声明 - 9种不同PID模式
 *============================================================================*/

/* 1. 向上恒定加速度飞行 */
error_code_t pid_mode_up_accel_const(float current_acc_x, float current_height, float *pwm_out);

/* 2. 向上匀速飞行 */
error_code_t pid_mode_up_speed_const(float current_speed, float current_height, float *pwm_out);

/* 3. 向下恒定加速度飞行 */
error_code_t pid_mode_down_accel_const(float current_acc_x, float current_height, float *pwm_out);

/* 4. 向下匀速飞行 */
error_code_t pid_mode_down_speed_const(float current_speed, float current_height, float *pwm_out);

/* 5. 悬停 */
error_code_t pid_mode_hover(float target_height, float current_height, float acc_z, float *pwm_out);

/* 6. X轴水平匀速飞行 */
error_code_t pid_mode_x_speed_const(float current_speed_x, float pitch_angle, float *pwm_out);

/* 7. X轴水平加速飞行 */
error_code_t pid_mode_x_accel_const(float current_acc_x, float pitch_angle, float *pwm_out);

/* 8. Y轴水平匀速飞行 */
error_code_t pid_mode_y_speed_const(float current_speed_y, float roll_angle, float *pwm_out);

/* 9. Y轴水平加速飞行 */
error_code_t pid_mode_y_accel_const(float current_acc_y, float roll_angle, float *pwm_out);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H__ */
