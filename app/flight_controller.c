/**
 * @file flight_controller.c
 * @brief 飞行控制器主逻辑实现
 * @details 实现飞行控制主循环、FreeRTOS任务创建和系统调度
 *          包含传感器融合、PID控制、电机混控、通信处理等功能
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#include "flight_controller.h"
#include "pid_controller.h"
#include "bsp_mpu6050.h"
#include "bsp_bmp250.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_w25q64.h"
#include "bsp_nrf24l01.h"
#include "global_data.h"
#include "stm32f10x_iwdg.h"

/*==============================================================================
 * 私有变量
 *============================================================================*/
static flight_controller_t s_flight_controller;     /**< 飞行控制器实例 */
static TaskHandle_t s_sensor_task_handle = NULL;    /**< 传感器任务句柄 */
static TaskHandle_t s_control_task_handle = NULL;   /**< 控制任务句柄 */
static TaskHandle_t s_comm_task_handle = NULL;      /**< 通信任务句柄 */
static TaskHandle_t s_log_task_handle = NULL;       /**< 日志任务句柄 */
static TaskHandle_t s_battery_task_handle = NULL;   /**< 电池监测任务句柄 */

/* 传感器数据 */
static sensor_data_t s_sensor_data;                 /**< 传感器数据 */
static attitude_t s_attitude={0};                       /**< 姿态数据 */
static control_output_t s_control_output={0};
/* 控制指令 */
static velocity_estimator_t s_velocity_estimator = {0}; /**< 速度估算器 */
static control_packet_t s_control_cmd;              /**< 控制指令 */
static volatile bool s_new_control_cmd = false;     /**< 新控制指令标志 */

/* 系统状态 */
static system_state_t s_system_state = SYS_STATE_INIT;   /**< 系统状态 */
volatile system_state_t g_system_state = SYS_STATE_INIT; /**< 全局系统状态 */
static uint32_t s_system_tick = 0;                  /**< 系统运行时间 */
static uint32_t s_control_seq = 0;                  /**< 控制序列号 */

/* 外部全局变量 */
extern float batteryVotlge;                         /**< 电池电压(来自bsp_adc.c) */
/* 过滤器参数 */
static lpf1_t s_temperature_filter = {0.1f, 0.0f};    /**< 温度数据低通滤波器 */    
static lpf1_t s_altitude_filter = {0.1116, 0.0f};    /**< 海拔数据低通滤波器 */     
static lpf1_t s_pressure_filter = {0.0609f, 0.0f};    /**< 压力数据低通滤波器 */        
static lpf1_t s_accle_filter = {0.5571f, 0.0f};    /**< 加速度数据低通滤波器 */    
static lpf1_t s_gyro_filter = {0.2391f, 0.0f};    /**< 陀螺仪数据低通滤波器 */    
/*==============================================================================
 * 私有函数声明
 *============================================================================*/
static void sensor_task(void *pv_parameters);
static void control_task(void *pv_parameters);
static void comm_task(void *pv_parameters);
static void log_task(void *pv_parameters);
static void battery_task(void *pv_parameters);
static error_code_t update_attitude(void);
static error_code_t run_control_loop(void);
static error_code_t motor_mixing(float throttle, float pitch_out, float roll_out, float yaw_out);
static error_code_t process_control_command(const control_packet_t *cmd);
static error_code_t send_telemetry(void);
static error_code_t log_flight_data(void);
/*==============================================================================
 * 任务实现 - 针对传感器数据滤波
 *============================================================================*/
/**
 * @brief 低通滤波器更新
 * @details 对输入信号进行低通滤波处理，输出滤波后的信号
 * @param[in] f 低通滤波器结构体指针
 * @param[in] input 输入信号值
 * @return 滤波后的信号值
 */
static inline float lpf1_update(lpf1_t *f, float input)
{
    f->output += f->alpha * (input - f->output);
    return f->output;
}
/*==============================================================================
 * 任务实现 - 速度计算
 *============================================================================*/
/**
 * @brief 从加速度积分得到速度
 * @param acc_measure  测量的加速度
 * @param dt           时间间隔
 * @return 估算的速度
 */
static float get_velocity_from_acceleration(float acc_measure, float dt)
{
    static velocity_estimator_t est = {0};

    if (dt <= 1e-6f || dt > 0.1f)
        return est.velocity;

    if (est.is_first_run)
    {
        est.prev_acc = acc_measure;
        est.velocity = 0.0f;
        est.is_first_run = false;
        return 0.0f;
    }

    /* 梯形积分（更精确） */
    est.velocity += (est.prev_acc + acc_measure) * 0.5f * dt;
    est.prev_acc = acc_measure;

    return est.velocity;
}


/*==============================================================================
 * 辅助函数 - 姿态更新
 *============================================================================*/
/**
 * @brief 更新姿态(互补滤波)
 * @return error_code_t 错误码
 */

 //进行飞控推荐参数初始化
void MahonyGated_Init(void)
{
    s->q0 = 1.0f;
    s->q1 = 0.0f;
    s->q2 = 0.0f;
    s->q3 = 0.0f;

    s->twoKp = 0.2f;   // 飞控推荐 0.1 ~ 0.3
}
//工具函数 进行快速平方根计算
static inline float fast_rsqrt(float x)
{
    union {
        float f;
        uint32_t i;
    } conv;

    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
    return conv.f;
}
//快速atan2近似
static inline float fast_atan2(float y, float x)
{
    const float ONEQTR_PI = 0.78539816339f;
    const float THRQTR_PI = 2.35619449019f;

    float abs_y = y > 0 ? y : -y;
    float r, angle;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = ONEQTR_PI - ONEQTR_PI * r;
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);
        angle = THRQTR_PI - ONEQTR_PI * r;
    }

    return (y < 0.0f) ? -angle : angle;
}
void MahonyGated_UpdateIMU(float dt)
{


    float ax =Global_message[X_ACCLE_INDEX];
    float ay =Global_message[Y_ACCLE_INDEX];
    float az =Global_message[Z_ACCLE_INDEX];

    float gx =Global_message[X_GYRO_INDEX];
    float gy =Global_message[Y_GYRO_INDEX];
    float gz =Global_message[Z_GYRO_INDEX];

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

    /* ========== 1. 加速度门控判断 ========== */
    float acc_norm2 = ax*ax + ay*ay + az*az;

    int accel_valid = 0;
    if (acc_norm2 > 0.7f && acc_norm2 < 1.3f) {
        accel_valid = 1;
    }

    /* ========== 2. 如果加速度可信，计算修正误差 ========== */
    if (accel_valid) {
        recipNorm = fast_rsqrt(acc_norm2);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向（由当前姿态）
        halfvx = s->q1 * s->q3 - s->q0 * s->q2;
        halfvy = s->q0 * s->q1 + s->q2 * s->q3;
        halfvz = s->q0 * s->q0 - 0.5f + s->q3 * s->q3;

        // 误差 = 测量重力 × 估计重力
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 反馈修正到陀螺
        gx += s->twoKp * halfex;
        gy += s->twoKp * halfey;
        gz += s->twoKp * halfez;
    }

    /* ========== 3. 四元数积分（始终执行） ========== */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = s->q0;
    float qb = s->q1;
    float qc = s->q2;

    s->q0 += (-qb * gx - qc * gy - s->q3 * gz);
    s->q1 += ( qa * gx + qc * gz - s->q3 * gy);
    s->q2 += ( qa * gy - qb * gz + s->q3 * gx);
    s->q3 += ( qa * gz + qb * gy - qc * gx);

    /* ========== 4. 四元数归一化 ========== */
    recipNorm = fast_rsqrt(s->q0*s->q0 + s->q1*s->q1 +
                           s->q2*s->q2 + s->q3*s->q3);

    s->q0 *= recipNorm;
    s->q1 *= recipNorm;
    s->q2 *= recipNorm;
    s->q3 *= recipNorm;
}

//四元数得到 roll / pitch 角度
void Quaternion_ToEuler(const MahonyGated *s,attitude_t *s_attitude)
{
    s_attitude->roll  = fast_atan2(2.0f*(s->q0*s->q1 + s->q2*s->q3),
                    1.0f - 2.0f*(s->q1*s->q1 + s->q2*s->q2));

    s_attitude->pitch = fast_asin(2.0f*(s->q0*s->q2 - s->q3*s->q1));    
    
    s_attitude->yaw = fast_atan2(2.0f*(s->q0*s->q3 + s->q1*s->q2),
                    1.0f - 2.0f*(s->q2*s->q2 + s->q3*s->q3));
}
/*==============================================================================
 * 任务实现 - 传感器任务
 *============================================================================*/
/**
 * @brief 传感器采集任务
 * @details 以100Hz频率采集MPU6050和BMP250传感器数据
 * @param[in] pv_parameters 任务参数
 */
static void sensor_task(void *pv_parameters)
{
    (void)pv_parameters;
    /* 等待传感器就绪 */
    vTaskDelay(pdMS_TO_TICKS(100));    


    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10);  /* 100Hz */
    
    while (1) {
        /* 等待下一个周期 */
        vTaskDelayUntil(&last_wake_time, frequency);
        
        /* 读取MPU6050数据 */
        error_code_t err = bsp_mpu6050_read_all( &s_sensor_data.accle_gyro);
        if (err == ERR_SUCCESS) {
            s_flight_controller.sensor_status.mpu6050_ok = true;
        } else {
            s_flight_controller.sensor_status.mpu6050_ok = false;
        }
        
        /* 读取BMP250数据 */
        err = bsp_bmp250_read_all(&s_sensor_data.bmp250);
        if (err == ERR_SUCCESS) {
            s_flight_controller.sensor_status.bmp250_ok = true;
        } else {
            s_flight_controller.sensor_status.bmp250_ok = false;
        }
        
        /* 更新姿态 */
        if (s_flight_controller.sensor_status.mpu6050_ok) {
            update_attitude();
        }
        
        /* 更新全局数据数组 */
       Global_message[0] =s_attitude.pitch;        /**< 俯仰角 */
       Global_message[1] =s_attitude.roll;         /**< 横滚角 */
       Global_message[2] = lpf1_update(&s_pressure_filter, s_sensor_data.bmp250.pressure);  
       Global_message[3] =lpf1_update(&s_altitude_filter, s_sensor_data.bmp250.altitude);  /**< 对地高度 */
       Global_message[4] =lpf1_update(&s_accle_filter, s_sensor_data.accle_gyro.acc_x);   /**< X轴加速度 */
       Global_message[5] =lpf1_update(&s_accle_filter, s_sensor_data.accle_gyro.acc_y);   /**< Y轴加速度 */
       Global_message[6] =lpf1_update(&s_accle_filter, s_sensor_data.accle_gyro.acc_z);   /**< Z轴加速度 */
       Global_message[7] =lpf1_update(&s_gyro_filter, s_sensor_data.accle_gyro.gyro_x);    /**< X轴角速度 */
       Global_message[8] =lpf1_update(&s_gyro_filter, s_sensor_data.accle_gyro.gyro_y);    /**< Y轴角速度 */
       Global_message[9] =lpf1_update(&s_gyro_filter, s_sensor_data.accle_gyro.gyro_z);    /**< Z轴角速度 */
       Global_message[10]=lpf1_update(&s_temperature_filter, s_sensor_data.bmp250.temperature); /**< 温度 */   
       Global_message[11]=get_velocity_from_acceleration(Global_message[4], frequency); /**< 速度 */
       Global_message[12]=get_velocity_from_acceleration(Global_message[5], frequency); /**< 速度 */
       Global_message[13]=get_velocity_from_acceleration(Global_message[6], frequency); /**< 速度 */
       Global_message[14]=9.8f; /**< 重力加速度 */
    }
}

/*==============================================================================
 * 任务实现 - 控制任务
 *============================================================================*/
/**
 * @brief 飞行控制任务
 * @details 以100Hz频率执行PID控制算法,优先级最高
 * @param[in] pv_parameters 任务参数
 */
static void control_task(void *pv_parameters)
{
    (void)pv_parameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10);  /* 100Hz */
    
    while (1) {
        /* 等待下一个周期 */
        vTaskDelayUntil(&last_wake_time, frequency);
        
        /* 检查系统状态 */
        if (s_system_state != SYSTEM_STATE_FLYING && s_system_state != SYSTEM_STATE_ARMED) {
            continue;
        }
        
        /* 执行控制循环 */
        run_control_loop();
        
        /* 更新控制序列号 */
        s_control_seq++;
    }
}

/*==============================================================================
 * 任务实现 - 通信任务
 *============================================================================*/
/**
 * @brief 无线通信任务
 * @details 处理NRF24L01无线通信,接收控制指令和发送遥测数据
 * @param[in] pv_parameters 任务参数
 */
static void comm_task(void *pv_parameters)
{
    (void)pv_parameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);  /* 20Hz */
    
    control_packet_t rx_packet;
    
    while (1) {
        /* 等待下一个周期 */
        vTaskDelayUntil(&last_wake_time, frequency);
        
        /* 尝试接收控制指令 */
        error_code_t err = bsp_nrf24l01_receive_control(&rx_packet, 10);
        if (err == ERR_SUCCESS) {
            /* 复制到控制指令缓冲区 */
            memcpy((void *)&s_control_cmd, &rx_packet, sizeof(control_packet_t));
            s_new_control_cmd = true;
            
            /* 处理控制指令 */
            process_control_command(&s_control_cmd);
        }
        
        /* 发送遥测数据 */
        send_telemetry();
    }
}

/*==============================================================================
 * 任务实现 - 日志任务
 *============================================================================*/
/**
 * @brief 日志记录任务
 * @details 以10Hz频率记录飞行数据到W25Q64
 * @param[in] pv_parameters 任务参数
 */
static void log_task(void *pv_parameters)
{
    (void)pv_parameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100);  /* 10Hz */
    
    /* 初始化日志系统 */
    bsp_w25q64_log_init();
    
    while (1) {
        /* 等待下一个周期 */
        vTaskDelayUntil(&last_wake_time, frequency);
        
        /* 记录飞行数据 */
        log_flight_data();
    }
}

/*==============================================================================
 * 任务实现 - 电池监测任务
 *============================================================================*/
/**
 * @brief 电池监测任务
 * @details 以1Hz频率监测电池电压
 * @param[in] pv_parameters 任务参数
 */
static void battery_task(void *pv_parameters)
{
    (void)pv_parameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000);  /* 1Hz */
    
    while (1) {
        /* 等待下一个周期 */
        vTaskDelayUntil(&last_wake_time, frequency);
        
        /* 读取电池电压 */
        float voltage = bsp_adc_get_battery_voltage();
        s_flight_controller.battery_voltage = voltage;
        
        /* 检查低电压 */
        if (voltage < BATTERY_VOLTAGE_LOW && voltage > 5.0f) {
            s_flight_controller.battery_low = true;
            
            /* 低电压警告: 如果正在飞行,自动降落 */
            if (s_system_state == SYSTEM_STATE_FLYING) {
                /* 切换到悬停模式 */
                pid_set_mode(CONTROL_MODE_HOVER);
                s_flight_controller.current_mode = CONTROL_MODE_HOVER;
            }
        } else {
            s_flight_controller.battery_low = false;
        }
        
        /* 喂狗 (Watchdog reset) */
        IWDG_ReloadCounter();
    }
}


/*==============================================================================
 * 辅助函数 - 执行飞行模式PID计算
 *============================================================================*/
/**
 * @brief 根据当前模式执行对应的PID计算
 * @param[out] throttle_delta 油门调整量
 * @param[out] pitch_out 俯仰输出
 * @param[out] roll_out 横滚输出
 * @return error_code_t 错误码
 */
static error_code_t execute_flight_mode_pid(void)
{
    float pid_output = 0.0f;
    
    switch (s_flight_controller.current_mode) {
        case CTRL_MODE_UP_ACCEL_CONST:
            pid_mode_up_accel_const(Global_message[X_ACCLE_INDEX],Global_message[ALTITUDE_INDEX], &pid_output);
            s_flight_controller.throttle_delta = pid_output;
            break;
            
        case CTRL_MODE_UP_SPEED_CONST:
            pid_mode_up_speed_const(Global_message[Z_ACCLE_INDEX] * 0.1f,Global_message[ALTITUDE_INDEX], &pid_output); 
            s_flight_controller.throttle_delta = pid_output;
            break;
            
        case CTRL_MODE_DOWN_ACCEL_CONST:
            pid_mode_down_accel_const(Global_message[X_ACCLE_INDEX],Global_message[ALTITUDE_INDEX], &pid_output);      
            s_flight_controller.throttle_delta = pid_output;
            break;
            
        case CTRL_MODE_DOWN_SPEED_CONST:
            pid_mode_down_speed_const(Global_message[Z_ACCLE_INDEX] * 0.1f,Global_message[ALTITUDE_INDEX], &pid_output);
            s_flight_controller.throttle_delta = pid_output;
            break;
            
        case CTRL_MODE_HOVER:
            pid_mode_hover(s_flight_controller.target_altitude,Global_message[ALTITUDE_INDEX],Global_message[Z_ACCLE_INDEX], &pid_output);    
            s_flight_controller.throttle_delta = pid_output;
            /* 悬停同时需要姿态稳定 */
            s_flight_controller.pitch_out = s_attitude.pitch * 1.0f;
            s_flight_controller.roll_out = s_attitude.roll * 1.0f;  
            break;
            
        case CTRL_MODE_X_SPEED_CONST:
            pid_mode_x_speed_const(Global_message[X_ACCLE_INDEX] * 0.1f,Global_message[ROLL_INDEX], &pid_output);  
            s_flight_controller.pitch_out = pid_output;
            break;
            
        case CTRL_MODE_X_ACCEL_CONST:
            pid_mode_x_accel_const(Global_message[X_ACCLE_INDEX],Global_message[ROLL_INDEX], &pid_output);
            s_flight_controller.pitch_out = pid_output;
            break;
            
        case CTRL_MODE_Y_SPEED_CONST:
            pid_mode_y_speed_const(Global_message[Y_ACCLE_INDEX] * 0.1f,Global_message[PITCH_INDEX], &pid_output);   
            s_flight_controller.roll_out = pid_output;
            break;
            
        case CTRL_MODE_Y_ACCEL_CONST:
            pid_mode_y_accel_const(Global_message[Y_ACCLE_INDEX],Global_message[PITCH_INDEX], &pid_output);    
            s_flight_controller.roll_out = pid_output;
            break;
            
        default: NULL   
            ERR_INVALID_PARAM;
            break;
    }
    return ERR_SUCCESS;
}

/*==============================================================================
 * 辅助函数 - 控制循环
 *============================================================================*/
/**
 * @brief 执行控制循环
 * @return error_code_t 错误码
 */
static error_code_t run_control_loop(void)
{
    float throttle_base = s_flight_controller.throttle;
    float pitch_adjust = s_attitude.pitch * 0.5f; /* 默认P控制保持水平 */
    float roll_adjust = s_attitude.roll * 0.5f;
    float yaw_adjust = 0.0f;
    float throttle_delta = 0.0f;
    
    /* 执行当前模式的PID计算 */
    execute_flight_mode_pid(&throttle_delta, &pitch_adjust, &roll_adjust);
    
    /* 应用油门调整 */
    throttle_base += throttle_delta;
    
    /* 限制油门范围 */
    if (throttle_base > 100.0f) throttle_base = 100.0f;
    if (throttle_base < 0.0f) throttle_base = 0.0f;
    
    /* 保存控制输出 */
    s_control_output.pitch_out = pitch_adjust;
    s_control_output.roll_out = roll_adjust;
    s_control_output.yaw_out = yaw_adjust;
    s_control_output.altitude_out = 0; 
    
    /* 电机混控 */
    if (s_system_state == SYSTEM_STATE_FLYING) {
        motor_mixing(throttle_base, pitch_adjust, roll_adjust, yaw_adjust);
    } else {
        bsp_pwm_set_all_duty(0, 0, 0, 0);
    }
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 辅助函数 - 电机混控
 *============================================================================*/
/**
 * @brief 电机混控算法
 * @details 将PID输出混合到四个电机
 * @param[in] throttle 油门(0-100%)
 * @param[in] pitch_out 俯仰输出
 * @param[in] roll_out 横滚输出
 * @param[in] yaw_out 偏航输出
 * @return error_code_t 错误码
 */
static error_code_t motor_mixing(float throttle, float pitch_out, float roll_out, float yaw_out)
{
    /* X型四旋翼混控
     * 电机布局:
     *    M1(CW)    M2(CCW)
     *       \      /
     *        \    /
     *         \  /
     *          \/
     *          /\
     *         /  \
     *        /    \
     *       /      \
     *    M4(CCW)   M3(CW)
     */
    
    /* 混控计算 */
    float motor1 = throttle + pitch_out - roll_out - yaw_out;  /* 前左 */
    float motor2 = throttle + pitch_out + roll_out + yaw_out;  /* 前右 */
    float motor3 = throttle - pitch_out + roll_out - yaw_out;  /* 后右 */
    float motor4 = throttle - pitch_out - roll_out + yaw_out;  /* 后左 */
    
    /* 限幅到0-100% */
    motor1 = (motor1 < 0.0f) ? 0.0f : ((motor1 > 100.0f) ? 100.0f : motor1);
    motor2 = (motor2 < 0.0f) ? 0.0f : ((motor2 > 100.0f) ? 100.0f : motor2);
    motor3 = (motor3 < 0.0f) ? 0.0f : ((motor3 > 100.0f) ? 100.0f : motor3);
    motor4 = (motor4 < 0.0f) ? 0.0f : ((motor4 > 100.0f) ? 100.0f : motor4);
    
    /* 设置PWM输出 */
    s_control_output.motor_pwm[0] = motor1;
    s_control_output.motor_pwm[1] = motor2;
    s_control_output.motor_pwm[2] = motor3;
    s_control_output.motor_pwm[3] = motor4;
    
    bsp_pwm_set_all_duty(motor1, motor2, motor3, motor4);
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 辅助函数 - 处理控制指令
 *============================================================================*/
/**
 * @brief 处理控制指令
 * @param[in] cmd 控制指令指针
 * @return error_code_t 错误码
 */
static error_code_t process_control_command(const control_packet_t *cmd)
{
    if (cmd == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 更新控制模式 */
    if (cmd->mode < CONTROL_MODE_COUNT) {
        s_flight_controller.current_mode = cmd->mode;
        pid_set_mode(cmd->mode);
    }
    
    /* 更新油门 */
    s_flight_controller.throttle = cmd->throttle;
    
    /* 更新目标值 */
    s_flight_controller.target_pitch = cmd->pitch_setpoint;
    s_flight_controller.target_roll = cmd->roll_setpoint;
    s_flight_controller.target_yaw_rate = cmd->yaw_rate;
    s_flight_controller.target_altitude = cmd->altitude_setpoint;
    
    /* 更新最后接收时间 */
    s_flight_controller.last_cmd_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 辅助函数 - 发送遥测数据
 *============================================================================*/
/**
 * @brief 发送遥测数据
 * @return error_code_t 错误码
 */
static error_code_t send_telemetry(void)
{
    telemetry_packet_t telemetry;
    
    /* 填充遥测数据 */
    telemetry.header = 0xBB;
    telemetry.type = PACKET_TYPE_TELEMETRY;
    telemetry.seq = (uint16_t)s_control_seq;
    telemetry.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    telemetry.mode = s_flight_controller.current_mode;
    telemetry.pitch = s_attitude.pitch;
    telemetry.roll = s_attitude.roll;
    telemetry.yaw = s_attitude.yaw;
    telemetry.altitude = s_sensor_data.altitude;
    telemetry.battery_voltage = s_flight_controller.battery_voltage;
    telemetry.motor_rpm[0] = s_control_output.motor_pwm[0] * 100.0f;  /* 估算RPM */
    telemetry.motor_rpm[1] = s_control_output.motor_pwm[1] * 100.0f;
    telemetry.motor_rpm[2] = s_control_output.motor_pwm[2] * 100.0f;
    telemetry.motor_rpm[3] = s_control_output.motor_pwm[3] * 100.0f;
    telemetry.status = (uint8_t)s_system_state;
    
    /* 发送遥测数据 */
    bsp_nrf24l01_send_telemetry(&telemetry);
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 辅助函数 - 记录飞行数据
 *============================================================================*/
/**
 * @brief 记录飞行数据
 * @return error_code_t 错误码
 */
static error_code_t log_flight_data(void)
{
    log_entry_t entry;
    
    /* 填充日志条目 */
    entry.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    entry.mode = s_flight_controller.current_mode;
    entry.accel_x = s_sensor_data.accel.x;
    entry.accel_y = s_sensor_data.accel.y;
    entry.accel_z = s_sensor_data.accel.z;
    entry.gyro_x = s_sensor_data.gyro.x;
    entry.gyro_y = s_sensor_data.gyro.y;
    entry.gyro_z = s_sensor_data.gyro.z;
    entry.pitch = s_attitude.pitch;
    entry.roll = s_attitude.roll;
    entry.yaw = s_attitude.yaw;
    entry.altitude = s_sensor_data.altitude;
    entry.temperature = s_sensor_data.temperature;
    entry.battery_voltage = s_flight_controller.battery_voltage;
    entry.pwm_out[0] = s_control_output.motor_pwm[0];
    entry.pwm_out[1] = s_control_output.motor_pwm[1];
    entry.pwm_out[2] = s_control_output.motor_pwm[2];
    entry.pwm_out[3] = s_control_output.motor_pwm[3];
    entry.pid_output[0] = s_control_output.pitch_out;
    entry.pid_output[1] = s_control_output.roll_out;
    entry.pid_output[2] = s_control_output.yaw_out;
    
    /* 写入日志 */
    bsp_w25q64_log_write(&entry);
    
    return ERR_SUCCESS;
}

/*==============================================================================
 * 公有函数实现 - 初始化和控制
 *============================================================================*/
error_code_t flight_controller_init(void)
{
    /* 清零控制器结构体 */
    memset(&s_flight_controller, 0, sizeof(s_flight_controller));
    memset(&s_sensor_data, 0, sizeof(s_sensor_data));
    memset(&s_attitude, 0, sizeof(s_attitude));
    memset(&s_control_output, 0, sizeof(s_control_output));
    
    /* 初始化PID控制器 */
    pid_controller_init();
    
    /* 设置初始状态 */
    s_flight_controller.current_mode = CONTROL_MODE_HOVER;
    s_flight_controller.throttle = 0.0f;
    s_flight_controller.target_altitude = 0.0f;
    s_flight_controller.armed = false;
    s_flight_controller.flying = false;
    
    /* 初始化系统状态 */
    s_system_state = SYSTEM_STATE_INIT;
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_start(void)
{
    error_code_t err;
    
    /* 初始化所有硬件模块 */
    err = bsp_i2c_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_mpu6050_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_bmp250_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_adc_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_pwm_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_w25q64_init();
    if (err != ERR_SUCCESS) return err;
    
    err = bsp_nrf24l01_init();
    if (err != ERR_SUCCESS) return err;
    
    /* 创建FreeRTOS任务 */
    
    /* 传感器任务 - 优先级3 */
    if (xTaskCreate(sensor_task, "Sensor", TASK_STACK_SIZE_SENSOR, NULL, 
                    TASK_PRIORITY_SENSOR, &s_sensor_task_handle) != pdPASS) {
        return ERR_TASK_CREATE_FAILED;
    }
    
    /* 控制任务 - 优先级5(最高) */
    if (xTaskCreate(control_task, "Control", TASK_STACK_SIZE_CONTROL, NULL,
                    TASK_PRIORITY_CONTROL, &s_control_task_handle) != pdPASS) {
        return ERR_TASK_CREATE_FAILED;
    }
    
    /* 通信任务 - 优先级4 */
    if (xTaskCreate(comm_task, "Comm", TASK_STACK_SIZE_COMM, NULL,
                    TASK_PRIORITY_COMM, &s_comm_task_handle) != pdPASS) {
        return ERR_TASK_CREATE_FAILED;
    }
    
    /* 日志任务 - 优先级2 */
    if (xTaskCreate(log_task, "Log", TASK_STACK_SIZE_LOG, NULL,
                    TASK_PRIORITY_LOG, &s_log_task_handle) != pdPASS) {
        return ERR_TASK_CREATE_FAILED;
    }
    
    /* 电池监测任务 - 优先级2 */
    if (xTaskCreate(battery_task, "Battery", TASK_STACK_SIZE_BATTERY, NULL,
                    TASK_PRIORITY_BATTERY, &s_battery_task_handle) != pdPASS) {
        return ERR_TASK_CREATE_FAILED;
    }
    
    /* 更新系统状态 */
    s_system_state = SYSTEM_STATE_STANDBY;
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_arm(void)
{
    if (s_system_state != SYSTEM_STATE_STANDBY) {
        return ERR_INVALID_STATE;
    }
    
    /* 检查传感器状态 */
    if (!s_flight_controller.sensor_status.mpu6050_ok || 
        !s_flight_controller.sensor_status.bmp250_ok) {
        return ERR_SENSOR_NOT_READY;
    }
    
    /* 检查电池电压 */
    if (s_flight_controller.battery_low) {
        return ERR_BATTERY_LOW;
    }
    
    /* 解锁 */
    s_flight_controller.armed = true;
    s_system_state = SYSTEM_STATE_ARMED;
    
    /* 重置PID控制器 */
    pid_reset(s_flight_controller.current_mode, 255);
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_disarm(void)
{
    /* 停止电机 */
    bsp_pwm_emergency_stop();
    
    /* 清零输出 */
    memset(&s_control_output, 0, sizeof(s_control_output));
    
    /* 锁定 */
    s_flight_controller.armed = false;
    s_flight_controller.flying = false;
    s_flight_controller.throttle = 0.0f;
    
    s_system_state = SYSTEM_STATE_STANDBY;
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_takeoff(float target_altitude)
{
    if (!s_flight_controller.armed) {
        return ERR_NOT_ARMED;
    }
    
    if (s_flight_controller.flying) {
        return ERR_ALREADY_FLYING;
    }
    
    /* 设置目标高度 */
    s_flight_controller.target_altitude = target_altitude;
    
    /* 切换到向上匀速模式 */
    s_flight_controller.current_mode = CONTROL_MODE_UP_CONST_SPEED;
    pid_set_mode(CONTROL_MODE_UP_CONST_SPEED);
    
    /* 设置初始油门 */
    s_flight_controller.throttle = 50.0f;
    
    /* 标记为飞行状态 */
    s_flight_controller.flying = true;
    s_system_state = SYSTEM_STATE_FLYING;
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_land(void)
{
    if (!s_flight_controller.flying) {
        return ERR_NOT_FLYING;
    }
    
    /* 切换到向下匀速模式 */
    s_flight_controller.current_mode = CONTROL_MODE_DOWN_CONST_SPEED;
    pid_set_mode(CONTROL_MODE_DOWN_CONST_SPEED);
    
    /* 设置目标高度为0 */
    s_flight_controller.target_altitude = 0.0f;
    
    /* 降低油门 */
    s_flight_controller.throttle = 30.0f;
    
    /* 当高度低于阈值时自动锁定 */
    if (s_sensor_data.altitude < 0.3f) {
        flight_controller_disarm();
    }
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_set_mode(control_mode_t mode)
{
    if (mode >= CONTROL_MODE_COUNT) {
        return ERR_INVALID_PARAM;
    }
    
    s_flight_controller.current_mode = mode;
    pid_set_mode(mode);
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_emergency_stop(void)
{
    /* 紧急停止电机 */
    bsp_pwm_emergency_stop();
    
    /* 清零输出 */
    memset(&s_control_output, 0, sizeof(s_control_output));
    
    /* 锁定 */
    s_flight_controller.armed = false;
    s_flight_controller.flying = false;
    s_flight_controller.throttle = 0.0f;
    
    s_system_state = SYSTEM_STATE_ERROR;
    
    return ERR_SUCCESS;
}

system_state_t flight_controller_get_state(void)
{
    return s_system_state;
}

error_code_t flight_controller_get_attitude(attitude_t *attitude)
{
    if (attitude == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memcpy(attitude, &s_attitude, sizeof(attitude_t));
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_get_sensor_data(sensor_data_t *data)
{
    if (data == NULL) {
        return ERR_INVALID_PARAM;
    }
    else if (!s_flight_controller.sensor_status.mpu6050_ok || 
             !s_flight_controller.sensor_status.bmp250_ok) {
        return ERR_SENSOR_NOT_READY;
    }
    else if (!data->mpu6050_data_t.vaild||!data->bmp250_data_t.vaild) {
        return ERR_SENSOR_DATA_INVALID;
    }
    memcpy(data, &s_sensor_data, sizeof(sensor_data_t));
    
    return ERR_SUCCESS;
}

error_code_t flight_controller_get_control_output(control_output_t *output)
{
    if (output == NULL) {
        return ERR_INVALID_PARAM;
    }
    else if (!s_flight_controller.armed) {
        return ERR_NOT_ARMED;
    }
    else if(!output->motor_pwm.vaild) {
        return ERR_SENSOR_DATA_INVALID;
    }
    
    memcpy(output, &s_control_output, sizeof(control_output_t));
    
    return ERR_SUCCESS;
}

uint32_t flight_controller_get_flight_time_ms(void)
{
    if (s_flight_controller.flying) {
        return (xTaskGetTickCount() * portTICK_PERIOD_MS) - s_flight_controller.takeoff_time_ms;
    }
    return 0;
}
