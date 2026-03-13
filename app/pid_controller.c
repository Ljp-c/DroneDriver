/**
 * @file pid_controller.c
 * @brief PID控制器实现 (9种控制模式)0z
 * @details 严格遵循规范实现9种控制模式PID控制器，每个模式包含积分限幅和微分滤波
 * @version 1.0.0
 * @date 2026-02-08
 */
#include "error_code.h"
#include "pid_controller.h"
#include "system_types.h"
#include "global_data.h"
/* 私有PID状态存储 */
static pid_state_t s_states[CTRL_MODE_MAX];
static pid_params_t s_params[CTRL_MODE_MAX];
static PWM_Output_t s_pwm_output;
static float dt=10.0f;
/**
 * @brief 内环控制器（加速度环）
 * @param acc_target  目标加速度
 * @param dt          采样周期(秒)
 * @param index       全局数据索引
 * @return 输出力矩/电流指令
 */
static float inner_ring(float acc_target, float dt, globedata index)
{
    if (dt <= 0.0f) return 0.0f;

    /* 读取加速度测量值 */
    float acc_measure = Global_message[index];  // 角加速度测量
    float error = acc_target - acc_measure;

    /* 只有P项（内环快速响应） */
    float output = s_params->kp * error;

    /* 输出限幅 */
    // output = CLAMP(output, -s_params->out_limit, s_params->out_limit);
    /* 输出限幅 */
// if (output >  s_params->out_limit)
//     output =  s_params->out_limit;
// else if (output < -s_params->out_limit)
//     output = -s_params->out_limit;
    float output = s_params->kp * error ;

    /* 判断是否应该停止积分 */
    bool is_saturated = (output >  s_params->out_limit) ||
                        (output < -s_params->out_limit);
    bool same_sign = ((error > 0.0f) && (s_states->integral > 0.0f)) ||
                    ((error < 0.0f) && (s_states->integral < 0.0f));

    /* 只在输出饱和且误差同向时才停止积分 */
    if (!(is_saturated && same_sign))
    {
        s_states->integral += error * dt;
    }

    /* 更新输出并限幅 */
    output = s_params->kp ;
    if (output >  s_params->out_limit)  output =  s_params->out_limit;
    if (output < -s_params->out_limit)  output = -s_params->out_limit;

    return output;
        return output;
    }
        
/**
 * @brief 外环控制器（速度环）
 * @param vel_target   目标速度
 * @param index        全局数据索引
 * @param dt           采样周期(秒)
 * @return 输出给内环的加速度指令
 */
static float outer_ring(float vel_target, globedata index, float dt)
{
    /* -------- 参数有效性检查 -------- */
    // dt过小或过大都拒绝执行，返回上次输出保持稳定
    if (dt <= 1e-6f || dt > 0.1f)  
    {
        return s_states->last_output;
    }

    /* -------- 读取测量值并计算误差 -------- */
    float vel_measure =Global_message[index];  // 假设这是速度测量值
    float error = vel_target - vel_measure;

    /* -------- P项：比例控制 -------- */
    float p_term = s_params->kp * error;

    /* -------- I项：积分控制（消除稳态误差） -------- */
    // 先试算积分更新后的值
    float integral_new = s_states->integral + error * dt;
    
    // 积分限幅（防止积分无限累积）
    if (integral_new > s_params->i_limit)
        integral_new = s_params->i_limit;
    else if (integral_new < -s_params->i_limit)
        integral_new = -s_params->i_limit;
    
    float i_term = s_params->ki * integral_new;

    /* -------- 合成输出：加速度指令 -------- */
    float acc_target = p_term + i_term;

    /* -------- 抗积分饱和（Anti-Windup）-------- */
    bool is_saturated = (acc_target > s_params->out_limit) || 
                        (acc_target < -s_params->out_limit);
    bool same_sign = (error > 0.0f && integral_new > 0.0f) || 
                     (error < 0.0f && integral_new < 0.0f);

    if (is_saturated && same_sign)
    {
        // 输出饱和且误差与积分同向：停止积分累积
        // 保持积分不变，不更新 s_states->integral
    }
    else
    {
        // 未饱和或反向积分（有助于退出饱和）：正常更新
        s_states->integral = integral_new;
    }

    /* -------- 输出限幅 -------- */
    if (acc_target > s_params->out_limit)
        acc_target = s_params->out_limit;
    else if (acc_target < -s_params->out_limit)
        acc_target = -s_params->out_limit;

    /* -------- 记录输出供下次使用 -------- */
    s_states->last_output = acc_target;

    return acc_target;
}
/**
 * @brief 速度控制器
 * @param vel_target   目标速度
 * @return 返回控制错误码
 */
 static error_code_t Speed_controler(float vel_target)
{
        float acc_target = outer_ring(vel_target, globedata_vel, dt);
        return inner_ring(acc_target, dt, globedata_acc);

}
/**
 * @brief 加速度控制器
 * @param acc_target   目标加速度   
 * @return 返回控制错误码
 */
static error_code_t Accel_controler(float acc_target, globedata index)
{
        return inner_ring(acc_target, dt, Global_message[index]);

}

error_code_t pid_controller_init(void)
{
    /* 初始化参数 (示例值，实际需调试) */
    for(int i=0; i<CTRL_MODE_MAX; i++) {
        s_params[i].kp = 1.0f; s_params[i].ki = 0.1f; s_params[i].kd = 0.05f;
        s_states[i].integral = 0; s_states[i].last_error = 0;
    }
    return ERR_OK;
}

error_code_t pid_reset_all(void)
{
    for(int i=0; i<CTRL_MODE_MAX; i++) {
        s_states[i].integral = 0; s_states[i].last_error = 0;
    }
    return ERR_OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        /** 飞行控制任务 **/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 1. 上升恒定加速度模式: 目标 +0.05m/s² */
error_code_t pid_mode_up_accel_const(float current_acc_z, float current_height, pwm_out)
{
    (void)current_height; /* 未直接使用高度，但需做安全边界逻辑 */
    if (pwm_out == NULL) return ERR_NULL_POINTER;
    
    float target = 0.05f; /* +0.05 m/s² */
    *pwm_out =  Accel_controler(target+Global_message[GRAVITY_INDEX], Z_ACCLE_INDEX);
    return ERR_OK;
}

/* 2. 上升匀速模式: 目标速度 (需外部得到) */
error_code_t pid_mode_up_speed_const(float current_speed, float current_height, float *pwm_out)
{
    (void)current_height;
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = 1.0f; /* 默认目标上升速度 1.0 m/s */
    
    *pwm_out = Speed_controler(target);
    return ERR_OK;
}

/* 3. 下降恒定加速度模式: 目标 -0.05m/s² */
error_code_t pid_mode_down_accel_const(float current_acc_x, float current_height, float *pwm_out)
{
    (void)current_height;
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = -0.05f;
    
    *pwm_out = Accel_controler(target+Global_message[GRAVITY_INDEX], Z_ACCLE_INDEX);
    return ERR_OK;
}

/* 4. 下降匀速模式: 自动减速逻辑 */
error_code_t pid_mode_down_speed_const(float current_speed, float current_height, float *pwm_out)
{
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = -1.0f; /* 默认下降速度 */
    /* 接近地面(<0.5m)自动减速 */
    if (current_height < 0.5f) target = -0.2f;

    *pwm_out = Speed_controler(target);
    return ERR_OK;
}

/* 5. 悬停: 高度 + 姿态 */
error_code_t pid_mode_hover(float target_height, float current_height, float acc_z, float *pwm_out)
{
    (void)acc_z; /* 可加入高度D项 */
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    *pwm_out = Accel_controler(target_height-current_height, Z_ACCLE_INDEX);
    return ERR_OK;
}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/* 6. X轴水平匀速: 目标 0.5m/s -> 控制 PA0/PA1 */
error_code_t pid_mode_x_speed_const(float current_speed_x, float pitch_angle, float *pwm_out)
{
    (void)pitch_angle; /* 姿态控制需在外环处理 */
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = 0.5f;
    *pwm_out = pid_calc_core(&s_states[CTRL_MODE_X_SPEED_CONST],
                             &s_params[CTRL_MODE_X_SPEED_CONST],
                             target, current_speed_x);
    return ERR_OK;
}

/* 7. X轴水平加速: 目标 0.05m/s² */
error_code_t pid_mode_x_accel_const(float current_acc_x, float pitch_angle, float *pwm_out)
{
    (void)pitch_angle;
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = 0.05f;
    *pwm_out = pid_calc_core(&s_states[CTRL_MODE_X_ACCEL_CONST],
                             &s_params[CTRL_MODE_X_ACCEL_CONST],
                             target, current_acc_x);
    return ERR_OK;
}

/* 8. Y轴水平匀速: 目标 0.5m/s -> 控制 PA2/PA3 */
error_code_t pid_mode_y_speed_const(float current_speed_y, float roll_angle, float *pwm_out)
{
    (void)roll_angle;
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = 0.5f;
    *pwm_out = pid_calc_core(&s_states[CTRL_MODE_Y_SPEED_CONST],
                             &s_params[CTRL_MODE_Y_SPEED_CONST],
                             target, current_speed_y);
    return ERR_OK;
}

/* 9. Y轴水平加速: 目标 0.05m/s² */
error_code_t pid_mode_y_accel_const(float current_acc_y, float roll_angle, float *pwm_out)
{
    (void)roll_angle;
    if (pwm_out == NULL) return ERR_NULL_POINTER;

    float target = 0.05f;
    *pwm_out = pid_calc_core(&s_states[CTRL_MODE_Y_ACCEL_CONST],
                             &s_params[CTRL_MODE_Y_ACCEL_CONST],
                             target, current_acc_y);
    return ERR_OK;
}
