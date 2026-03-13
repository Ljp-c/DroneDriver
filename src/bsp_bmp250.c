/**
 * @file bsp_bmp250.c
 * @brief BMP250气压传感器驱动实现
 * @version 1.0.0
 * @date 2026-02-08
 * 
 * @copyright Copyright (c) 2026
 * 
 * 功能说明：
 * - 实现BMP250初始化、IIC读写
 * - 提供高度数据采集与校准功能
 * - 支持温度补偿
 * 
 * 补偿算法说明：
 * 温度补偿公式（来自Bosch官方数据手册）：
 *   var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2
 *   var2 = (adc_T / 131072.0 - dig_T1 / 8192.0)^2 * dig_T3
 *   t_fine = var1 + var2
 *   T = (var1 + var2) / 5120.0
 * 
 * 压力补偿公式：
 *   var1 = t_fine / 2.0 - 64000.0
 *   var2 = var1^2 * dig_P6 / 32768.0
 *   var2 = var2 + var1 * dig_P5 * 2.0
 *   ...（完整公式见代码）
 * 
 * 高度计算公式（国际标准大气模型ISA）：
 *   h = 44330 * (1 - (P/P0)^(1/5.255))
 *   其中P0 = 1013.25 hPa（海平面标准气压）
 *   推导依据：大气压强随高度指数衰减规律
 */

#include "bsp_bmp250.h"
#include "bsp_i2c.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

/* 静态变量 */
static bmp250_calib_t s_calib;
static uint8_t s_initialized = 0;

static SemaphoreHandle_t s_data_ready_sem = NULL;

/* 默认海平面气压（帕斯卡） */
static float s_sea_level_pressure = 101325.0;

/* 函数声明 */
static uint8_t bmp250_read_calib(void);

/**
 * @brief BMP250初始化
 * @param config: 配置参数，NULL使用默认配置
 * @return 错误码
 */
uint8_t bsp_bmp250_init(const bmp250_config_t *config)
{
    uint8_t id = 0;
    uint8_t err;
    uint8_t ctrl_meas;
    uint8_t cfg;
    
    /* 创建数据就绪信号量（用于任务同步） */
    if(s_data_ready_sem == NULL) {
        s_data_ready_sem = xSemaphoreCreateBinary();
        if(s_data_ready_sem == NULL) {
            return ERR_SEMAPHORE_CREATE_FAILED;
        }
    }
    
    /* 软件复位 */
    err = bsp_bmp250_soft_reset();
    if(err != ERR_OK) {
        return err;
    }
    
    /* 延时等待复位完成（10ms） */
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* 检查芯片ID */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_ID, &id, 1);
    if(err != ERR_OK || id != BMP250_DEVICE_ID) {
        return ERR_BMP250_ID_MISMATCH;
    }
    
    /* 读取校准参数 */
    err = bmp250_read_calib();
    if(err != ERR_OK) {
        return ERR_BMP250_CALIB_READ;
    }
    
    /* 配置工作模式 */
    if(config != NULL) {
        /* 使用自定义配置 */
        ctrl_meas = ((config->temp_oversampling & 0x07) << 5) |
                    ((config->press_oversampling & 0x07) << 2) |
                    (config->work_mode & 0x03);
        cfg = ((config->standby_time & 0x07) << 5) |
              ((config->filter_coeff & 0x07) << 2);
    } else {
        /* 使用默认配置：温度x2，压力x16，正常模式，IIR滤波x4，待机250ms */
        ctrl_meas = (0x02 << 5) | (0x05 << 2) | 0x03;
        cfg = (0x03 << 5) | (0x02 << 2);
    }
    
    /* 写入配置寄存器 */
    err = bsp_i2c_write_byte(BMP250_ADDR, BMP250_REG_CONFIG, cfg);
    if(err != ERR_OK) {
        return ERR_BMP250_I2C_WRITE;
    }
    
    /* 写入测量控制寄存器 */
    err = bsp_i2c_write_byte(BMP250_ADDR, BMP250_REG_CTRL_MEAS, ctrl_meas);
    if(err != ERR_OK) {
        return ERR_BMP250_I2C_WRITE;
    }
    
    s_initialized = 1;
    return ERR_OK;
}

/**
 * @brief 读取所有物理量（温度、气压、高度）
 * @param data: 物理量数据结构体
 * @return 错误码
 */
uint8_t bsp_bmp250_read_all(bmp250_data_t *data)
{
    int32_t adc_temp, adc_press;
    int32_t t_fine;
    float temp, press;
    uint8_t err1, err2;
    
    if(data == NULL) {
        return ERR_NULL_POINTER;
    }
    
    if(!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 读取原始温度数据 */
    err1 = bsp_bmp250_read_raw_temp(&adc_temp);
    if(err1 != ERR_OK) {
        return err1;
    }
    
    /* 读取原始压力数据 */
    err2 = bsp_bmp250_read_raw_press(&adc_press);
    if(err2 != ERR_OK) {
        return err2;    
    }
    
    /* 温度补偿计算 */
    temp = bsp_bmp250_compensate_temp(adc_temp, &t_fine);
    
    /* 压力补偿计算 */
    press = bsp_bmp250_compensate_press(adc_press, t_fine);
    
    /* 计算高度 */
    data->altitude = (float)bsp_bmp250_press_to_altitude(press, 0);
    data->pressure = (float)press;
    data->temperature = (float)temp;
    if(err1 == ERR_OK && err2 == ERR_OK) {
        data->valid = 1;
    } else {
        data->valid = 0;
    }
    
    /* 释放数据就绪信号量 */
    if(s_data_ready_sem != NULL) {
        xSemaphoreGive(s_data_ready_sem);
    }
    
    return ERR_OK;
}

/**
 * @brief 读取原始温度数据（ADC值）
 * @param adc_temp: 输出原始温度ADC值
 * @return 错误码
 */
uint8_t bsp_bmp250_read_raw_temp(int32_t *adc_temp)
{
    uint8_t buf[3];
    uint8_t err;
    
    if(adc_temp == NULL) {
        return ERR_NULL_POINTER;
    }
    
    /* 读取温度寄存器 */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_TEMP_MSB, buf, 3);
    if(err != ERR_OK) {
        return ERR_BMP250_I2C_READ;
    }
    
    /* 组合20位数据 */
    *adc_temp = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    
    return ERR_OK;
}

/**
 * @brief 读取原始压力数据（ADC值）
 * @param adc_press: 输出原始压力ADC值
 * @return 错误码
 */
uint8_t bsp_bmp250_read_raw_press(int32_t *adc_press)
{
    uint8_t buf[3];
    uint8_t err;
    
    if(adc_press == NULL) {
        return ERR_NULL_POINTER;
    }
    
    /* 读取压力寄存器 */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_PRESS_MSB, buf, 3);
    if(err != ERR_OK) {
        return ERR_BMP250_I2C_READ;
    }
    
    /* 组合20位数据 */
    *adc_press = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    
    return ERR_OK;
}

/**
 * @brief 温度补偿计算
 * @param adc_temp: 原始温度ADC值
 * @param t_fine: 输出精细温度值
 * @return 补偿后的温度值（摄氏度）
 */
float bsp_bmp250_compensate_temp(int32_t adc_temp, int32_t *t_fine)
{
    float var1, var2, T;
    
    /* 温度补偿公式（Bosch官方） */
    var1 = (((float)adc_temp) / 16384.0 - ((float)s_calib.dig_t1) / 1024.0) *
           ((float)s_calib.dig_t2);
    var2 = ((((float)adc_temp) / 131072.0 - ((float)s_calib.dig_t1) / 8192.0) *
            (((float)adc_temp) / 131072.0 - ((float)s_calib.dig_t1) / 8192.0)) *
           ((float)s_calib.dig_t3);
    
    *t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0;
    
    return T;
}

/**
 * @brief 压力补偿计算
 * @param adc_press: 原始压力ADC值
 * @param t_fine: 精细温度值
 * @return 补偿后的压力值（帕斯卡）
 */
int32_t bmp280_compensate_press(int32_t adc_press, int32_t t_fine)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_p6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_p5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_p3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) *
           ((int64_t)s_calib.dig_p1) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)s_calib.dig_p9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)s_calib.dig_p8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) +
        (((int64_t)s_calib.dig_p7) << 4);

    return (int32_t)p;   // 返回 Pa * 256
}
/**
 * @brief 压力转换为高度
 * @param pressure_pa: 压力值（帕斯卡）
 * @param sea_level_pressure: 海平面参考压力（帕斯卡），0使用默认值
 * @return 高度值（米）
 */
float bsp_bmp250_press_to_altitude(float pressure_pa, float sea_level_pressure)
{
    float p0;
    float altitude;
    
    /* 使用默认或自定义海平面气压 */
    p0 = (sea_level_pressure > 0) ? sea_level_pressure : s_sea_level_pressure;
    
    /* 国际标准大气模型（ISA）高度公式 （泰勒近似）*/
    altitude = 8434.0f * (p0 - pressure_pa) / p0;
    
    return altitude;
}

/**
 * @brief 强制进行一次测量
 * @return 错误码
 */
uint8_t bsp_bmp250_trigger_measurement(void)
{
    uint8_t ctrl_meas;
    uint8_t err;
    
    /* 读取当前配置 */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_CTRL_MEAS, &ctrl_meas, 1);
    if(err != ERR_OK) {
        return err;
    }
    
    /* 设置为强制模式 */
    ctrl_meas = (ctrl_meas & 0xFC) | 0x01;
    
    /* 写入测量控制寄存器 */
    err = bsp_i2c_write_byte(BMP250_ADDR, BMP250_REG_CTRL_MEAS, ctrl_meas);
    if(err != ERR_OK) {
        return ERR_BMP250_I2C_WRITE;
    }
    
    return ERR_OK;
}

/**
 * @brief 检查测量是否完成
 * @param ready: 输出就绪状态，1表示完成
 * @return 错误码
 */
uint8_t bsp_bmp250_check_measurement_ready(uint8_t *ready)
{
    uint8_t status;
    uint8_t err;
    
    if(ready == NULL) {
        return ERR_NULL_POINTER;
    }
    
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_STATUS, &status, 1);
    if(err != ERR_OK) {
        return err;
    }
    
    /* bit3为1表示正在测量 */
    *ready = (status & 0x08) ? 0 : 1;
    
    return ERR_OK;
}

/**
 * @brief 进入低功耗模式
 * @return 错误码
 */
uint8_t bsp_bmp250_enter_sleep(void)
{
    uint8_t ctrl_meas;
    uint8_t err;
    
    /* 读取当前配置 */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_CTRL_MEAS, &ctrl_meas, 1);
    if(err != ERR_OK) {
        return err;
    }
    
    /* 设置为睡眠模式 */
    ctrl_meas = ctrl_meas & 0xFC;
    
    return bsp_i2c_write_byte(BMP250_ADDR, BMP250_REG_CTRL_MEAS, ctrl_meas);
}

/**
 * @brief 软件复位
 * @return 错误码
 */
uint8_t bsp_bmp250_soft_reset(void)
{
    return bsp_i2c_write_byte(BMP250_ADDR, BMP250_REG_RESET, BMP250_RESET_CMD);
}

/**
 * @brief 获取数据就绪信号量（用于任务同步）
 * @return 信号量句柄
 */
SemaphoreHandle_t bsp_bmp250_get_data_sem(void)
{
    return s_data_ready_sem;
}

/* ===================== 内部函数 ===================== */

/**
 * @brief 读取校准参数
 * @return 错误码
 */
static uint8_t bmp250_read_calib(void)
{
    uint8_t calib[BMP250_REG_CALIB_LEN];
    uint8_t err;
    
    /* 读取校准数据（24字节） */
    err = bsp_i2c_read_reg(BMP250_ADDR, BMP250_REG_CALIB_START, calib, BMP250_REG_CALIB_LEN);
    if(err != ERR_OK) {
        return err;
    }
    
    /* 解析校准参数 */
    s_calib.dig_t1 = (uint16_t)((calib[1] << 8) | calib[0]);
    s_calib.dig_t2 = (int16_t)((calib[3] << 8) | calib[2]);
    s_calib.dig_t3 = (int16_t)((calib[5] << 8) | calib[4]);
    s_calib.dig_p1 = (uint16_t)((calib[7] << 8) | calib[6]);
    s_calib.dig_p2 = (int16_t)((calib[9] << 8) | calib[8]);
    s_calib.dig_p3 = (int16_t)((calib[11] << 8) | calib[10]);
    s_calib.dig_p4 = (int16_t)((calib[13] << 8) | calib[12]);
    s_calib.dig_p5 = (int16_t)((calib[15] << 8) | calib[14]);
    s_calib.dig_p6 = (int16_t)((calib[17] << 8) | calib[16]);
    s_calib.dig_p7 = (int16_t)((calib[19] << 8) | calib[18]);
    s_calib.dig_p8 = (int16_t)((calib[21] << 8) | calib[20]);
    s_calib.dig_p9 = (int16_t)((calib[23] << 8) | calib[22]);
    
    return ERR_OK;
}
