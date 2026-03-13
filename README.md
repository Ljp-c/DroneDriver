# STM32F103C8T6 无人机四轴飞行器飞控系统

## 项目概述

基于STM32F103C8T6主控芯片的企业级无人机飞控系统，采用硬件层-应用层解耦架构，集成FreeRTOS实时操作系统，支持9种飞行控制模式。

## 工程架构

```
无人机四轴飞行器/
├── common/                 # 公共模块
│   ├── error_code.h/c     # 全局错误码定义
│   ├── system_types.h     # 系统类型定义
│   ├── global_data.h/c    # 全局数据定义
├── src/                    # 硬件层（BSP）
│   ├── bsp_i2c.h/c        # I2C总线驱动
│   ├── bsp_mpu6050.h/c    # MPU6050传感器驱动
│   ├── bsp_bmp250.h/c     # BMP250气压传感器驱动
│   ├── bsp_adc.h/c        # ADC电压监测
│   ├── bsp_pwm.h/c        # PWM输出控制
│   ├── bsp_w25q64.h/c     # W25Q64日志存储
│   └── bsp_nrf24l01.h/c   # NRF24L01无线通讯
├── app/                    # 应用层
│   ├── flight_controller.h/c  # 飞行控制主逻辑
│   ├── pid_controller.h/c     # PID控制算法
│   ├── FreeRTOSConfig.h       # FreeRTOS配置
│   └── main.c             # 程序入口
├── FreeRTOS-Kernel/        # FreeRTOS内核
└── library/                # STM32标准库
```

## 硬件配置

### 引脚分配

| 功能模块 | 引脚 | 说明 |
|---------|------|------|
| **MPU6050** | PB10 (SCL)<br>PB11 (SDA)<br>PA11 (INT) | I2C2接口<br>数据就绪中断 |
| **BMP250** | PB10 (SCL)<br>PB11 (SDA) | I2C2接口（与MPU6050共用） |
| **ADC电压** | PB1 | 电池电压检测 |
| **PWM输出** | PA0-PA3 | 4通道PWM输出（TIM2） |
| **W25Q64** | PB12 (CS)<br>PB13 (SCK)<br>PB14 (MISO)<br>PB15 (MOSI) | SPI2接口 |
| **NRF24L01** | PA0 (CS)<br>PA1 (IRQ)<br>PA4 (CE)<br>PA5 (SCK)<br>PA6 (MISO)<br>PA7 (MOSI) | SPI1接口 |

## 控制模式

系统支持9种飞行控制模式：

| 模式ID | 模式名称 | 说明 |
|--------|---------|------|
| 0 | 向上恒定加速度 | 加速度固定0.05m/s² |
| 1 | 向下恒定加速度 | 加速度固定-0.05m/s² |
| 2 | 向上匀速 | 速度固定1.0m/s |
| 3 | 向下匀速 | 速度固定-1.0m/s |
| 4 | 浮空悬停 | 保持当前高度和姿态 |
| 5 | X轴正方向水平飞行 | 沿X轴正向飞行 |
| 6 | X轴负方向水平飞行 | 沿X轴负向飞行 |
| 7 | Y轴正方向水平飞行 | 沿Y轴正向飞行 |
| 8 | Y轴负方向水平飞行 | 沿Y轴负向飞行 |

## 任务优先级

| 任务 | 优先级 | 周期 | 说明 |
|------|--------|------|------|
| PID控制 | 5 (最高) | 10ms | 实时控制算法 |
| 通信 | 4 | 50ms | NRF24L01无线通信 |
| 传感器采集 | 3 | 10ms | MPU6050+BMP250 |
| 日志记录 | 2 | 100ms | W25Q64数据记录 |
| 电池监测 | 2 | 1000ms | ADC电压检测 |

## FreeRTOS配置

- 系统时钟: 72MHz
- 时钟节拍: 1kHz (1ms)
- 堆内存: 10KB
- 最大优先级: 8
- 调度方式: 抢占式

## 错误码定义

```c
/* 通用错误 (0x00-0x0F) */
#define ERR_SUCCESS                     0x00    /* 成功 */
#define ERR_NULL_POINTER                0x01    /* 空指针 */
#define ERR_INVALID_PARAM               0x02    /* 无效参数 */
#define ERR_TIMEOUT                     0x03    /* 超时 */

/* 传感器错误 (0x10-0x2F) */
#define ERR_MPU6050_INIT_FAILED         0x10
#define ERR_BMP250_INIT_FAILED          0x20

/* 存储错误 (0x40-0x4F) */
#define ERR_W25Q64_INIT_FAILED          0x40
#define ERR_FLASH_TIMEOUT               0x47

/* 通信错误 (0x60-0x6F) */
#define ERR_NRF24L01_INIT_FAILED        0x60
#define ERR_NRF24L01_TIMEOUT            0x68

/* 飞行控制错误 (0x90-0x9F) */
#define ERR_NOT_ARMED                   0x90
#define ERR_ALREADY_FLYING              0x91
#define ERR_BATTERY_LOW                 0x94

/* 系统错误 (0xF0-0xFF) */
#define ERR_TASK_CREATE_FAILED          0xF1
```

## 编码规范

1. **命名规范**
   - 函数：`bsp_xxx_action()` 或 `module_action()`
   - 变量：`lower_case_with_underscores`
   - 常量：`UPPER_CASE_WITH_UNDERSCORES`
   - 类型：`lower_case_t`

2. **注释规范**
   - 文件头包含版本、日期、功能说明
   - 函数注释包含参数、返回值、功能说明
   - 关键算法需说明推导过程

3. **错误处理**
   - 所有函数返回错误码
   - 使用`IS_ERROR_OK()`和`IS_ERROR_FAIL()`宏检查

4. **架构规范**
   - 硬件层与应用层解耦
   - 除`Global_message`外不使用全局变量
   - 使用FreeRTOS信号量/互斥锁进行任务同步

## 使用说明

### 初始化流程

```c
#include "flight_controller.h"

int main(void)
{
    error_code_t err;
    
    /* 初始化飞行控制系统 */
    err = flight_controller_init();
    if(err != ERR_SUCCESS) {
        /* 错误处理 */
    }
    
    /* 启动系统（创建任务并启动调度器） */
    err = flight_controller_start();
    
    /* 正常情况下不会执行到这里 */
    while(1);
}
```

### 解锁与起飞

```c
/* 解锁 */
flight_controller_arm();

/* 起飞到2米高度 */
flight_controller_takeoff(2.0f);

/* 设置为悬停模式 */
flight_controller_set_mode(CONTROL_MODE_HOVER);
```

### 紧急停止

```c
/* 紧急停止所有电机 */
flight_controller_emergency_stop();
```

## 版本历史

| 版本 | 日期 | 修改内容 | 作者 |
|------|------|---------|------|
| 1.0.0 | 2025 | 初始版本，完整实现9种控制模式 | - |

## 许可证

MIT License
Copyright (c) 2025
