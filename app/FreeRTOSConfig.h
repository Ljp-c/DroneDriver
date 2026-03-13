/**
 * @file FreeRTOSConfig.h
 * @brief FreeRTOS配置文件
 * @details 针对STM32F103C8T6的FreeRTOS内核配置
 *          系统时钟: 72MHz
 *          任务调度: 抢占式
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*==============================================================================
 * 内核配置
 *============================================================================*/
#define configUSE_PREEMPTION                    1       /**< 使用抢占式调度 */
#define configUSE_IDLE_HOOK                     0       /**< 不使用空闲钩子 */
#define configUSE_TICK_HOOK                     0       /**< 不使用时钟钩子 */
#define configCPU_CLOCK_HZ                      72000000UL  /**< CPU时钟频率: 72MHz */
#define configTICK_RATE_HZ                      1000    /**< 时钟节拍: 1kHz (1ms) */
#define configMAX_PRIORITIES                    8       /**< 最大优先级数 */
#define configMINIMAL_STACK_SIZE                128     /**< 最小任务堆栈大小(字) */
#define configTOTAL_HEAP_SIZE                   10240   /**< 堆内存大小(字节): 10KB */
#define configMAX_TASK_NAME_LEN                 16      /**< 任务名最大长度 */
#define configUSE_TRACE_FACILITY                0       /**< 不使用跟踪功能 */
#define configUSE_16_BIT_TICKS                  0       /**< 使用32位时钟节拍 */
#define configIDLE_SHOULD_YIELD                 1       /**< 空闲任务让步 */
#define configUSE_MUTEXES                       1       /**< 使用互斥锁 */
#define configQUEUE_REGISTRY_SIZE               8       /**< 队列注册表大小 */
#define configCHECK_FOR_STACK_OVERFLOW          2       /**< 检查堆栈溢出(方法2) */
#define configUSE_RECURSIVE_MUTEXES             1       /**< 使用递归互斥锁 */
#define configUSE_MALLOC_FAILED_HOOK            1       /**< 使用内存分配失败钩子 */
#define configUSE_APPLICATION_TASK_TAG          0       /**< 不使用任务标签 */
#define configUSE_COUNTING_SEMAPHORES           1       /**< 使用计数信号量 */
#define configGENERATE_RUN_TIME_STATS           0       /**< 不生成运行时统计 */

/*==============================================================================
 * 协程配置
 *============================================================================*/
#define configUSE_CO_ROUTINES                   0       /**< 不使用协程 */
#define configMAX_CO_ROUTINE_PRIORITIES         2       /**< 协程最大优先级 */

/*==============================================================================
 * 软件定时器配置
 *============================================================================*/
#define configUSE_TIMERS                        1       /**< 使用软件定时器 */
#define configTIMER_TASK_PRIORITY               2       /**< 定时器任务优先级 */
#define configTIMER_QUEUE_LENGTH                10      /**< 定时器命令队列长度 */
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE  /**< 定时器任务堆栈 */

/*==============================================================================
 * 中断配置
 *============================================================================*/
#define configKERNEL_INTERRUPT_PRIORITY         255     /**< 内核中断优先级(最低) */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    191     /**< 可屏蔽中断最高优先级 */

/*==============================================================================
 * 可选功能配置
 *============================================================================*/
#define configUSE_QUEUE_SETS                    0       /**< 不使用队列集 */
#define configUSE_TASK_NOTIFICATIONS            1       /**< 使用任务通知 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0       /**< 不使用优化任务选择 */
#define configUSE_TICKLESS_IDLE                 0       /**< 不使用无节拍空闲 */

/*==============================================================================
 * 断言配置
 *============================================================================*/
#define configASSERT(x) if((x)==0) { taskDISABLE_INTERRUPTS(); for(;;); }

/*==============================================================================
 * 钩子函数声明
 *============================================================================*/
extern void vApplicationMallocFailedHook(void);
extern void vApplicationStackOverflowHook(void *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);

/*==============================================================================
 * 中断处理函数映射
 *============================================================================*/
#define vPortSVCHandler         SVC_Handler
#define xPortPendSVHandler      PendSV_Handler
#define xPortSysTickHandler     SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
