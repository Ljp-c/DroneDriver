/**
 * @file error_code.c
 * @brief 错误码实现
 * @details 错误码描述字符串转换函数
 * @author 
 * @date 2025
 * @version 1.0.0
 * @copyright MIT License
 */

#include "error_code.h"

const char* error_code_get_string(error_code_t err_code)
{
    switch (err_code) {
        case ERR_SUCCESS:
            return "Success";
        case ERR_NULL_POINTER:
            return "Null pointer";
        case ERR_INVALID_PARAM:
            return "Invalid parameter";
        case ERR_TIMEOUT:
            return "Timeout";
        case ERR_BUSY:
            return "Device busy";
        case ERR_NOT_INITIALIZED:
            return "Not initialized";
        case ERR_OUT_OF_RANGE:
            return "Out of range";
        case ERR_MEMORY_ALLOC:
            return "Memory allocation failed";
        case ERR_INVALID_STATE:
            return "Invalid state";
            
        case ERR_MPU6050_INIT_FAILED:
            return "MPU6050 init failed";
        case ERR_MPU6050_ID_MISMATCH:
            return "MPU6050 ID mismatch";
        case ERR_MPU6050_I2C_READ:
            return "MPU6050 I2C read failed";
        case ERR_MPU6050_I2C_WRITE:
            return "MPU6050 I2C write failed";
        case ERR_MPU6050_DATA_NOT_READY:
            return "MPU6050 data not ready";
        case ERR_MPU6050_SELF_TEST_FAILED:
            return "MPU6050 self test failed";
            
        case ERR_BMP250_INIT_FAILED:
            return "BMP250 init failed";
        case ERR_BMP250_ID_MISMATCH:
            return "BMP250 ID mismatch";
        case ERR_BMP250_I2C_READ:
            return "BMP250 I2C read failed";
        case ERR_BMP250_I2C_WRITE:
            return "BMP250 I2C write failed";
        case ERR_BMP250_CALIB_READ:
            return "BMP250 calibration read failed";
        case ERR_BMP250_MEASURING:
            return "BMP250 measuring";
            
        case ERR_ADC_INIT_FAILED:
            return "ADC init failed";
        case ERR_ADC_CHANNEL_INVALID:
            return "ADC channel invalid";
        case ERR_ADC_CONVERSION_TIMEOUT:
            return "ADC conversion timeout";
        case ERR_ADC_CALIBRATION_FAILED:
            return "ADC calibration failed";
            
        case ERR_W25Q64_INIT_FAILED:
            return "W25Q64 init failed";
        case ERR_W25Q64_ID_MISMATCH:
            return "W25Q64 ID mismatch";
        case ERR_W25Q64_SPI_READ:
            return "W25Q64 SPI read failed";
        case ERR_W25Q64_SPI_WRITE:
            return "W25Q64 SPI write failed";
        case ERR_W25Q64_ERASE_FAILED:
            return "W25Q64 erase failed";
        case ERR_W25Q64_WRITE_PROTECTED:
            return "W25Q64 write protected";
        case ERR_W25Q64_ADDRESS_INVALID:
            return "W25Q64 address invalid";
        case ERR_FLASH_TIMEOUT:
            return "Flash timeout";
        case ERR_FLASH_WRITE_ENABLE_FAILED:
            return "Flash write enable failed";
        case ERR_FLASH_ID_MISMATCH:
            return "Flash ID mismatch";
        case ERR_FLASH_CRC_ERROR:
            return "Flash CRC error";
            
        case ERR_PWM_INIT_FAILED:
            return "PWM init failed";
        case ERR_PWM_CHANNEL_INVALID:
            return "PWM channel invalid";
        case ERR_PWM_DUTY_OUT_OF_RANGE:
            return "PWM duty out of range";
        case ERR_PWM_FREQUENCY_INVALID:
            return "PWM frequency invalid";
            
        case ERR_NRF24L01_INIT_FAILED:
            return "NRF24L01 init failed";
        case ERR_NRF24L01_SPI_READ:
            return "NRF24L01 SPI read failed";
        case ERR_NRF24L01_SPI_WRITE:
            return "NRF24L01 SPI write failed";
        case ERR_NRF24L01_TX_FAILED:
            return "NRF24L01 TX failed";
        case ERR_NRF24L01_RX_FAILED:
            return "NRF24L01 RX failed";
        case ERR_NRF24L01_IRQ_TIMEOUT:
            return "NRF24L01 IRQ timeout";
        case ERR_NRF24L01_DATA_INVALID:
            return "NRF24L01 data invalid";
        case ERR_NRF24L01_MAX_RT:
            return "NRF24L01 max retransmission";
        case ERR_NRF24L01_TIMEOUT:
            return "NRF24L01 timeout";
        case ERR_INVALID_PACKET:
            return "Invalid packet";
        case ERR_CRC_ERROR:
            return "CRC error";
            
        case ERR_PID_INIT_FAILED:
            return "PID init failed";
        case ERR_PID_PARAM_INVALID:
            return "PID parameter invalid";
        case ERR_PID_OUTPUT_SATURATION:
            return "PID output saturation";
            
        case ERR_I2C_BUS_BUSY:
            return "I2C bus busy";
        case ERR_I2C_BUS_ERROR:
            return "I2C bus error";
        case ERR_I2C_NACK:
            return "I2C NACK";
        case ERR_SPI_BUS_BUSY:
            return "SPI bus busy";
        case ERR_SPI_BUS_ERROR:
            return "SPI bus error";
        case ERR_SPI_INIT_FAILED:
            return "SPI init failed";
        case ERR_MUTEX_CREATE_FAILED:
            return "Mutex create failed";
            
        case ERR_NOT_ARMED:
            return "Not armed";
        case ERR_ALREADY_FLYING:
            return "Already flying";
        case ERR_NOT_FLYING:
            return "Not flying";
        case ERR_SENSOR_NOT_READY:
            return "Sensor not ready";
        case ERR_BATTERY_LOW:
            return "Battery low";
            
        case ERR_SYSTEM_CRITICAL:
            return "System critical error";
        case ERR_TASK_CREATE_FAILED:
            return "Task create failed";
        case ERR_SEMAPHORE_CREATE_FAILED:
            return "Semaphore create failed";
        case ERR_QUEUE_CREATE_FAILED:
            return "Queue create failed";
            
        default:
            return "Unknown error";
    }
}
