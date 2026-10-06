/*
 * ms5837_dma.h
 *
 *  Created on: Dec 14, 2025
 *      Author: abdelrauf
 *  DMA-enabled version of MS5837 driver
 */

#ifndef MS5837_DMA_H
#define MS5837_DMA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// MS5837-30BA I2C Address
#define MS5837_ADDR         (0x76 << 1)

// Commands
#define MS5837_CMD_RESET    0x1E
#define MS5837_CMD_ADC_READ 0x00
#define MS5837_CMD_PROM_RD  0xA0

// OSR Selection
typedef enum {
    MS5837_OSR_256  = 0,
    MS5837_OSR_512  = 1,
    MS5837_OSR_1024 = 2,
    MS5837_OSR_2048 = 3,
    MS5837_OSR_4096 = 4,
    MS5837_OSR_8192 = 5
} MS5837_OSR;

// Error codes
typedef enum {
    MS5837_OK = 0,
    MS5837_ERR_I2C_TIMEOUT,
    MS5837_ERR_I2C_ERROR,
    MS5837_ERR_CRC_FAILED,
    MS5837_ERR_RESET_FAILED,
    MS5837_ERR_PROM_READ_FAILED,
    MS5837_ERR_CONVERSION_FAILED,
    MS5837_ERR_BUSY
} MS5837_Error;

// State machine states
typedef enum {
    MS5837_STATE_IDLE = 0,
    MS5837_STATE_RESET,
    MS5837_STATE_RESET_WAIT,
    MS5837_STATE_PROM_READ_CMD,
    MS5837_STATE_PROM_READ_DATA,
    MS5837_STATE_D1_CONVERT,
    MS5837_STATE_D1_WAIT,
    MS5837_STATE_D1_READ_CMD,
    MS5837_STATE_D1_READ_DATA,
    MS5837_STATE_D2_CONVERT,
    MS5837_STATE_D2_WAIT,
    MS5837_STATE_D2_READ_CMD,
    MS5837_STATE_D2_READ_DATA,
    MS5837_STATE_CALCULATE,
    MS5837_STATE_ERROR
} MS5837_State;

// Sensor structure
typedef struct {
    I2C_HandleTypeDef *hi2c;

    // Calibration coefficients
    uint16_t C[7];

    // Raw ADC values
    uint32_t D1;
    uint32_t D2;

    // Calculated values
    int32_t dT;
    int64_t OFF;
    int64_t SENS;

    // Final values
    float pressure;     // mbar
    float temperature;  // °C
    float depth;        // meters

    // Configuration
    MS5837_OSR osr;
    uint8_t conv_time;

    // State machine
    MS5837_State state;
    MS5837_Error last_error;
    uint8_t prom_index;
    uint32_t conversion_start_tick;

    // DMA buffers
    uint8_t tx_buffer[1];
    uint8_t rx_buffer[3];

    // Flags
    volatile bool transfer_complete;
    volatile bool data_ready;

} MS5837_t;

// Function prototypes
MS5837_Error MS5837_Init(MS5837_t *dev, I2C_HandleTypeDef *hi2c, MS5837_OSR osr);
MS5837_Error MS5837_StartReading(MS5837_t *dev);
void MS5837_Process(MS5837_t *dev);
bool MS5837_IsDataReady(MS5837_t *dev);
float MS5837_GetPressure(MS5837_t *dev);
float MS5837_GetTemperature(MS5837_t *dev);
float MS5837_GetDepth(MS5837_t *dev);
bool MS5837_IsConnected(MS5837_t *dev);
MS5837_State MS5837_GetState(MS5837_t *dev);

// Callback functions (call from HAL I2C callbacks)
void MS5837_I2C_TxCpltCallback(MS5837_t *dev);
void MS5837_I2C_RxCpltCallback(MS5837_t *dev);
void MS5837_I2C_ErrorCallback(MS5837_t *dev);

#endif // MS5837_DMA_H
