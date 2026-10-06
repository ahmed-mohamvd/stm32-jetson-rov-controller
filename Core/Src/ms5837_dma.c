/*
 * ms5837_dma.c
 *
 *  Created on: Dec 14, 2025
 *      Author: abdelrauf
 *  DMA-enabled version of MS5837 driver
 */

#include "ms5837_dma.h"

// CRC-4 calculation for PROM validation
static uint8_t MS5837_CRC4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;
    uint8_t n_bit;

    n_prom[0] = n_prom[0] & 0x0FFF;
    n_prom[7] = 0;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) {
            n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = (n_rem >> 12) & 0x000F;
    return (n_rem ^ 0x00);
}

// Calculate temperature compensated pressure
static void MS5837_Calculate(MS5837_t *dev) {
    // First order temperature
    dev->dT = (int32_t)dev->D2 - ((int32_t)dev->C[5] * 256);
    int32_t TEMP = 2000 + ((int64_t)dev->dT * dev->C[6]) / 8388608LL;

    // First order pressure
    dev->OFF = (int64_t)dev->C[2] * 65536LL + ((int64_t)dev->C[4] * dev->dT) / 128LL;
    dev->SENS = (int64_t)dev->C[1] * 32768LL + ((int64_t)dev->C[3] * dev->dT) / 256LL;

    // Second order temperature compensation
    int64_t Ti = 0, OFFi = 0, SENSi = 0;

    if (TEMP < 2000) {
        Ti = (3 * (int64_t)dev->dT * (int64_t)dev->dT) / 8589934592LL;
        OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;

        if (TEMP < -1500) {
            OFFi += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENSi += 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    } else {
        Ti = (2 * (int64_t)dev->dT * (int64_t)dev->dT) / 137438953472LL;
        OFFi = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENSi = 0;
    }

    dev->OFF -= OFFi;
    dev->SENS -= SENSi;
    TEMP -= Ti;

    // Final calculations
    int32_t P = (((int64_t)dev->D1 * dev->SENS / 2097152LL) - dev->OFF) / 8192LL;

    dev->temperature = TEMP / 100.0f;
    dev->pressure = P / 10.0f;
    dev->depth = (dev->pressure - 1013.25f) / 100.0f;
}

// Check if sensor is connected
bool MS5837_IsConnected(MS5837_t *dev) {
    return (HAL_I2C_IsDeviceReady(dev->hi2c, MS5837_ADDR, 3, 100) == HAL_OK);
}

// Initialize the sensor (blocking mode for initial setup)
MS5837_Error MS5837_Init(MS5837_t *dev, I2C_HandleTypeDef *hi2c, MS5837_OSR osr) {
    dev->hi2c = hi2c;
    dev->osr = osr;
    dev->last_error = MS5837_OK;
    dev->state = MS5837_STATE_IDLE;
    dev->transfer_complete = false;
    dev->data_ready = false;

    // Set conversion time based on OSR
    const uint8_t conv_times[] = {1, 2, 3, 5, 10, 20};
    dev->conv_time = conv_times[osr];

    // Check if device is connected
    if (!MS5837_IsConnected(dev)) {
        dev->last_error = MS5837_ERR_I2C_TIMEOUT;
        return MS5837_ERR_I2C_TIMEOUT;
    }

    // Reset sensor (blocking)
    uint8_t cmd = MS5837_CMD_RESET;
    if (HAL_I2C_Master_Transmit(dev->hi2c, MS5837_ADDR, &cmd, 1, 100) != HAL_OK) {
        dev->last_error = MS5837_ERR_RESET_FAILED;
        return MS5837_ERR_RESET_FAILED;
    }
    HAL_Delay(10);

    // Read the sensor's seven 16-bit PROM words (112 bits). CRC processing
    // uses an eighth scratch word set to zero, as required by the algorithm.
    uint8_t data[2];
    for (uint8_t i = 0; i < 7; i++) {
        cmd = MS5837_CMD_PROM_RD + (i * 2);

        if (HAL_I2C_Master_Transmit(dev->hi2c, MS5837_ADDR, &cmd, 1, 100) != HAL_OK) {
            dev->last_error = MS5837_ERR_PROM_READ_FAILED;
            return MS5837_ERR_PROM_READ_FAILED;
        }
        HAL_Delay(1);

        if (HAL_I2C_Master_Receive(dev->hi2c, MS5837_ADDR, data, 2, 100) != HAL_OK) {
            dev->last_error = MS5837_ERR_PROM_READ_FAILED;
            return MS5837_ERR_PROM_READ_FAILED;
        }

        dev->C[i] = (data[0] << 8) | data[1];
    }

    // Verify CRC
    uint16_t prom_copy[8];
    for (uint8_t i = 0; i < 7; i++) {
        prom_copy[i] = dev->C[i];
    }
    prom_copy[7] = 0;

    uint8_t crc_read = (dev->C[0] >> 12) & 0x0F;
    uint8_t crc_calc = MS5837_CRC4(prom_copy);

    if (crc_read != crc_calc) {
        dev->last_error = MS5837_ERR_CRC_FAILED;
        return MS5837_ERR_CRC_FAILED;
    }

    return MS5837_OK;
}

// Start a new reading cycle (non-blocking)
MS5837_Error MS5837_StartReading(MS5837_t *dev) {
    if (dev->state != MS5837_STATE_IDLE) {
        return MS5837_ERR_BUSY;
    }

    dev->data_ready = false;
    dev->state = MS5837_STATE_D1_CONVERT;
    dev->transfer_complete = false;

    // Start D1 conversion
    const uint8_t d1_cmds[] = {0x40, 0x42, 0x44, 0x46, 0x48, 0x4A};
    dev->tx_buffer[0] = d1_cmds[dev->osr];

    if (HAL_I2C_Master_Transmit_DMA(dev->hi2c, MS5837_ADDR, dev->tx_buffer, 1) != HAL_OK) {
        dev->state = MS5837_STATE_ERROR;
        dev->last_error = MS5837_ERR_CONVERSION_FAILED;
        return MS5837_ERR_CONVERSION_FAILED;
    }

    return MS5837_OK;
}

// Process state machine (call from main loop)
void MS5837_Process(MS5837_t *dev) {
    const uint8_t d2_cmds[] = {0x50, 0x52, 0x54, 0x56, 0x58, 0x5A};

    switch (dev->state) {
        case MS5837_STATE_IDLE:
            // Nothing to do
            break;

        case MS5837_STATE_D1_CONVERT:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->conversion_start_tick = HAL_GetTick();
                dev->state = MS5837_STATE_D1_WAIT;
            }
            break;

        case MS5837_STATE_D1_WAIT:
            if ((HAL_GetTick() - dev->conversion_start_tick) >= (dev->conv_time + 2)) {
                dev->state = MS5837_STATE_D1_READ_CMD;
                dev->tx_buffer[0] = MS5837_CMD_ADC_READ;

                if (HAL_I2C_Master_Transmit_DMA(dev->hi2c, MS5837_ADDR, dev->tx_buffer, 1) != HAL_OK) {
                    dev->state = MS5837_STATE_ERROR;
                    dev->last_error = MS5837_ERR_CONVERSION_FAILED;
                }
            }
            break;

        case MS5837_STATE_D1_READ_CMD:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->state = MS5837_STATE_D1_READ_DATA;

                if (HAL_I2C_Master_Receive_DMA(dev->hi2c, MS5837_ADDR, dev->rx_buffer, 3) != HAL_OK) {
                    dev->state = MS5837_STATE_ERROR;
                    dev->last_error = MS5837_ERR_CONVERSION_FAILED;
                }
            }
            break;

        case MS5837_STATE_D1_READ_DATA:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->D1 = ((uint32_t)dev->rx_buffer[0] << 16) |
                          ((uint32_t)dev->rx_buffer[1] << 8) |
                          dev->rx_buffer[2];

                // Start D2 conversion
                dev->state = MS5837_STATE_D2_CONVERT;
                dev->tx_buffer[0] = d2_cmds[dev->osr];

                if (HAL_I2C_Master_Transmit_DMA(dev->hi2c, MS5837_ADDR, dev->tx_buffer, 1) != HAL_OK) {
                    dev->state = MS5837_STATE_ERROR;
                    dev->last_error = MS5837_ERR_CONVERSION_FAILED;
                }
            }
            break;

        case MS5837_STATE_D2_CONVERT:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->conversion_start_tick = HAL_GetTick();
                dev->state = MS5837_STATE_D2_WAIT;
            }
            break;

        case MS5837_STATE_D2_WAIT:
            if ((HAL_GetTick() - dev->conversion_start_tick) >= (dev->conv_time + 2)) {
                dev->state = MS5837_STATE_D2_READ_CMD;
                dev->tx_buffer[0] = MS5837_CMD_ADC_READ;

                if (HAL_I2C_Master_Transmit_DMA(dev->hi2c, MS5837_ADDR, dev->tx_buffer, 1) != HAL_OK) {
                    dev->state = MS5837_STATE_ERROR;
                    dev->last_error = MS5837_ERR_CONVERSION_FAILED;
                }
            }
            break;

        case MS5837_STATE_D2_READ_CMD:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->state = MS5837_STATE_D2_READ_DATA;

                if (HAL_I2C_Master_Receive_DMA(dev->hi2c, MS5837_ADDR, dev->rx_buffer, 3) != HAL_OK) {
                    dev->state = MS5837_STATE_ERROR;
                    dev->last_error = MS5837_ERR_CONVERSION_FAILED;
                }
            }
            break;

        case MS5837_STATE_D2_READ_DATA:
            if (dev->transfer_complete) {
                dev->transfer_complete = false;
                dev->D2 = ((uint32_t)dev->rx_buffer[0] << 16) |
                          ((uint32_t)dev->rx_buffer[1] << 8) |
                          dev->rx_buffer[2];

                dev->state = MS5837_STATE_CALCULATE;
            }
            break;

        case MS5837_STATE_CALCULATE:
            MS5837_Calculate(dev);
            dev->data_ready = true;
            dev->state = MS5837_STATE_IDLE;
            break;

        case MS5837_STATE_ERROR:
            // Stay in error state until StartReading is called again
            dev->state = MS5837_STATE_IDLE;
            break;

        default:
            dev->state = MS5837_STATE_IDLE;
            break;
    }
}

// Check if new data is ready
bool MS5837_IsDataReady(MS5837_t *dev) {
    return dev->data_ready;
}

// Get current state
MS5837_State MS5837_GetState(MS5837_t *dev) {
    return dev->state;
}

// Getter functions
float MS5837_GetPressure(MS5837_t *dev) {
    return dev->pressure;
}

float MS5837_GetTemperature(MS5837_t *dev) {
    return dev->temperature;
}

float MS5837_GetDepth(MS5837_t *dev) {
    return dev->depth;
}

// I2C Callback handlers - call these from HAL callbacks
void MS5837_I2C_TxCpltCallback(MS5837_t *dev) {
    dev->transfer_complete = true;
}

void MS5837_I2C_RxCpltCallback(MS5837_t *dev) {
    dev->transfer_complete = true;
}

void MS5837_I2C_ErrorCallback(MS5837_t *dev) {
    dev->state = MS5837_STATE_ERROR;
    dev->last_error = MS5837_ERR_I2C_ERROR;
}
