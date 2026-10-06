#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"   // change to f1xx/f3xx/etc. for your MCU family
#include <stdint.h>

/* ---------- I2C Address ---------- */
#define MPU6050_ADDR          (0x68 << 1)   // HAL expects 8-bit address
#define MPU6050_ADDR_ALT      (0x69 << 1)   // if AD0 pin is high

/* ---------- Register Map ---------- */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_INT_PIN_CFG    0x37
#define MPU6050_REG_USER_CTRL      0x6A
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* HW-290/GY-87 magnetometer variants found on the MPU6050 AUX bus. */
#define HW290_QMC5883L_ADDR        (0x0D << 1)
#define HW290_HMC5883L_ADDR        (0x1E << 1)

/* ---------- Sensitivity ---------- */
#define MPU6050_ACCEL_SENS_2G     16384.0f   // LSB/g at ±2g
#define MPU6050_GYRO_SENS_250DPS  131.0f     // LSB/(°/s) at ±250°/s

/* ---------- Data Structure ---------- */
typedef struct {
    I2C_HandleTypeDef *hi2c;     // any I2C handle (hi2c1, hi2c2, ...)
    uint8_t  address;            // 8-bit I2C address
    uint8_t  raw_buf[14];        // DMA buffer: accel(6) + temp(2) + gyro(6)

    // Processed sensor data
    float accel_x, accel_y, accel_z;   // in g
    float gyro_x,  gyro_y,  gyro_z;    // in °/s

    // Orientation
    float pitch;                  // in degrees
    float roll;                   // in degrees

    // Internal timing
    uint32_t last_update_ms;
    uint8_t  data_ready;          // set by DMA complete callback
    uint8_t  filter_initialized;  // first sample seeds roll/pitch directly
} MPU6050_t;

typedef enum {
    HW290_MAG_NONE = 0,
    HW290_MAG_QMC5883L,
    HW290_MAG_HMC5883L
} HW290_MagType_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    HW290_MagType_t type;
    uint8_t address;
    float field_x;
    float field_y;
    float field_z;
    float yaw_deg;
} HW290_Magnetometer_t;

/* ---------- Public API ---------- */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef MPU6050_ReadAll_DMA(MPU6050_t *dev);
HAL_StatusTypeDef MPU6050_EnableAuxBypass(MPU6050_t *dev);
void              MPU6050_ProcessData(MPU6050_t *dev);
float             MPU6050_GetPitch(MPU6050_t *dev);
float             MPU6050_GetRoll(MPU6050_t *dev);

/* Call this from HAL_I2C_MemRxCpltCallback */
void MPU6050_DMA_CompleteCallback(MPU6050_t *dev, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef HW290_Magnetometer_Init(HW290_Magnetometer_t *mag,
                                          I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HW290_Magnetometer_Read(HW290_Magnetometer_t *mag);
float HW290_Magnetometer_ComputeYaw(HW290_Magnetometer_t *mag,
                                    float roll_deg, float pitch_deg);

#endif /* MPU6050_H */
