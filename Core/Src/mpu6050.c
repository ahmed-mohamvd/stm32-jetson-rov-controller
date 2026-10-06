#include "mpu6050.h"
#include <math.h>

#define RAD_TO_DEG  57.2957795f
#define DEG_TO_RAD  0.0174532925f
#define ALPHA       0.98f     // complementary filter coefficient

#define QMC5883L_REG_DATA_X_LSB   0x00U
#define QMC5883L_REG_STATUS       0x06U
#define QMC5883L_REG_CONTROL_1    0x09U
#define QMC5883L_REG_CONTROL_2    0x0AU
#define QMC5883L_REG_SET_RESET    0x0BU
#define QMC5883L_REG_CHIP_ID      0x0DU

#define HMC5883L_REG_CONFIG_A     0x00U
#define HMC5883L_REG_CONFIG_B     0x01U
#define HMC5883L_REG_MODE         0x02U
#define HMC5883L_REG_DATA_X_MSB   0x03U
#define HMC5883L_REG_STATUS       0x09U
#define HMC5883L_REG_ID_A         0x0AU

/* ---------- Low-level register write (blocking, only used at init) ---------- */
static HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef MPU6050_ReadReg(MPU6050_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, HAL_MAX_DELAY);
}

/* ---------- Init ---------- */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t address)
{
    dev->hi2c    = hi2c;
    dev->address = address;
    dev->pitch   = 0.0f;
    dev->roll    = 0.0f;
    dev->data_ready = 0;
    dev->filter_initialized = 0;
    dev->last_update_ms = HAL_GetTick();

    // Verify device with WHO_AM_I (should return 0x68)
    uint8_t who = 0;
    if (MPU6050_ReadReg(dev, MPU6050_REG_WHO_AM_I, &who) != HAL_OK) return HAL_ERROR;
    if (who != 0x68 && who != 0x72) return HAL_ERROR;   // 0x72 on some clones

    // Wake up (clear sleep bit, use internal 8 MHz oscillator)
    if (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x00) != HAL_OK) return HAL_ERROR;

    // Sample rate divider: 1 kHz / (1 + 7) = 125 Hz
    if (MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x07) != HAL_OK) return HAL_ERROR;

    // DLPF config: 44 Hz bandwidth
    if (MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x03) != HAL_OK) return HAL_ERROR;

    // Gyro: ±250°/s
    if (MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) return HAL_ERROR;

    // Accel: ±2g
    if (MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK) return HAL_ERROR;

    // Expose the HW-290 magnetometer connected to MPU6050 AUX_DA/AUX_CL.
    if (MPU6050_EnableAuxBypass(dev) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_EnableAuxBypass(MPU6050_t *dev)
{
    /* I2C_MST_EN must be clear before BYPASS_EN connects AUX I2C to I2C1. */
    if (MPU6050_WriteReg(dev, MPU6050_REG_USER_CTRL, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }

    if (MPU6050_WriteReg(dev, MPU6050_REG_INT_PIN_CFG, 0x02U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ---------- Trigger DMA read of all sensor registers ---------- */
HAL_StatusTypeDef MPU6050_ReadAll_DMA(MPU6050_t *dev)
{
    // Reads 14 bytes: AccelXYZ (6) + Temp (2) + GyroXYZ (6)
    return HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->address,
                                MPU6050_REG_ACCEL_XOUT_H,
                                I2C_MEMADD_SIZE_8BIT,
                                dev->raw_buf, 14);
}

/* ---------- Convert raw buffer to physical units & compute pitch ---------- */
void MPU6050_ProcessData(MPU6050_t *dev)
{
    if (!dev->data_ready) return;
    dev->data_ready = 0;

    // Parse big-endian raw values
    int16_t ax = (int16_t)((dev->raw_buf[0]  << 8) | dev->raw_buf[1]);
    int16_t ay = (int16_t)((dev->raw_buf[2]  << 8) | dev->raw_buf[3]);
    int16_t az = (int16_t)((dev->raw_buf[4]  << 8) | dev->raw_buf[5]);
    // bytes 6,7 are temperature (skipped)
    int16_t gx = (int16_t)((dev->raw_buf[8]  << 8) | dev->raw_buf[9]);
    int16_t gy = (int16_t)((dev->raw_buf[10] << 8) | dev->raw_buf[11]);
    int16_t gz = (int16_t)((dev->raw_buf[12] << 8) | dev->raw_buf[13]);

    // Convert to physical units
    dev->accel_x = ax / MPU6050_ACCEL_SENS_2G;
    dev->accel_y = ay / MPU6050_ACCEL_SENS_2G;
    dev->accel_z = az / MPU6050_ACCEL_SENS_2G;
    dev->gyro_x  = gx / MPU6050_GYRO_SENS_250DPS;
    dev->gyro_y  = gy / MPU6050_GYRO_SENS_250DPS;
    dev->gyro_z  = gz / MPU6050_GYRO_SENS_250DPS;

    // Time delta in seconds
    uint32_t now = HAL_GetTick();
    float dt = (now - dev->last_update_ms) / 1000.0f;
    dev->last_update_ms = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;   // sanity guard

    // Pitch & roll from accelerometer
    float pitch_acc = atan2f(-dev->accel_x,
                             sqrtf(dev->accel_y * dev->accel_y +
                                   dev->accel_z * dev->accel_z)) * RAD_TO_DEG;
    float roll_acc  = atan2f(dev->accel_y, dev->accel_z) * RAD_TO_DEG;

    // Seed from gravity so the first real packet is not slowly pulled from 0°.
    if (!dev->filter_initialized) {
        dev->pitch = pitch_acc;
        dev->roll = roll_acc;
        dev->filter_initialized = 1;
    } else {
        // Complementary filter (gyro integration + accel correction)
        dev->pitch = ALPHA * (dev->pitch + dev->gyro_y * dt) +
                     (1.0f - ALPHA) * pitch_acc;
        dev->roll  = ALPHA * (dev->roll + dev->gyro_x * dt) +
                     (1.0f - ALPHA) * roll_acc;
    }
}

float MPU6050_GetPitch(MPU6050_t *dev) { return dev->pitch; }
float MPU6050_GetRoll (MPU6050_t *dev) { return dev->roll;  }

/* ---------- DMA complete callback hook ---------- */
void MPU6050_DMA_CompleteCallback(MPU6050_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (hi2c == dev->hi2c) {
        dev->data_ready = 1;
    }
}

HAL_StatusTypeDef HW290_Magnetometer_Init(HW290_Magnetometer_t *mag,
                                          I2C_HandleTypeDef *hi2c)
{
    uint8_t value;
    uint8_t id[3];

    if ((mag == NULL) || (hi2c == NULL)) {
        return HAL_ERROR;
    }

    mag->hi2c = hi2c;
    mag->type = HW290_MAG_NONE;
    mag->address = 0U;
    mag->field_x = 0.0f;
    mag->field_y = 0.0f;
    mag->field_z = 0.0f;
    mag->yaw_deg = 0.0f;

    /* Most recent HW-290 boards use QMC5883L at 0x0D. */
    if (HAL_I2C_IsDeviceReady(hi2c, HW290_QMC5883L_ADDR, 2U, 20U) == HAL_OK) {
        value = 0U;
        if (HAL_I2C_Mem_Read(hi2c, HW290_QMC5883L_ADDR,
                             QMC5883L_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        /* QMC5883L reports 0xFF; accept responding clones as well. */
        value = 0x80U;
        if (HAL_I2C_Mem_Write(hi2c, HW290_QMC5883L_ADDR,
                              QMC5883L_REG_CONTROL_2, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }
        HAL_Delay(10U);

        value = 0x01U;
        if (HAL_I2C_Mem_Write(hi2c, HW290_QMC5883L_ADDR,
                              QMC5883L_REG_SET_RESET, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        /* OSR=512, range=8 gauss, ODR=50 Hz, continuous measurement. */
        value = 0x15U;
        if (HAL_I2C_Mem_Write(hi2c, HW290_QMC5883L_ADDR,
                              QMC5883L_REG_CONTROL_1, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        mag->type = HW290_MAG_QMC5883L;
        mag->address = HW290_QMC5883L_ADDR;
        return HAL_OK;
    }

    /* Older genuine GY-87 boards use HMC5883L at 0x1E. */
    if (HAL_I2C_IsDeviceReady(hi2c, HW290_HMC5883L_ADDR, 2U, 20U) == HAL_OK) {
        if (HAL_I2C_Mem_Read(hi2c, HW290_HMC5883L_ADDR,
                             HMC5883L_REG_ID_A, I2C_MEMADD_SIZE_8BIT,
                             id, sizeof(id), 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        value = 0x70U; /* 8-sample average, 15 Hz, normal measurement. */
        if (HAL_I2C_Mem_Write(hi2c, HW290_HMC5883L_ADDR,
                              HMC5883L_REG_CONFIG_A, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        value = 0x20U; /* ±1.3 gauss gain. */
        if (HAL_I2C_Mem_Write(hi2c, HW290_HMC5883L_ADDR,
                              HMC5883L_REG_CONFIG_B, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        value = 0x00U; /* Continuous measurement. */
        if (HAL_I2C_Mem_Write(hi2c, HW290_HMC5883L_ADDR,
                              HMC5883L_REG_MODE, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        mag->type = HW290_MAG_HMC5883L;
        mag->address = HW290_HMC5883L_ADDR;
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef HW290_Magnetometer_Read(HW290_Magnetometer_t *mag)
{
    uint8_t status;
    uint8_t data[6];
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;

    if ((mag == NULL) || (mag->hi2c == NULL)) {
        return HAL_ERROR;
    }

    if (mag->type == HW290_MAG_QMC5883L) {
        if (HAL_I2C_Mem_Read(mag->hi2c, mag->address,
                             QMC5883L_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
                             &status, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & 0x01U) == 0U) {
            return HAL_BUSY;
        }
        if (HAL_I2C_Mem_Read(mag->hi2c, mag->address,
                             QMC5883L_REG_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT,
                             data, sizeof(data), 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        rawX = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
        rawY = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
        rawZ = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    } else if (mag->type == HW290_MAG_HMC5883L) {
        if (HAL_I2C_Mem_Read(mag->hi2c, mag->address,
                             HMC5883L_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
                             &status, 1U, 20U) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & 0x01U) == 0U) {
            return HAL_BUSY;
        }
        if (HAL_I2C_Mem_Read(mag->hi2c, mag->address,
                             HMC5883L_REG_DATA_X_MSB, I2C_MEMADD_SIZE_8BIT,
                             data, sizeof(data), 20U) != HAL_OK) {
            return HAL_ERROR;
        }

        /* HMC register order is X, Z, Y and each value is big-endian. */
        rawX = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
        rawZ = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        rawY = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    } else {
        return HAL_ERROR;
    }

    if ((rawX == 0) && (rawY == 0) && (rawZ == 0)) {
        return HAL_ERROR;
    }

    mag->field_x = (float)rawX;
    mag->field_y = (float)rawY;
    mag->field_z = (float)rawZ;
    return HAL_OK;
}

float HW290_Magnetometer_ComputeYaw(HW290_Magnetometer_t *mag,
                                    float roll_deg, float pitch_deg)
{
    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;
    float cosRoll = cosf(roll);
    float sinRoll = sinf(roll);
    float cosPitch = cosf(pitch);
    float sinPitch = sinf(pitch);
    float horizontalX;
    float horizontalY;
    float yaw;

    horizontalX = mag->field_x * cosPitch + mag->field_z * sinPitch;
    horizontalY = mag->field_x * sinRoll * sinPitch +
                  mag->field_y * cosRoll -
                  mag->field_z * sinRoll * cosPitch;

    yaw = atan2f(horizontalY, horizontalX) * RAD_TO_DEG;
    if (yaw < 0.0f) {
        yaw += 360.0f;
    }

    mag->yaw_deg = yaw;
    return yaw;
}
