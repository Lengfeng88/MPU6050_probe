#ifndef APP_IMU_MPU6050_H
#define APP_IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW   0x68u   /* 7-bit address, AD0 pin low */
#define MPU6050_RAW_LEN        14u     /* ax ay az temp gx gy gz, 2 bytes each */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            addr7;          /* 7-bit I2C address */
} mpu6050_t;

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_BUS,                   /* an I2C transaction failed */
    MPU6050_ERR_ID,                    /* WHO_AM_I did not read 0x68 */
} mpu6050_status_t;

/**
 * Probe WHO_AM_I, reset the device and configure it for 100 Hz output
 * (DLPF_CFG=3, SMPLRT_DIV=9) with the DATA_RDY interrupt enabled.
 *
 * Blocking: takes about 110 ms because of the post-reset delays.
 * Returns on the first failing step; later steps are not attempted.
 */
mpu6050_status_t mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);

/**
 * Burst-read regs 0x3B..0x48 into raw[] with no conversion.
 * Returns true on success.  Blocking, 20 ms timeout.
 */
bool mpu6050_read_raw(const mpu6050_t *dev, uint8_t raw[MPU6050_RAW_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_MPU6050_H */
