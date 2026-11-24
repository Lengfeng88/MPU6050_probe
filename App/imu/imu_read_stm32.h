#ifndef APP_IMU_IMU_READ_STM32_H
#define APP_IMU_IMU_READ_STM32_H

#include <stdint.h>

#include "imu/imu_read.h"
#include "imu/mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

extern imu_read_t g_imu_rd;     /* counters: read them with a debugger (see docs/m2_i2c.md) */

/** Set up the read engine for `dev` (call after the sensor is configured).  The DMA stream, its
 *  IRQ and the I2C IRQs are configured in HAL_I2C_MspInit() (Core/Src/i2c.c, USER CODE blocks). */
void imu_read_stm32_init(const mpu6050_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_IMU_READ_STM32_H */
