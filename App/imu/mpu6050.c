#include "imu/mpu6050.h"

/* Register map (MPU-6000/6050 register map, rev 4.2) */
#define REG_SMPLRT_DIV     0x19u
#define REG_CONFIG         0x1Au
#define REG_INT_ENABLE     0x38u
#define REG_ACCEL_XOUT_H   0x3Bu   /* start of the 14-byte burst */
#define REG_PWR_MGMT_1     0x6Bu
#define REG_WHO_AM_I       0x75u

#define WHO_AM_I_VALUE     0x68u

#define PWR_DEVICE_RESET   0x80u
#define PWR_CLKSEL_XGYRO   0x01u   /* SLEEP cleared, clock = X-axis gyro PLL */
#define CFG_DLPF_CFG_3     0x03u
#define SMPLRT_DIV_100HZ   9u      /* 1 kHz / (1 + 9) = 100 Hz */
#define INT_DATA_RDY_EN    0x01u

#define TIMEOUT_CFG_MS     100u
#define TIMEOUT_BURST_MS   20u
#define DELAY_AFTER_RESET_MS   100u
#define DELAY_AFTER_WAKE_MS    10u

static uint16_t dev_addr(const mpu6050_t *dev)
{
    return (uint16_t)(dev->addr7 << 1);
}

static bool write_reg(const mpu6050_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev_addr(dev), reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, TIMEOUT_CFG_MS) == HAL_OK;
}

mpu6050_status_t mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c  = hi2c;
    dev->addr7 = addr7;

    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(dev->hi2c, dev_addr(dev), REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &id, 1, TIMEOUT_CFG_MS) != HAL_OK)
        return MPU6050_ERR_BUS;
    if (id != WHO_AM_I_VALUE)
        return MPU6050_ERR_ID;

    if (!write_reg(dev, REG_PWR_MGMT_1, PWR_DEVICE_RESET))
        return MPU6050_ERR_BUS;
    HAL_Delay(DELAY_AFTER_RESET_MS);

    if (!write_reg(dev, REG_PWR_MGMT_1, PWR_CLKSEL_XGYRO))
        return MPU6050_ERR_BUS;
    HAL_Delay(DELAY_AFTER_WAKE_MS);

    if (!write_reg(dev, REG_CONFIG, CFG_DLPF_CFG_3))
        return MPU6050_ERR_BUS;
    if (!write_reg(dev, REG_SMPLRT_DIV, SMPLRT_DIV_100HZ))
        return MPU6050_ERR_BUS;
    if (!write_reg(dev, REG_INT_ENABLE, INT_DATA_RDY_EN))   /* last: interrupts start here */
        return MPU6050_ERR_BUS;

    return MPU6050_OK;
}

bool mpu6050_read_raw(const mpu6050_t *dev, uint8_t raw[MPU6050_RAW_LEN])
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev_addr(dev), REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                            raw, MPU6050_RAW_LEN, TIMEOUT_BURST_MS) == HAL_OK;
}
