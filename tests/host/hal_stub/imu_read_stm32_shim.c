/* Host replacement for App/imu/imu_read_stm32.c in the main.c glue test.
 * The glue test compares the HAL call trace with the pre-refactor firmware, so the "DMA" read is the
 * blocking mock read, finished before start() returns.  The state machine itself is tested in
 * test_units.c (imu_read). */
#include "imu/imu_read_stm32.h"

#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim2;

imu_read_t g_imu_rd;
static const mpu6050_t *s_dev;

static bool port_start(uint8_t *buf)
{
    if (mpu6050_read_raw(s_dev, buf)) imu_read_on_complete(&g_imu_rd);
    else                              imu_read_on_error(&g_imu_rd);
    return true;
}
static void     port_recover(void) {}
static uint32_t port_lock(void) { return 0; }
static void     port_unlock(uint32_t t) { (void)t; }
static uint32_t port_now_us(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

void imu_read_stm32_init(const mpu6050_t *dev)
{
    static uint8_t raw[32];
    static const imu_read_port_t port = { port_start, port_recover, port_lock, port_unlock, port_now_us, 0 };
    s_dev = dev;
    imu_read_init(&g_imu_rd, raw, &port, 3000);
}
