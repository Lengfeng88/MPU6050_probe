/* STM32H7 glue for imu_read.c: HAL DMA start, BASEPRI lock, bus recovery, IRQ handlers.
 * Target build only (the host tests replace this file with tests/host/hal_stub/imu_read_stm32_shim.c). */
#include "imu/imu_read_stm32.h"

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "bus/i2c_bus_clear_gpio.h"
#include "classify/cyccnt.h"
#include "measure/m0_pins.h"

/* Normal read: about 430 us at 400 kHz (1600 us at 100 kHz).  The DATA_RDY period is 9979 us. */
#ifndef IMU_READ_TIMEOUT_US
#define IMU_READ_TIMEOUT_US 3000u
#endif

/* Same level as the UART / DMA interrupts (see usart.c); EXTI3 (0) stays above both. */
#define IMU_READ_IRQ_PRIO 2u

/* DMA1 cannot reach DTCM, where .data / .bss / the stack live: the buffer goes to AXI SRAM. */
__attribute__((section(".dma_buf"), aligned(32)))
static uint8_t s_raw[32];

imu_read_t g_imu_rd;
static const mpu6050_t *s_dev;

static const i2c_bus_pins_t s_pins = { GPIOB, GPIO_PIN_8, GPIO_PIN_9 };

/* Debugger-visible: how often recovery ran, what the bus clear did, time spent in the I2C interrupts. */
volatile unsigned           g_imu_rd_clocks;          /* SCL pulses used by the last bus clear */
volatile i2c_bus_status_t   g_imu_rd_bus_status;
volatile uint32_t           g_imu_irq_calls;
volatile uint64_t           g_imu_irq_cycles;         /* CPU cycles inside the three handlers, nested EXTI3 included */
volatile uint32_t           g_imu_irq_cycles_max;

/* Optional fault injection (opt-in, for the recovery test): after every N-th started read, wait
 * M2B_FAULT_AFTER_US and switch the I2C peripheral off in the middle of the transfer.  The DMA never
 * finishes, the slave may be left holding SDA low, so the timeout and the bus clear are really needed. */
#ifdef M2B_FAULT_EVERY
#ifndef M2B_FAULT_AFTER_US
#define M2B_FAULT_AFTER_US 150u
#endif
volatile uint32_t g_imu_faults_injected;
static void fault_after_start(void)
{
    static uint32_t n;
    if (++n % (uint32_t)(M2B_FAULT_EVERY) != 0u) return;
    uint32_t cycles = (uint32_t)(((uint64_t)SystemCoreClock * M2B_FAULT_AFTER_US) / 1000000u);
    uint32_t t0 = cyccnt_read();
    while (cyccnt_read() - t0 < cycles) {}
    I2C1->CR1 &= ~I2C_CR1_PE;
    g_imu_faults_injected++;
}
#else
static void fault_after_start(void) {}
#endif

static bool port_start(uint8_t *buf)
{
    if (!mpu6050_read_raw_dma_start(s_dev, buf))
        return false;
    fault_after_start();
    return true;
}

static uint32_t port_lock(void)
{
    uint32_t old = __get_BASEPRI();
    uint32_t lvl = IMU_READ_IRQ_PRIO << (8u - __NVIC_PRIO_BITS);
    if (old == 0u || old > lvl)
        __set_BASEPRI(lvl);
    __ISB();
    return old;
}

static void port_unlock(uint32_t token)
{
    __set_BASEPRI(token);
}

static uint32_t port_now_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);       /* TIM2: 1 us per count, free running */
}

static void port_mark(bool busy)
{
    if (busy) m0_i2c_high(); else m0_i2c_low();
}

static void clear_pending(void)
{
    NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
}

/* Main loop only.  HAL_Delay (inside the bus clear) needs SysTick to run, so no BASEPRI here:
 * the DeInit disables the three IRQs in the NVIC (HAL_I2C_MspDeInit), which is what keeps the
 * abandoned transfer quiet; pending flags are cleared before and after. */
static void port_recover(void)
{
    unsigned clocks = 0;
    (void)HAL_I2C_DeInit(&hi2c1);
    clear_pending();
    g_imu_rd_bus_status = i2c_bus_clear_gpio_always(&s_pins, &clocks);
    g_imu_rd_clocks = clocks;
    MX_I2C1_Init();
    clear_pending();
}

void imu_read_stm32_init(const mpu6050_t *dev)
{
    static const imu_read_port_t port = {
        port_start, port_recover, port_lock, port_unlock, port_now_us, port_mark
    };
    s_dev = dev;
    imu_read_init(&g_imu_rd, s_raw, &port, IMU_READ_TIMEOUT_US);
}

/* ---- interrupt plumbing (DMA and these IRQs are set up in i2c.c, not in the .ioc) ---- */

static inline void irq_account(uint32_t t0)
{
    uint32_t c = cyccnt_read() - t0;
    g_imu_irq_calls++;
    g_imu_irq_cycles += c;
    if (c > g_imu_irq_cycles_max) g_imu_irq_cycles_max = c;
}

void I2C1_EV_IRQHandler(void)
{
    uint32_t t0 = cyccnt_read();
    HAL_I2C_EV_IRQHandler(&hi2c1);
    irq_account(t0);
}

void I2C1_ER_IRQHandler(void)
{
    uint32_t t0 = cyccnt_read();
    HAL_I2C_ER_IRQHandler(&hi2c1);
    irq_account(t0);
}

void DMA1_Stream1_IRQHandler(void)
{
    uint32_t t0 = cyccnt_read();
    HAL_DMA_IRQHandler(hi2c1.hdmarx);
    irq_account(t0);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
        imu_read_on_complete(&g_imu_rd);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
        imu_read_on_error(&g_imu_rd);
}
