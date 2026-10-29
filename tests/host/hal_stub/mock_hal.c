/*
 * Mock HAL + simulated MPU-6050 + scenario driver for the host tests.
 *
 * Unit tests call mock_reset() and drive the App/ modules directly.
 * Glue tests link this with a generated main (see gen_glue.py) and select a
 * scenario with SIM_SCENARIO=steady|burst|readfail|badid|writefail|stuckbus|deadbus; the UART
 * bytes and the HAL call trace are written to $SIM_UART / $SIM_TRACE at exit.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "glue_prelude.h"
#include "mock_hal.h"

I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;
GPIO_TypeDef      stub_gpiob;
static bool        gpiob_clk;     /* RCC clock gate of GPIOB */
void mock_rcc_gpiob_clk_enable(void) { gpiob_clk = true; }
TIM_HandleTypeDef  htim2;
mock_cfg_t         mock_cfg;

static char     trace[1u << 20];
static size_t   trace_len;
static uint8_t  uart[1u << 16];
static size_t   uart_len;
static int      n_writes, n_bursts, n_txn;
static int      n_deinit, n_reinit;

static void tr(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(trace + trace_len, sizeof trace - trace_len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof trace - trace_len) { fputs("mock trace overflow\n", stderr); abort(); }
    trace_len += (size_t)n;
}

bus_sim_t g_bus;
int       mock_gpio_glitches, mock_gpio_deinits;
uint16_t  mock_gpio_deinit_pins;
static uint16_t gpio_odr, gpio_out;     /* per-pin: output data bit, "pin is an output" */

void mock_reset(void)
{
    memset(&g_bus, 0, sizeof g_bus);
    bus_sim_start(&g_bus);
    gpio_odr = gpio_out = 0;
    mock_gpio_glitches = mock_gpio_deinits = 0;
    mock_gpio_deinit_pins = 0;
    memset(&mock_cfg, 0, sizeof mock_cfg);
    mock_cfg.who_am_i = 0x68;
    memset(trace, 0, sizeof trace);
    trace_len = 0;
    uart_len  = 0;
    n_writes  = 0;
    n_bursts  = 0;
    n_txn     = 0;
    gpiob_clk = false;
}

const char *mock_trace(void) { return trace; }

/* Does this I2C transaction fail because the bus is wedged?  A confused slave (or the
 * poison_at_txn fault) grabs SDA when a transaction starts. */
static bool txn_fails_on_bus(void)
{
    n_txn++;
    if (mock_cfg.poison_relapse && mock_cfg.poison_at_txn && n_txn == mock_cfg.poison_at_txn) {
        g_bus.confused = true;                 /* transaction fails, SDA stays high: only a STOP helps */
        return true;
    }
    if (g_bus.confused || (mock_cfg.poison_at_txn && n_txn == mock_cfg.poison_at_txn)) {
        g_bus.confused = false;
        g_bus.sda_hold_edges = g_bus.falls + 5;
    }
    return !bus_sim_get_sda(&g_bus);
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c) { (void)hi2c; n_deinit++; tr("I2C_DEINIT\n"); return HAL_OK; }
void MX_I2C1_Init(void) { n_reinit++; tr("I2C_INIT\n"); }

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c;
    tr("R dev=0x%02X reg=0x%02X len=%u to=%u\n", DevAddress, MemAddress, Size, (unsigned)Timeout);
    if (txn_fails_on_bus()) return HAL_ERROR;          /* SDA held low by a stuck slave: arbitration lost */
    if (MemAddSize != I2C_MEMADD_SIZE_8BIT) return HAL_ERROR;

    if (MemAddress == 0x75 && Size == 1) {
        if (mock_cfg.fail_id_read) return HAL_ERROR;
        pData[0] = mock_cfg.who_am_i;
        return HAL_OK;
    }
    if (MemAddress == 0x3B) {
        n_bursts++;
        if (mock_cfg.fail_burst_from && n_bursts >= mock_cfg.fail_burst_from &&
            n_bursts <= mock_cfg.fail_burst_to)
            return HAL_ERROR;
        for (unsigned i = 0; i < Size; i++)             /* deterministic, varied payload */
            pData[i] = (uint8_t)(n_bursts * 7u + i * 13u);
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c;
    if (MemAddSize != I2C_MEMADD_SIZE_8BIT || Size != 1) return HAL_ERROR;
    tr("W dev=0x%02X reg=0x%02X val=0x%02X to=%u\n", DevAddress, MemAddress, pData[0], (unsigned)Timeout);
    if (txn_fails_on_bus()) return HAL_ERROR;
    n_writes++;
    return (mock_cfg.fail_write_n && n_writes == mock_cfg.fail_write_n) ? HAL_ERROR : HAL_OK;
}

void HAL_Delay(uint32_t Delay) { tr("DELAY %u\n", (unsigned)Delay); }

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim) { (void)htim; tr("TIM_START\n"); return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)huart;
    tr("TX len=%u to=%u\n", Size, (unsigned)Timeout);
    if (uart_len + Size > sizeof uart) abort();
    memcpy(uart + uart_len, pData, Size);
    uart_len += Size;
    return HAL_OK;
}

/* On real hardware, touching a GPIO port whose RCC clock is off silently does nothing. */
static void gpio_clk_check(const GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOB && !gpiob_clk) { fputs("mock GPIO: GPIOB accessed while its RCC clock is off\n", stderr); abort(); }
}

/* ---- mock GPIO: only SCL (PB8) and SDA (PB9) exist, wired to g_bus ---- */
static void gpio_apply(uint16_t pin)
{
    bool pulled_low = (gpio_out & pin) && !(gpio_odr & pin);   /* open-drain output driving 0 */
    if (pin == GPIO_PIN_8)      bus_sim_set_scl(&g_bus, !pulled_low);
    else if (pin == GPIO_PIN_9) bus_sim_set_sda(&g_bus, !pulled_low);
}

static void gpio_check(uint32_t mask)
{
    if (mask & ~(uint32_t)(GPIO_PIN_8 | GPIO_PIN_9)) { fputs("mock GPIO: unexpected pin\n", stderr); abort(); }
}

/* statement-level helper: validate, then iterate over the pins set in `mask` */
#define FOR_EACH_PIN(mask, p) \
    gpio_check(mask); \
    for (uint16_t p = 1; p; p = (uint16_t)(p << 1)) if ((mask) & p)

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    gpio_clk_check(GPIOx);
    FOR_EACH_PIN(GPIO_Init->Pin, p) {
        if (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) {
            if (!(gpio_odr & p)) mock_gpio_glitches++;          /* would pull the line low right now */
            gpio_out |= p;
        } else {
            gpio_out &= (uint16_t)~p;
        }
        gpio_apply(p);
    }
}

void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    gpio_clk_check(GPIOx);
    mock_gpio_deinits++;
    mock_gpio_deinit_pins = (uint16_t)GPIO_Pin;
    FOR_EACH_PIN(GPIO_Pin, p) {
        gpio_out &= (uint16_t)~p;
        gpio_odr &= (uint16_t)~p;
        gpio_apply(p);
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    gpio_clk_check(GPIOx);
    FOR_EACH_PIN(GPIO_Pin, p) {
        if (PinState == GPIO_PIN_SET) gpio_odr |= p; else gpio_odr &= (uint16_t)~p;
        gpio_apply(p);
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    gpio_clk_check(GPIOx);
    bool level = (GPIO_Pin == GPIO_PIN_8) ? bus_sim_get_scl(&g_bus) : bus_sim_get_sda(&g_bus);
    return level ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void Error_Handler(void)
{
    tr("ERROR_HANDLER\n");
    exit(42);                       /* real firmware spins forever with IRQs off */
}

/* ------------------------------ scenario driver ------------------------------ */

#define N_EVENTS        300
#define PASSES_BETWEEN  4           /* main-loop passes between DATA_RDY events */
#define DRAIN_PASSES    8

/* Unit-test binaries have no ISR; a glue binary's own definition overrides this. */
__attribute__((weak)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { (void)GPIO_Pin; }

static int      scen_burst;
static uint32_t lcg = 12345u;

static void fire(void)
{
    lcg = lcg * 1103515245u + 12345u;
    htim2.counter += 10000u + ((lcg >> 16) % 4u);   /* ~10 ms with 0..3 us jitter */
    HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
}

bool sim_continue(void)
{
    static int pass, fired, last_fire_pass;

    if (pass % PASSES_BETWEEN == 0 && fired < N_EVENTS) {
        /* burst scenario: 20 interrupts back-to-back while the main loop is "stalled" */
        int n = (scen_burst && fired == 50) ? 20 : 1;
        for (int i = 0; i < n && fired < N_EVENTS; i++, fired++)
            fire();
        last_fire_pass = pass;
    }
    pass++;
    return !(fired >= N_EVENTS && pass - last_fire_pass > DRAIN_PASSES);
}

static void dump(void)
{
    const char *u = getenv("SIM_UART"), *t = getenv("SIM_TRACE");
    FILE *f;
    if (u && (f = fopen(u, "wb"))) { fwrite(uart, 1, uart_len, f); fclose(f); }
    if (t && (f = fopen(t, "wb"))) { fwrite(trace, 1, trace_len, f); fclose(f); }
}

__attribute__((constructor)) static void sim_boot(void)
{
    const char *s = getenv("SIM_SCENARIO");
    mock_reset();
    htim2.counter = 0xFFFFF000u;    /* start just below 2^32 so the timestamp wraps early */
    if (!s) return;                 /* unit tests: nothing to configure */

    if      (!strcmp(s, "steady"))    { }
    else if (!strcmp(s, "burst"))     { scen_burst = 1; }
    else if (!strcmp(s, "readfail"))  { mock_cfg.fail_burst_from = 40; mock_cfg.fail_burst_to = 41; }
    else if (!strcmp(s, "badid"))     { mock_cfg.who_am_i = 0x70; }
    else if (!strcmp(s, "writefail")) { mock_cfg.fail_write_n = 3; }      /* the CONFIG (0x1A) write */
    else if (!strcmp(s, "stuckbus"))  { g_bus.sda_hold_edges = 5;  bus_sim_start(&g_bus); }   /* needs 5 clocks to let go */
    else if (!strcmp(s, "midbyte"))   { g_bus.confused = true; bus_sim_start(&g_bus); }       /* SDA high at boot, slave confused */
    else if (!strcmp(s, "wedge"))     { mock_cfg.poison_at_txn = 1; }                         /* reset hits during our own init */
    else if (!strcmp(s, "relapse"))   { mock_cfg.poison_at_txn = 1; mock_cfg.poison_relapse = 1; }   /* fails once, SDA high, confused */
    else if (!strcmp(s, "deadbus"))   { g_bus.sda_hold_edges = 60; bus_sim_start(&g_bus); }   /* never lets go, even over 3 attempts of 9 clocks */
    else { fprintf(stderr, "unknown SIM_SCENARIO '%s'\n", s); exit(2); }
    atexit(dump);
}
