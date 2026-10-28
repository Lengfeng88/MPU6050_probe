/*
 * Mock HAL + simulated MPU-6050 + scenario driver for the host tests.
 *
 * Unit tests call mock_reset() and drive the App/ modules directly.
 * Glue tests link this with a generated main (see gen_glue.py) and select a
 * scenario with SIM_SCENARIO=steady|burst|readfail|badid|writefail; the UART
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
TIM_HandleTypeDef  htim2;
mock_cfg_t         mock_cfg;

static char     trace[1u << 20];
static size_t   trace_len;
static uint8_t  uart[1u << 16];
static size_t   uart_len;
static int      n_writes, n_bursts;

static void tr(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(trace + trace_len, sizeof trace - trace_len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof trace - trace_len) { fputs("mock trace overflow\n", stderr); abort(); }
    trace_len += (size_t)n;
}

void mock_reset(void)
{
    memset(&mock_cfg, 0, sizeof mock_cfg);
    mock_cfg.who_am_i = 0x68;
    memset(trace, 0, sizeof trace);
    trace_len = 0;
    uart_len  = 0;
    n_writes  = 0;
    n_bursts  = 0;
}

const char *mock_trace(void) { return trace; }

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)hi2c;
    tr("R dev=0x%02X reg=0x%02X len=%u to=%u\n", DevAddress, MemAddress, Size, (unsigned)Timeout);
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
    else { fprintf(stderr, "unknown SIM_SCENARIO '%s'\n", s); exit(2); }
    atexit(dump);
}
