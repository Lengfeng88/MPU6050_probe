#ifndef APP_CLASSIFY_CYCCNT_H
#define APP_CLASSIFY_CYCCNT_H

#include <stdint.h>

/*
 * CPU cycle counter (Cortex-M7 DWT->CYCCNT) for timing code on the board.  On the host (tests) it
 * is a stub that always reads 0, so code that uses it still builds there.
 *   cyccnt_init();   once at start-up
 *   uint32_t t0 = cyccnt_read(); ... work ...; uint32_t cycles = cyccnt_read() - t0;
 * cycles / SystemCoreClock is the time in seconds.  uint32 subtraction is correct across a wrap.
 */
#if defined(__arm__)
#include "main.h"

static inline void cyccnt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;               /* unlock; needed on Cortex-M7 */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t cyccnt_read(void)
{
    return DWT->CYCCNT;
}
#else
static inline void cyccnt_init(void) {}
static inline uint32_t cyccnt_read(void) { return 0; }
#endif

#endif /* APP_CLASSIFY_CYCCNT_H */
