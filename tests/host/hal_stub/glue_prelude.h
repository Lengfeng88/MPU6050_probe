#ifndef GLUE_PRELUDE_H
#define GLUE_PRELUDE_H

/*
 * Stands in for the CubeMX headers (main.h, i2c.h, tim.h, usart.h) at the top
 * of main.c, so the USER CODE blocks extracted by gen_glue.py compile on the host.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim2;

void Error_Handler(void);
void MX_I2C1_Init(void);      /* CubeMX-generated; mocked on the host */

/* Called once per main-loop pass (replaces `while (1)`); fires simulated
 * DATA_RDY interrupts and returns false when the scenario is over. */
bool sim_continue(void);

/* Defined by the main.c glue, called by the simulator in place of the EXTI ISR. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* GLUE_PRELUDE_H */
