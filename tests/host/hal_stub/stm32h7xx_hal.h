#ifndef STM32H7XX_HAL_STUB_H
#define STM32H7XX_HAL_STUB_H

/*
 * Host-side stand-in for the handful of HAL symbols that App/ and the
 * main.c glue use.  Signatures match the real STM32H7 HAL.  Never used in
 * the firmware build: there the real header comes from Drivers/.
 */
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

typedef struct { int unused; }       I2C_HandleTypeDef;
typedef struct { int unused; }       UART_HandleTypeDef;
typedef struct { uint32_t counter; } TIM_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define GPIO_PIN_3            ((uint16_t)0x0008)
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->counter)

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
void HAL_Delay(uint32_t Delay);

#endif /* STM32H7XX_HAL_STUB_H */
