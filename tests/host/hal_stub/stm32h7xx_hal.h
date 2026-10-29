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
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->counter)

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
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

/* ---- GPIO ---- */
typedef struct { int unused; } GPIO_TypeDef;
typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

#define GPIO_PIN_3            ((uint16_t)0x0008)
#define GPIO_PIN_6            ((uint16_t)0x0040)
#define GPIO_PIN_7            ((uint16_t)0x0080)
#define GPIO_PIN_8            ((uint16_t)0x0100)
#define GPIO_PIN_9            ((uint16_t)0x0200)
#define GPIO_MODE_INPUT       0x00000000U
#define GPIO_MODE_OUTPUT_OD   0x00000011U
#define GPIO_NOPULL           0x00000000U
#define GPIO_PULLUP           0x00000001U
#define GPIO_SPEED_FREQ_LOW   0x00000000U

extern GPIO_TypeDef stub_gpiob;
#define GPIOB (&stub_gpiob)              /* address constant, like the real ((GPIO_TypeDef *)GPIOB_BASE) */
void mock_rcc_gpiob_clk_enable(void);
#define __HAL_RCC_GPIOB_CLK_ENABLE() mock_rcc_gpiob_clk_enable()

void          HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void          HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#endif /* STM32H7XX_HAL_STUB_H */
