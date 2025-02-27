#ifndef INC_STM32F4XX_RCC_DRIVER_H_
#define INC_STM32F4XX_RCC_DRIVER_H_

#include "stm32f4xx.h"

/*
 * Prototype for the APIs supported by the RCC peripheral
*/

/*Returns the value of APB1 peripheral clock*/
uint32_t RCC_GetPCLK1Value(void);

/*Returns the value of APB2 peripheral clock*/
uint32_t RCC_GetPCLK2Value(void);

/*Returns the value of PLL generated clock*/
uint32_t RCC_GetPLLClk(void);

#endif /* INC_STM32F4XX_RCC_DRIVER_H_ */
