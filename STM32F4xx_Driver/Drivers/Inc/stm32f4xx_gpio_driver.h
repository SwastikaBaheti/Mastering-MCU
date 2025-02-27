#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f4xx.h"

/*Configuration structure for a GPIO pin*/

typedef struct{
	uint8_t GPIO_PinNumber;			//From @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			//From @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//From @GGPIO_OUTPUT_SPEEDS
	uint8_t GPIO_PinPuPdControl;	//From @GPIO_PUPD_CONFIG
	uint8_t GPIO_PinOPType;			//From @GPIO_OUTPUT_TYPES
	uint8_t GPIO_PinAltFuncMode;	//From @GPIO_ALT_FUNC
}GPIO_PinConfig_t;

/*Handle structure for a GPIO pin*/

typedef struct{
	GPIO_RegDef_t *pGPIOx;		//Base address of the GPIO port to which the GPIO pin belongs to
	GPIO_PinConfig_t GPIOPinConfig;		//Pin Configuration of the GPIO pin
}GPIO_Handle_t;

/*
 * Prototype for the APIs supported by the GPIO peripheral
*/

/*GPIO Peripheral clock setup*/
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t clkValue);

/*Initialize or De-initialize GPIO*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Reading data from the GPIO Input pin*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber);

/*Reading data from the GPIO Input Port*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*Writing data to the GPIO Output Pin*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber, uint8_t value);

/*Writing data to the GPIO Output Port*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/*Toggle a GPIO Output pin*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber);

/*Configuring the interrupt for a GPIO*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue);

/*Handling the interrupt triggered from the GPIO*/
void GPIO_IRQHandling(uint8_t GPIOPinNumber);


/*
 * User friendly macros
*/

/*@GPIO_PIN_MODES
 * GPIO pin modes*/
#define INPUT_MODE			0
#define OUTPUT_MODE			1
#define ALT_FUNC_MODE		2
#define ANALOG_MODE			3
#define IN_FE_MODE			4
#define IN_RE_MODE			5
#define IN_RFE_MODE			6

/*@GPIO_OUTPUT_TYPES
 * GPIO Output types*/
#define OUT_PUSH_PULL		0
#define OUT_OPEN_DRAIN		1

/*@GPIO_OUTPUT_SPEEDS
 * GPIO Output Speed types*/
#define LOW_SPEED			0
#define MED_SPEED			1
#define HIGH_SPEED			2
#define VHIGH_SPEED			3

/*@GPIO_PUPD_CONFIG
 * GPIO PU-PD Configuration*/
#define NO_PUPD				0
#define ONLY_PU				1
#define ONLY_PD				2

/*@GPIO_PIN_NUMBERS
 * GPIO Pin Numbers*/
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*@GPIO_ALT_FUNC
 * GPIO Alternate Functionality*/
#define AF0			0
#define AF1			1
#define AF2 		2
#define AF3			3
#define AF4			4
#define AF5			5
#define AF6			6
#define AF7 		7
#define AF8			8
#define AF9			9
#define AF10		10
#define AF11		11
#define AF12		12
#define AF13 		13
#define AF14		14
#define AF15		15

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
