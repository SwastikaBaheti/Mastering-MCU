#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>
#include <stddef.h>

/*Base Addresses of FLASH and SRAM memories*/

#define FLASH_BASE_ADDR				0x08000000UL		/*Base Addresses of FLASH memory*/
#define SRAM1_BASE_ADDR				0x20000000UL		/*Base Addresses of SRAM1 memory*/
#define SRAM2_BASE_ADDR				0x2001C000UL		/*Base Addresses of SRAM2 memory*/
#define ROM_BASE_ADDR				0x1FFF0000UL		/*Base Addresses of System memory (ROM)*/

/*Base Addresses of Peripherals*/

#define PERIPH_BASE_ADDR			0x40000000UL		/*Base Addresses of Peripheral Registers*/
#define APB1_PERIPH_ADDR			PERIPH_BASE_ADDR	/*Base Addresses of APB1 bus*/
#define APB2_PERIPH_ADDR			0x40010000UL		/*Base Addresses of APB2 bus*/
#define AHB1_PERIPH_ADDR			0x40020000UL		/*Base Addresses of AHB1 bus*/
#define AHB2_PERIPH_ADDR			0x50000000UL		/*Base Addresses of AHB2 bus*/

/*Base Addresses of AHB1 peripherals*/

#define GPIOA_BASE_ADDR				0x40020000UL		/*Base Addresses of GPIO Port A*/
#define GPIOB_BASE_ADDR				0x40020400UL		/*Base Addresses of GPIO Port B*/
#define GPIOC_BASE_ADDR				0x40020800UL		/*Base Addresses of GPIO Port C*/
#define GPIOD_BASE_ADDR				0x40020C00UL		/*Base Addresses of GPIO Port D*/
#define GPIOE_BASE_ADDR				0x40021000UL		/*Base Addresses of GPIO Port E*/
#define GPIOF_BASE_ADDR				0x40021400UL		/*Base Addresses of GPIO Port F*/
#define GPIOG_BASE_ADDR				0x40021800UL		/*Base Addresses of GPIO Port G*/
#define GPIOH_BASE_ADDR				0x40021C00UL		/*Base Addresses of GPIO Port H*/
#define GPIOI_BASE_ADDR				0x40022000UL		/*Base Addresses of GPIO Port I*/
#define RCC_BASE_ADDR				0x40023800UL		/*Base Addresses of Reset and Clock Control Register*/

/*Base Addresses of APB1 peripherals*/

#define SPI2_BASE_ADDR				0x40003800UL		/*Base Addresses of SPI2 Peripheral*/
#define SPI3_BASE_ADDR				0x40003C00UL		/*Base Addresses of SPI3 Peripheral*/

#define USART2_BASE_ADDR			0x40004400UL		/*Base Addresses of USART2 Peripheral*/
#define USART3_BASE_ADDR			0x40004800UL		/*Base Addresses of USART3 Peripheral*/
#define UART4_BASE_ADDR				0x40004C00UL		/*Base Addresses of UART4 Peripheral*/
#define UART5_BASE_ADDR				0x40005000UL		/*Base Addresses of UART5 Peripheral*/

#define I2C1_BASE_ADDR				0x40005400UL		/*Base Addresses of I2C1 Peripheral*/
#define I2C2_BASE_ADDR				0x40005800UL		/*Base Addresses of I2C2 Peripheral*/
#define I2C3_BASE_ADDR				0x40005C00UL		/*Base Addresses of I2C3 Peripheral*/

/*Base Addresses of APB2 peripherals*/

#define SPI1_BASE_ADDR				0x40013000UL		/*Base Addresses of SPI1 Peripheral*/
#define SPI4_BASE_ADDR				0x40013400UL		/*Base Addresses of SPI4 Peripheral*/
#define SPI5_BASE_ADDR				0x40015000UL		/*Base Addresses of SPI5 Peripheral*/
#define SPI6_BASE_ADDR				0x40015400UL		/*Base Addresses of SPI6 Peripheral*/

#define USART1_BASE_ADDR			0x40011000UL		/*Base Addresses of USART1 Peripheral*/
#define USART6_BASE_ADDR			0x40011400UL		/*Base Addresses of USART6 Peripheral*/

#define EXTI_BASE_ADDR				0x40013C00UL		/*Base Addresses of EXTI Peripheral*/

#define SYSCFG_BASE_ADDR			0x40013800UL		/*Base Addresses of System Configuration Peripheral*/

/*Base Addresses of NVIC peripherals*/
#define NVIC_ISER0_ADDR				((volatile uint32_t *)0xE000E100UL)		/*Base Addresses of NVIC Interrupt Set Register*/
#define NVIC_ISER1_ADDR				((volatile uint32_t*)0xE000E104UL)		/*Base Addresses of NVIC Interrupt Set Register*/
#define NVIC_ISER2_ADDR				((volatile uint32_t*)0xE000E108UL)		/*Base Addresses of NVIC Interrupt Set Register*/

#define NVIC_ICER0_ADDR				((volatile uint32_t*)0xE000E180UL)		/*Base Addresses of NVIC Interrupt Clear Register*/
#define NVIC_ICER1_ADDR				((volatile uint32_t*)0xE000E184UL)		/*Base Addresses of NVIC Interrupt Clear Register*/
#define NVIC_ICER2_ADDR				((volatile uint32_t*)0xE000E188UL)		/*Base Addresses of NVIC Interrupt Clear Register*/

#define NVIC_IPR_ADDR				((volatile uint32_t*)0xE000E400UL)		/*Base Addresses of NVIC Interrupt Priority Registers*/

/*Structure peripheral definition for GPIO*/

typedef struct {
	uint32_t volatile MODER;
	uint32_t volatile OTYPER;
	uint32_t volatile OSPEEDR;
	uint32_t volatile PUPDR;
	uint32_t volatile IDR;
	uint32_t volatile ODR;
	uint32_t volatile BSRR;
	uint32_t volatile LCKR;
	uint32_t volatile AFRL;
	uint32_t volatile AFRH;
}GPIO_RegDef_t;

/*Type casted definition for all GPIOs*/

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

/*Structure peripheral definition for RCC*/

typedef struct {
	uint32_t volatile CR;
	uint32_t volatile PLLCFGR;
	uint32_t volatile CFGR;
	uint32_t volatile CIR;
	uint32_t volatile AHB1RSTR;
	uint32_t volatile AHB2RSTR;
	uint32_t volatile AHB3RSTR;
	uint32_t Reserved1;
	uint32_t volatile APB1RSTR;
	uint32_t volatile APB2RSTR;
	uint32_t Reserved2;
	uint32_t Reserved3;
	uint32_t volatile AHB1ENR;
	uint32_t volatile AHB2ENR;
	uint32_t volatile AHB3ENR;
	uint32_t Reserved4;
	uint32_t volatile APB1ENR;
	uint32_t volatile APB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	uint32_t volatile AHB1LPENR;
	uint32_t volatile AHB2LPENR;
	uint32_t volatile AHB3LPENR;
	uint32_t Reserved7;
	uint32_t volatile APB1LPENR;
	uint32_t volatile APB2LPENR;
	uint32_t Reserved8;
	uint32_t Reserved9;
	uint32_t volatile BDCR;
	uint32_t volatile CSR;
	uint32_t Reserved10;
	uint32_t Reserved11;
	uint32_t volatile SSCGR;
	uint32_t volatile PLLI2SCFGR;
}RCC_RegDef_t;

/*Type casted definition for RCC*/

#define RCC 	((RCC_RegDef_t*)RCC_BASE_ADDR)

/*Structure peripheral definition for USART*/

typedef struct {
	uint32_t volatile SR;
	uint32_t volatile DR;
	uint32_t volatile BRR;
	uint32_t volatile CR1;
	uint32_t volatile CR2;
	uint32_t volatile CR3;
	uint32_t volatile GTPR;
}USART_RegDef_t;

/*Type casted definition for all USARTs*/

#define USART1	((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2	((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3	((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4	((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5	((USART_RegDef_t*)UART5_BASE_ADDR)
#define USART6	((USART_RegDef_t*)USART6_BASE_ADDR)

/*Structure peripheral definition for SPI*/

typedef struct {
	uint32_t volatile CR1;
	uint32_t volatile CR2;
	uint32_t volatile SR;
	uint32_t volatile DR;
	uint32_t volatile CRCPR;
	uint32_t volatile RXCRCR;
	uint32_t volatile TXCRCR;
	uint32_t volatile I2SCFGR;
	uint32_t volatile I2SPR;
}SPI_RegDef_t;

/*Type casted definition for all SPIs*/

#define SPI1	((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASE_ADDR)
#define SPI5	((SPI_RegDef_t*)SPI5_BASE_ADDR)
#define SPI6	((SPI_RegDef_t*)SPI6_BASE_ADDR)

/*Structure peripheral definition for I2C*/

typedef struct {
	uint32_t volatile CR1;
	uint32_t volatile CR2;
	uint32_t volatile OAR1;
	uint32_t volatile OAR2;
	uint32_t volatile DR;
	uint32_t volatile SR1;
	uint32_t volatile SR2;
	uint32_t volatile CCR;
	uint32_t volatile TRISE;
	uint32_t volatile FLTR;
}I2C_RegDef_t;

/*Type casted definition for all I2Cs*/
#define I2C1	((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2	((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3	((I2C_RegDef_t*)I2C3_BASE_ADDR)

/*Structure peripheral definition for EXTI*/

typedef struct {
	uint32_t volatile IMR;
	uint32_t volatile EMR;
	uint32_t volatile RTSR;
	uint32_t volatile FTSR;
	uint32_t volatile SWIER;
	uint32_t volatile PR;
}EXTI_RegDef_t;

/*Type casted definition for EXTI*/

#define EXTI 	((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/*Structure peripheral definition for SYSCFG*/

typedef struct {
	uint32_t volatile MEMRMP;
	uint32_t volatile PMC;
	uint32_t volatile EXTICR[4];
	uint32_t Reserved0;
	uint32_t Reserved1;
	uint32_t volatile CMPCR;
}SYSCFG_RegDef_t;

/*Type casted definition for SYSCFG*/

#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*Clock enable macros for GPIOx peripherals*/

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/*Clock disable macros for GPIOx peripherals*/

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/*Clock enable macros for I2Cx peripherals*/

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*Clock disable macros for I2Cx peripherals*/

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*Clock enable macros for SPIx peripherals*/

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1<<21))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))

/*Clock disable macros for SPIx peripherals*/

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1<<20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<21))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))

/*Clock enable macros for USART/UARTx peripherals*/

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))

/*Clock disable macros for USART/UARTx peripherals*/

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

/*Clock enable and disable macros for SYSCFG peripheral*/
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

/*IRQ numbers associated to the EXTI lines*/
#define IRQ_EXTI0				6
#define IRQ_EXTI1				7
#define IRQ_EXTI2				8
#define IRQ_EXTI3				9
#define IRQ_EXTI4				10
#define IRQ_EXTI9_5				23
#define IRQ_EXTI15_10			40

/*IRQ numbers associated to the SPI peripherals*/
#define IRQ_SPI1				35
#define IRQ_SPI2				36
#define IRQ_SPI3				51

/*IRQ numbers associated to the I2C peripherals*/
#define IRQ_I2C1_EV				31
#define IRQ_I2C1_ER				32
#define IRQ_I2C2_EV				33
#define IRQ_I2C2_ER				34
#define IRQ_I2C3_EV				79
#define IRQ_I2C3_ER				80

/*IRQ numbers associated to the USART/UART peripherals*/
#define IRQ_USART1				37
#define IRQ_USART2				38
#define IRQ_USART3				39
#define IRQ_UART4				52
#define IRQ_UART5				53
#define IRQ_USART6				71

/*Interrupt Priorities*/
#define INTERRUPT_PRI0			0
#define INTERRUPT_PRI1			1
#define INTERRUPT_PRI2			2
#define INTERRUPT_PRI3			3
#define INTERRUPT_PRI4			4
#define INTERRUPT_PRI5			5
#define INTERRUPT_PRI6			6
#define INTERRUPT_PRI7			7
#define INTERRUPT_PRI8			8
#define INTERRUPT_PRI9			9
#define INTERRUPT_PRI10			10
#define INTERRUPT_PRI11			11
#define INTERRUPT_PRI12			12
#define INTERRUPT_PRI13			13
#define INTERRUPT_PRI14			14
#define INTERRUPT_PRI15			15

/*Other important macros*/

#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE

/*
 * Bit definition macros for SPI peripherals
*/
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_BIDIMODE		15

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_FRE				8
#define SPI_SR_BSY				7

#define SPI_CR2_SSOE			2
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit definition macros for I2C peripherals
*/
#define I2C_CR1_PE				0
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10

#define I2C_OAR1_ADDR			1

#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FSMODE			15

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14

#define I2C_SR2_MSL				0
#define I2C_SR2_TRA				2

#define I2C_TRISE				0

/*
 * Bit definition macros for USART peripherals
*/
#define USART_SR_FE				1
#define USART_SR_NE				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_CTS			9

#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

#define USART_CR2_STOP			12

#define USART_CR3_EIE			0
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10

#define USART_BRR_DIVFRACTION	0
#define USART_BRR_DIVMANTISSA	4

#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"

#endif /* INC_STM32F4XX_H_ */
