#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

#define MAX_LEN	500

SPI_Handle_t spi2_peripheral;
char rcvBuffer[MAX_LEN];
volatile char readData;

/*This variable is used to indicate if the data is available for reading from the slave*/
volatile uint8_t readDataAvailable = 0;

/*This variable is used to indicate to stop reading the data from Slave*/
volatile uint8_t rcvStop = 0;

void IRQGPIO_Init(){

	/* Port D pin 6 will issue an interrupt on the EXTI9_5 line
	 * Edge detection block should be set to detect the falling edge
	 * */

	GPIO_Handle_t gpioIRQPin;
	memset(&gpioIRQPin, 0, sizeof(gpioIRQPin));
	gpioIRQPin.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_6;
	gpioIRQPin.GPIOPinConfig.GPIO_PinMode = IN_FE_MODE;
	gpioIRQPin.pGPIOx = GPIOD;

	// Enable the peripheral clock for GPIO PORTD
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// Configure the GPIO pin
	GPIO_Init(&gpioIRQPin);

	// IRQ Configuration
	GPIO_IRQConfig(IRQ_EXTI9_5, INTERRUPT_PRI15, ENABLE);
}

void SPI2_GPIOInit(){

	/* GPIO pins for SPI2 functionality are as follows:
	 *
	 * SPI2_NSS - PB12 (Alternate functionality: 5)
	 * SPI2_SCK - PB13 (Alternate functionality: 5)
	 * SPI2_MISO - PB14 (Alternate functionality: 5)
	 * SPI2_MOSI - PB15 (Alternate functionality: 5)
	 *
	 * */

	GPIO_Handle_t spi_gpios;
	spi_gpios.pGPIOx = GPIOB;
	spi_gpios.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	spi_gpios.GPIOPinConfig.GPIO_PinAltFuncMode = AF5;
	spi_gpios.GPIOPinConfig.GPIO_PinOPType = OUT_PUSH_PULL;
	spi_gpios.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;
	spi_gpios.GPIOPinConfig.GPIO_PinPuPdControl = NO_PUPD;

	// Enable the peripheral clock for GPIO PORTB
	GPIO_PeriClkControl(GPIOB, ENABLE);

	// SPI2_NSS
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&spi_gpios);

	// SPI2_SCK
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi_gpios);

	// SPI2_MISO
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&spi_gpios);

	// SPI2_MOSI
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi_gpios);
}

void SPI2_Init(){
	SPI_Config_t SPI2_Config;
	SPI2_Config.SPI_DeviceMode = SPI_MASTER;
	SPI2_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Config.SPI_SclkSpeed = SPI_SCLK_DIVIDE_8; /*Confirm?*/
	SPI2_Config.SPI_SSM = SPI_SSM_HW;
	SPI2_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Config.SPI_DFF = SPI_DFF_BYTE; /*Confirm?*/

	spi2_peripheral.pSPIx = SPI2;
	spi2_peripheral.SPIConfig = SPI2_Config;

	// Enabling the peripheral clock for SPI2
	SPI_PeriClkControl(spi2_peripheral.pSPIx, ENABLE);

	// Initialize the SPI2 peripheral
	SPI_Init(&spi2_peripheral);
}

void delay(void){
	for(uint32_t i=0; i<250000; i++);
}

int main(void){
	uint8_t dummyWrite = 0xFF;

	/*Configure the GPIO PD6 pin for giving interrupt on falling edge*/
	IRQGPIO_Init();

	/*Configure the GPIO pins for SPI communication*/
	SPI2_GPIOInit();

	/*Configure the SPI2 peripheral*/
	SPI2_Init();

	/*Enable the NSS Output for SPI2 peripheral*/
	SPI_SSOutput_EN(SPI2);

	/*Configure the SPI IRQ*/
	SPI_IRQConfig(IRQ_SPI2, INTERRUPT_PRI14, ENABLE);

	while(1){
		rcvStop = 0;

		//Check if there is data to be read by the master
		while(!readDataAvailable);

		//When the read data is available
		GPIO_IRQConfig(IRQ_EXTI9_5, INTERRUPT_PRI15, DISABLE);

		//Enable the SPE bit of SPI peripheral
		SPI_Peri_EN(spi2_peripheral.pSPIx);

		while(!rcvStop){
			// Initiate the SPI Tx to receive the data from the Slave
			while(SPI_transmitData_IT(&spi2_peripheral, &dummyWrite, 1) == SPI_BUSY_IN_TX);

			//Receive the data from the Slave
			while(SPI_ReceiveData_IT(&spi2_peripheral, (uint8_t *)&readData, 1) == SPI_BUSY_IN_RX);
		}

		while(SPI_getStatusFlag(spi2_peripheral.pSPIx, SPI_SR_BSY));

		//Disable the SPE bit of SPI peripheral
		SPI_Peri_DI(spi2_peripheral.pSPIx);

		//Data received in the rcvBuffer
		printf("The data received is %s", rcvBuffer);

		readDataAvailable = 0;
		GPIO_IRQConfig(IRQ_EXTI9_5, INTERRUPT_PRI15, ENABLE);
	}
}

void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_5);
	readDataAvailable = 1;
}

void SPI2_IRQHandler(void){
	SPI_IRQHandling(&spi2_peripheral);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event){
	static uint32_t i=0;

	// In case of the Rx Complete event, store the 1 byte of data received in the RcvBuffer
	if(event == SPI_EVENT_RX_CMPLT){
		rcvBuffer[i++] = readData;
		if((i == MAX_LEN-1) || (readData != '\0')){
			rcvBuffer[i] = '\0';
			rcvStop = 1;
			i=0;
		}
	}
}
