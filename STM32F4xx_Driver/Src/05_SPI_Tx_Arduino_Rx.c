#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

void GPIO_ButtonConfig(){
	GPIO_PinConfig_t GPIOPinConfigButton;
	GPIOPinConfigButton.GPIO_PinNumber = GPIO_PIN_0;
	GPIOPinConfigButton.GPIO_PinMode = INPUT_MODE;
	GPIOPinConfigButton.GPIO_PinSpeed = HIGH_SPEED;
	GPIOPinConfigButton.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_Handle_t gpioButton;
	gpioButton.GPIOPinConfig = GPIOPinConfigButton;
	gpioButton.pGPIOx = GPIOA;

	// Enabling the peripheral clock for GPIOA
	GPIO_PeriClkControl(GPIOA, ENABLE);

	// Initialize the GPIO pin
	GPIO_Init(&gpioButton);
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

	//SPI2_MOSI
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi_gpios);

	/*MISO is not configured as the MASTER is TX only*/
}

void SPI2_Init(){
	SPI_Config_t SPI2_Config;
	SPI2_Config.SPI_DeviceMode = SPI_MASTER;
	SPI2_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Config.SPI_SclkSpeed = SPI_SCLK_DIVIDE_8;
	SPI2_Config.SPI_SSM = SPI_SSM_HW;
	SPI2_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Config.SPI_DFF = SPI_DFF_BYTE;

	SPI_Handle_t spi2_peripheral;
	spi2_peripheral.pSPIx = SPI2;
	spi2_peripheral.SPIConfig = SPI2_Config;

	// Enabling the peripheral clock for SPI2
	SPI_PeriClkControl(spi2_peripheral.pSPIx, ENABLE);

	// Initialize the SPI2 peripheral
	SPI_Init(&spi2_peripheral);
}

int main(void){

	char user_data[] = "You just pressed a button";
	uint8_t sizeOfUserData = strlen(user_data);

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for SPI communication*/
	SPI2_GPIOInit();

	/*Configure the SPI2 peripheral*/
	SPI2_Init();

	/*Enable the NSS Output for SPI2 peripheral*/
	SPI_SSOutput_EN(SPI2);

	while(1){
		uint8_t inputValue = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0);
		if(inputValue){
			/*Handle button de-bouncing*/
			while(inputValue);

			/*Enable the SPI2 peripheral*/
			SPI_Peri_EN(SPI2);

			/*Transmitting the size of user data*/
			SPI_transmitData(SPI2, &sizeOfUserData, strlen(user_data));

			/*Transmitting the user data*/
			SPI_transmitData(SPI2, (uint8_t*)user_data, strlen(user_data));

			/*Disable the SPI2 peripheral*/
			while(SPI_getStatusFlag(SPI2, SPI_SR_BSY));
			SPI_Peri_DI(SPI2);
		}
	}
}
