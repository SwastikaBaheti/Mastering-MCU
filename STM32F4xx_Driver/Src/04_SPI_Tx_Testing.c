#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

void SPI_GPIOInit(){
	GPIO_Handle_t spi_gpios;
	spi_gpios.pGPIOx = GPIOB;
	spi_gpios.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	spi_gpios.GPIOPinConfig.GPIO_PinAltFuncMode = AF5;
	spi_gpios.GPIOPinConfig.GPIO_PinOPType = OUT_PUSH_PULL;
	spi_gpios.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;
	spi_gpios.GPIOPinConfig.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_PeriClkControl(GPIOB, ENABLE);

	// SPI2_SCK
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi_gpios);

	//SPI2_MOSI
	spi_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi_gpios);

	/*MISO and NSS pins are not configured as in this exercise no slave is connected*/
}

int main(void){

	char user_data[] = "Hello World!";

	/* GPIO pins for SPI2 functionality are as follows:
	 *
	 * SPI2_SCK - PB13 (Alternate functionality: 5)
	 * SPI2_MOSI - PB15 (Alternate functionality: 5)
	 * SPI2_MISO - PB14 (Alternate functionality: 5)
	 * SPI2_NSS - PB12 (Alternate functionality: 5)
	 *
	 * */

	/*Configure the GPIO pins for SPI functionality*/
	SPI_GPIOInit();


	/*SPI2 peripheral configurations*/
	SPI_Config_t SPI2_Config;
	SPI2_Config.SPI_DeviceMode = SPI_MASTER;
	SPI2_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Config.SPI_SclkSpeed = SPI_SCLK_DIVIDE_2;
	SPI2_Config.SPI_SSM = SPI_SSM_SW;
	SPI2_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Config.SPI_DFF = SPI_DFF_BYTE;

	SPI_Handle_t spi2_peripheral;
	spi2_peripheral.pSPIx = SPI2;
	spi2_peripheral.SPIConfig = SPI2_Config;

	/*Enabling the peripheral clock for SPI2*/
	SPI_PeriClkControl(spi2_peripheral.pSPIx, ENABLE);

	/*Initialize the SPI2 peripheral*/
	SPI_Init(&spi2_peripheral);

	/*Enable the SPI2 peripheral*/
	SPI_Peri_EN(spi2_peripheral.pSPIx);

	/*Transmitting the user data*/
	SPI_transmitData(spi2_peripheral.pSPIx, (uint8_t*)user_data, strlen(user_data));

	/*Disable the SPI2 peripheral*/
	while(SPI_getStatusFlag(SPI2, SPI_SR_BSY));
	SPI_Peri_DI(SPI2);

	while(1);
}
