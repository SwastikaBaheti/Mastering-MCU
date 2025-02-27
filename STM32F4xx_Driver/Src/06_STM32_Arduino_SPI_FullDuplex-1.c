#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"

/*Command codes macros*/
#define CMD_LED_CTRL			0x50
#define CMD_SENSOR_READ			0x51
#define CMD_LED_READ			0x52
#define CMD_PRINT				0x53
#define CMD_ID_READ				0x54

/*ACK macros*/
#define ACK_BYTE				0xF5

/*LED Control*/
#define LED_ON					1
#define LED_OFF					0

/*Arduino Analog pins*/
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4
#define ANALOG_PIN5				5

/*Arduino Digital pins*/
#define DIGITAL_PIN0			0
#define DIGITAL_PIN1			1
#define DIGITAL_PIN2			2
#define DIGITAL_PIN3			3
#define DIGITAL_PIN4			4
#define DIGITAL_PIN5			5
#define DIGITAL_PIN6			6
#define DIGITAL_PIN7			7
#define DIGITAL_PIN8			8
#define DIGITAL_PIN9			9
#define DIGITAL_PIN10			10
#define DIGITAL_PIN11			11
#define DIGITAL_PIN12			12
#define DIGITAL_PIN13			13

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

void delay(void){
	for(uint32_t i=0; i<250000; i++);
}

uint8_t SPI_VerifyResponse(uint8_t response){
	if(response == ACK_BYTE){
		return 1;
	}
	return 0;
}

int main(void){

	/*Dummy data*/
	uint8_t dummyWrite = 0xFF;
	uint8_t dummyRead;

	char userData[] = "Hello World!";

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for SPI communication*/
	SPI2_GPIOInit();

	/*Configure the SPI2 peripheral*/
	SPI2_Init();

	/*Enable the NSS Output for SPI2 peripheral*/
	SPI_SSOutput_EN(SPI2);

	while(1){
		/*Wait for first button press*/
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/*Enable the SPI2 peripheral*/
		SPI_Peri_EN(SPI2);

		/**
		 * COMMAND 1
		 */

		/*Transmit the CMD_LED_CTRL command (1 byte)*/
		uint8_t commandCode = CMD_LED_CTRL;
		uint8_t ackData;
		SPI_transmitData(SPI2, &commandCode, 1);

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the ACK/NACK byte from the Slave*/
		SPI_ReceiveData(SPI2, &ackData, 1);

		/*To check if the byte received is ACK or NACK*/
		if(SPI_VerifyResponse(ackData)){
			// Send arguments
			uint8_t args[2] = {DIGITAL_PIN9, LED_ON};
			SPI_transmitData(SPI2, args, 2);
		} else {
			// Display error message
			printf("The command entered is not recognized by the slave");
		}

		/*Wait for first button press*/
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/**
		 * COMMAND 2
		 */

		/*Transmit the CMD_SENSOR_READ command (1 byte)*/
		commandCode = CMD_SENSOR_READ;
		SPI_transmitData(SPI2, &commandCode, 1);

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the ACK/NACK byte from the Slave*/
		SPI_ReceiveData(SPI2, &ackData, 1);

		/*To check if the byte received is ACK or NACK*/
		if(SPI_VerifyResponse(ackData)){
			// Send arguments
			uint8_t analogPin = ANALOG_PIN0;
			SPI_transmitData(SPI2, &analogPin, 1);
		} else {
			// Display error message
			printf("The command entered is not recognized by the slave");
		}

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Give delay to let the Slave collect the sensor data*/
		delay();

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the sensor data (1 byte) from the Slave*/
		uint8_t sensorData;
		SPI_ReceiveData(SPI2, &sensorData, 1);
		printf("The sensor data is %d",sensorData);

		/*Wait for first button press*/
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/**
		 * COMMAND 3
		 */

		/*Transmit the CMD_LED_READ command (1 byte)*/
		commandCode = CMD_LED_READ;
		SPI_transmitData(SPI2, &commandCode, 1);

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the ACK/NACK byte from the Slave*/
		SPI_ReceiveData(SPI2, &ackData, 1);

		/*To check if the byte received is ACK or NACK*/
		if(SPI_VerifyResponse(ackData)){
			// Send arguments
			uint8_t ledPin = DIGITAL_PIN9;
			SPI_transmitData(SPI2, &ledPin, 1);
		} else {
			// Display error message
			printf("The command entered is not recognized by the slave");
		}

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Give delay to let the Slave collect the led status*/
		delay();

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the sensor data (1 byte) from the Slave*/
		uint8_t ledStatus;
		SPI_ReceiveData(SPI2, &ledStatus, 1);
		printf("The led status is %d",ledStatus);

		/*Wait for first button press*/
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/**
		 * COMMAND 4
		 */

		/*Transmit the CMD_PRINT command (1 byte)*/
		commandCode = CMD_PRINT;
		SPI_transmitData(SPI2, &commandCode, 1);

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the ACK/NACK byte from the Slave*/
		SPI_ReceiveData(SPI2, &ackData, 1);

		/*To check if the byte received is ACK or NACK*/
		if(SPI_VerifyResponse(ackData)){
			// Send arguments
			uint8_t args[2];
			args[0] = strlen(userData);
			args[1] = *((uint8_t*)userData);
			SPI_transmitData(SPI2, args, 2);
		} else {
			// Display error message
			printf("The command entered is not recognized by the slave");
		}

		/*Wait for first button press*/
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/**
		 * COMMAND 5
		 */

		/*Transmit the CMD_ID_READ command (1 byte)*/
		commandCode = CMD_ID_READ;
		SPI_transmitData(SPI2, &commandCode, 1);

		/*Dummy read to clear off the RXNE bit*/
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		/*Send dummy data (1 byte) to fetch response from the Slave*/
		SPI_transmitData(SPI2, &dummyWrite, 1);

		/*Receive the ACK/NACK byte from the Slave*/
		SPI_ReceiveData(SPI2, &ackData, 1);

		uint8_t boardId[10];

		/*To check if the byte received is ACK or NACK*/
		if(SPI_VerifyResponse(ackData)){
			uint32_t i=0;
			for(i=0; i<10; i++){
				/*Send dummy data (1 byte) to fetch response from the Slave*/
				SPI_transmitData(SPI2, &dummyWrite, 1);

				/*Receive the 10 byte board ID from the Slave*/
				SPI_ReceiveData(SPI2, &boardId[i], 1);
			}
			boardId[10] = '\0';
		} else {
			// Display error message
			printf("The command entered is not recognized by the slave");
		}

		printf("The board is %s", boardId);

		/*Disable the SPI2 peripheral*/
		while(SPI_getStatusFlag(SPI2, SPI_SR_BSY));
		SPI_Peri_DI(SPI2);
	}
}
