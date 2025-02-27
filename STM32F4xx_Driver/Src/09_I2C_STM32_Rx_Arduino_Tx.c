#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_i2c_driver.h"

#define DEVICEADDR			0x61U
#define ARDUINOADDR			0x68U

#define CMD_READ_LENGTH		0x51
#define CMD_SLAVE_DATA		0X52

I2C_Handle_t i2c1_peripheral;

void delay(void){
	for(uint32_t i=0; i<250000; i++);
}

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

void I2C_GPIOInit(){

	/* GPIO pins for I2C functionality are as follows:
	 *
	 * I2C1_SCL - PB6 (Alternate functionality: 4)
	 * I2C1_SDA - PB7 (Alternate functionality: 4)
	 *
	 * */

	GPIO_Handle_t i2c_gpios;
	i2c_gpios.pGPIOx = GPIOB;
	i2c_gpios.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	i2c_gpios.GPIOPinConfig.GPIO_PinAltFuncMode = AF4;
	i2c_gpios.GPIOPinConfig.GPIO_PinOPType = OUT_OPEN_DRAIN;
	i2c_gpios.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;
	i2c_gpios.GPIOPinConfig.GPIO_PinPuPdControl = ONLY_PU;

	// Enable the peripheral clock for GPIO PORTB
	GPIO_PeriClkControl(GPIOB, ENABLE);

	// I2C_SCL
	i2c_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&i2c_gpios);

	// I2C_SDA
	i2c_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&i2c_gpios);
}

void I2C_PeriInit(){
	I2C_Config_t I2C1_Config;
	I2C1_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Config.I2C_DeviceAddress = DEVICEADDR;
	I2C1_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1_Config.I2C_SCLSpeed = I2C_SCL_STANDARD;

	i2c1_peripheral.pI2Cx = I2C1;
	i2c1_peripheral.I2C_Config = I2C1_Config;

	// Enabling the peripheral clock for I2C1
	I2C_PeriClkControl(i2c1_peripheral.pI2Cx, ENABLE);

	// Initialize the I2C1 peripheral
	I2C_Init(&i2c1_peripheral);
}

int main(void){
	/*Length of the slave data*/
	uint8_t sizeOfData = 0;

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for I2C communication*/
	I2C_GPIOInit();

	/*Configure the I2C peripheral*/
	I2C_PeriInit();

	/*Enable the I2C1 peripheral*/
	I2C_Peri_EN(I2C1);

	/*Set the ACK bit as ACK=1 only when PE=1*/
	I2C_SetACKbit(I2C1);

	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		//Command-1

		/*Transmitting the command to get the length of the slave data*/
		uint8_t command = CMD_READ_LENGTH;
		I2C_MasterTransmitData(&i2c1_peripheral, &command, sizeof(command), ARDUINOADDR, I2C_STOP_DI);

		/*Receiving the size of the data from the slave*/
		I2C_MasterReceiveData(&i2c1_peripheral, &sizeOfData, 1, ARDUINOADDR, I2C_STOP_DI);

		//Command-2

		/*Transmitting the command to get entire slave data*/
		command = CMD_SLAVE_DATA;
		I2C_MasterTransmitData(&i2c1_peripheral, &command, sizeof(command), ARDUINOADDR, I2C_STOP_DI);

		/*Receiving the data from the slave*/
		uint8_t slaveData[sizeOfData+1];
		I2C_MasterReceiveData(&i2c1_peripheral, slaveData, sizeOfData, ARDUINOADDR, I2C_STOP_EN);

		slaveData[sizeOfData+1] = '\0';

		/*Displaying the data received*/
		printf("The data received is: %s", slaveData);
	}
}
