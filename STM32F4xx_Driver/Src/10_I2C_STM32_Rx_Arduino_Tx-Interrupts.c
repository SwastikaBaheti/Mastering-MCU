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
volatile uint8_t enableComm = 0;
volatile uint8_t rxComplete = 0;

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

	// IRQ Configuration
	GPIO_IRQConfig(IRQ_EXTI0, INTERRUPT_PRI15, ENABLE);
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
	uint8_t slaveData[32];

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for I2C communication*/
	I2C_GPIOInit();

	/*Configure the I2C peripheral*/
	I2C_PeriInit();

	/*Configure the I2C IRQ*/
	I2C_IRQConfig(IRQ_I2C1_EV, INTERRUPT_PRI13, ENABLE);
	I2C_IRQConfig(IRQ_I2C1_ER, INTERRUPT_PRI14, ENABLE);

	/*Enable the I2C1 peripheral*/
	I2C_Peri_EN(I2C1);

	/*Set the ACK bit as ACK=1 only when PE=1*/
	I2C_SetACKbit(I2C1);

	while(1){
		//Check whether the button has been pressed
		while(!enableComm);

		//When the communication device is available
		GPIO_IRQConfig(IRQ_EXTI0, INTERRUPT_PRI15, DISABLE);

		//Command-1

		/*Transmitting the command to get the length of the slave data*/
		uint8_t command = CMD_READ_LENGTH;
		while(!(I2C_MasterTransmitDataIT(&i2c1_peripheral, &command, sizeof(command), ARDUINOADDR, I2C_STOP_DI) != I2C_READY));

		/*Receiving the size of the data from the slave*/
		while(!(I2C_MasterReceiveDataIT(&i2c1_peripheral, &sizeOfData, 1, ARDUINOADDR, I2C_STOP_DI) != I2C_READY));

		//Command-2

		/*Transmitting the command to get entire slave data*/
		command = CMD_SLAVE_DATA;
		while(!(I2C_MasterTransmitDataIT(&i2c1_peripheral, &command, sizeof(command), ARDUINOADDR, I2C_STOP_DI) != I2C_READY));

		/*Receiving the data from the slave*/
		rxComplete = 0;
		while(!(I2C_MasterReceiveDataIT(&i2c1_peripheral, slaveData, sizeOfData, ARDUINOADDR, I2C_STOP_EN) != I2C_READY));

		while(!rxComplete);
		slaveData[sizeOfData+1] = '\0';

		/*Displaying the data received*/
		printf("The data received is: %s\n", slaveData);

		rxComplete = 0;
		enableComm = 0;
		GPIO_IRQConfig(IRQ_EXTI0, INTERRUPT_PRI15, ENABLE);
	}
}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_0);
	enableComm = 1;
}

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&i2c1_peripheral);
}


void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&i2c1_peripheral);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event){
	if(event == I2C_EV_TX_CMPLT){
		printf("The data transmission is completed\n");
	} else if(event == I2C_EV_RX_CMPLT){
		printf("The data is successfully received\n");
		rxComplete = 1;
	} else if(event == I2C_ERROR_BERR){
		printf("Bus error occurred\n");
	} else if(event == I2C_ERROR_ARLO){
		printf("Arbitration error occurred\n");
	} else if(event == I2C_ERROR_AF){
		printf("Acknowledgment error occurred\n");

		//Close the transmission since there is no ACK received
		I2C_CloseTransmission(&i2c1_peripheral);

		//Generate the STOP condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		//Hang the application so as to avoid the further execution of program
		while(1);
	} else if(event == I2C_ERROR_OVR){
		printf("Overrun/underrun error occurred\n");
	} else if(event == I2C_ERROR_TIMEOUT){
		printf("Timeout error occurred\n");
	}
}
