#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_i2c_driver.h"

#define SLAVEADDR			0x68U

#define CMD_SEND_LENGTH		0x51
#define CMD_SEND_DATA		0X52

I2C_Handle_t i2c1_peripheral;
uint8_t txBuffer[32] = "STM32 Slave mode testing";

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
	I2C1_Config.I2C_DeviceAddress = SLAVEADDR;
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

	/*Configure the GPIO pins for I2C communication*/
	I2C_GPIOInit();

	/*Configure the I2C peripheral*/
	I2C_PeriInit();

	/*Configure the I2C IRQ*/
	I2C_IRQConfig(IRQ_I2C1_EV, INTERRUPT_PRI13, ENABLE);
	I2C_IRQConfig(IRQ_I2C1_ER, INTERRUPT_PRI14, ENABLE);

	/*Enable the IRQ control bits of control register in I2C peripheral*/
	I2C_SetIRQControlBits(I2C1);

	/*Enable the I2C1 peripheral*/
	I2C_Peri_EN(I2C1);

	/*Set the ACK bit as ACK=1 only when PE=1*/
	I2C_SetACKbit(I2C1);

	while(1);
}

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&i2c1_peripheral);
}


void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&i2c1_peripheral);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event){
	static uint8_t idx = 0;
	static uint8_t commandCode = 0;

	if(event == I2C_EV_DATA_REQ){
		//Slave transmits the data (1 byte) to the master
		if(commandCode == CMD_SEND_LENGTH){
			//Send length information to the master
			I2C_SlaveTransmitData(pI2CHandle->pI2Cx, strlen((char*)txBuffer));
		} else if(commandCode == CMD_SEND_DATA){
			//Send the data to the master
			I2C_SlaveTransmitData(pI2CHandle->pI2Cx, txBuffer[idx++]);
		}
	} else if(event == I2C_EV_DATA_RCV){
		//Slave receives the data (1 byte) from the master
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if(event == I2C_ERROR_AF){
		//This happens only during Slave transmission
		//Master doesn't want to receive more data as master has send NACK
		commandCode = 0xFF;
		idx = 0;
	} else if(event == I2C_EV_STOP){
		//This happens only during slave reception
		//Master stops the communication
	}
}
