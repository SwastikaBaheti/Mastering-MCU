#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_i2c_driver.h"

#define DEVICEADDR		0x61U
#define ARDUINOADDR		0x68U

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

	/*User data that has to be transmitted from STM32 to Arduino UNO*/
	char user_data[] = "Hello World!";

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for I2C communication*/
	I2C_GPIOInit();

	/*Configure the I2C peripheral*/
	I2C_PeriInit();

	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		/*Enable the I2C1 peripheral*/
		I2C_Peri_EN(I2C1);

		/*Transmitting the user data*/
		I2C_MasterTransmitData(&i2c1_peripheral, (uint8_t*)user_data, strlen(user_data), ARDUINOADDR, I2C_STOP_EN);

		/*Disable the I2C1 peripheral*/
		I2C_Peri_DI(I2C1);
	}
}
