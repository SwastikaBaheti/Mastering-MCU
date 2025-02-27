#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i<1000; i++);
}

int main(void)
{
    /*
     * Exercise: Write a program to toggle the on board LED whenever the on board button is pressed
     * */

	//LED
	GPIO_PinConfig_t GPIOPinConfigLed;
	GPIOPinConfigLed.GPIO_PinNumber = GPIO_PIN_12;
	GPIOPinConfigLed.GPIO_PinMode = OUTPUT_MODE;
	GPIOPinConfigLed.GPIO_PinOPType = OUT_PUSH_PULL;
	GPIOPinConfigLed.GPIO_PinSpeed = MED_SPEED;
	GPIOPinConfigLed.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_Handle_t gpioLed;
	gpioLed.GPIOPinConfig = GPIOPinConfigLed;
	gpioLed.pGPIOx = GPIOD;

	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	//Button
	GPIO_PinConfig_t GPIOPinConfigButton;
	GPIOPinConfigButton.GPIO_PinNumber = GPIO_PIN_0;
	GPIOPinConfigButton.GPIO_PinMode = INPUT_MODE;
	GPIOPinConfigButton.GPIO_PinSpeed = MED_SPEED;
	GPIOPinConfigButton.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_Handle_t gpioButton;
	gpioButton.GPIOPinConfig = GPIOPinConfigButton;
	gpioButton.pGPIOx = GPIOA;

	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	while(1){
		uint8_t inputValue = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0);
		if(inputValue){
			while(inputValue);	//Handle button de-bouncing
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		}
	}

	/* Loop forever */
	for(;;);
}
