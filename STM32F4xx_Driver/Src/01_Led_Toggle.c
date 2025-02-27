#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i<10000; i++);
}

int main(void)
{
    /*
     * Exercise: Write a program to toggle the on board LED (Port D Pin 12) with some delay
     * Case 1: Use push pull configuration for the output pin
     * Case 2: Use open drain configuration for the output pin
     *
     * */

	//Case-1
	GPIO_PinConfig_t GPIOPinConfig;
	GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOPinConfig.GPIO_PinMode = OUTPUT_MODE;
	GPIOPinConfig.GPIO_PinOPType = OUT_PUSH_PULL;
	GPIOPinConfig.GPIO_PinSpeed = MED_SPEED;
	GPIOPinConfig.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_Handle_t gpioLed;
	gpioLed.GPIOPinConfig = GPIOPinConfig;
	gpioLed.pGPIOx = GPIOD;

	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}

	//Case-2
	GPIOPinConfig.GPIO_PinOPType = OUT_OPEN_DRAIN;
	GPIOPinConfig.GPIO_PinPuPdControl = ONLY_PU;

	gpioLed.GPIOPinConfig = GPIOPinConfig;

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}

	/* Loop forever */
	for(;;);
}
