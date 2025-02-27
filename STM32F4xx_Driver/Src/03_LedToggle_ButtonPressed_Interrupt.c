#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i<250000; i++);
}

int main(void)
{
    /*
     * Exercise: Connect an external button to PD5 pin and toggle the LED whenever interrupt is triggered by the
     * button press
     *
     * Interrupt should be triggered during falling edge of button press
     * */

	//LED
	GPIO_PinConfig_t GPIOPinConfigLed;
	GPIOPinConfigLed.GPIO_PinNumber = GPIO_PIN_12;
	GPIOPinConfigLed.GPIO_PinMode = OUTPUT_MODE;
	GPIOPinConfigLed.GPIO_PinOPType = OUT_PUSH_PULL;
	GPIOPinConfigLed.GPIO_PinSpeed = MED_SPEED;
	GPIOPinConfigLed.GPIO_PinPuPdControl = NO_PUPD;

	GPIO_Handle_t gpioLed;
	memset(&gpioLed, 0, sizeof(gpioLed));
	gpioLed.GPIOPinConfig = GPIOPinConfigLed;
	gpioLed.pGPIOx = GPIOD;

	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	//Button
	GPIO_PinConfig_t GPIOPinConfigButton;
	GPIOPinConfigButton.GPIO_PinNumber = GPIO_PIN_5;
	GPIOPinConfigButton.GPIO_PinMode = IN_FE_MODE;
	GPIOPinConfigButton.GPIO_PinPuPdControl = ONLY_PU;

	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(gpioButton));
	gpioButton.GPIOPinConfig = GPIOPinConfigButton;
	gpioButton.pGPIOx = GPIOD;

	GPIO_Init(&gpioButton);
	GPIO_IRQConfig(IRQ_EXTI9_5,INTERRUPT_PRI15, ENABLE);

	/* Loop forever */
	for(;;);
}

void EXTI9_5_IRQHandler(void){
	delay(); //Button de-bouncing, wait 200ms
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
