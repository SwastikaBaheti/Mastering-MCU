#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_usart_driver.h"

USART_Handle_t usart1_peripheral;
volatile uint8_t receivedData = 0;

char *message[3] = {"Message1", "Message2", "Message3"};
char rcvBuffer[1024];

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

void USART_GPIOInit(){

	/* GPIO pins for USART functionality are as follows:
	 *
	 * USART1_TX - PA9	  (Alternate functionality: 7)
	 * USART1_RX - PA10   (Alternate functionality: 7)
	 * USART1_CTS - PA11  (Alternate functionality: 7)
	 * USART1_RTS - PA12  (Alternate functionality: 7)
	 *
	 * */

	GPIO_Handle_t usart_gpios;
	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	usart_gpios.GPIOPinConfig.GPIO_PinAltFuncMode = AF7;
	usart_gpios.GPIOPinConfig.GPIO_PinOPType = OUT_PUSH_PULL;
	usart_gpios.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;
	usart_gpios.GPIOPinConfig.GPIO_PinPuPdControl = ONLY_PU;

	// Enable the peripheral clock for GPIO PORTA
	GPIO_PeriClkControl(GPIOA, ENABLE);

	// USART_TX
	usart_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&usart_gpios);

	// USART_RX
	usart_gpios.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&usart_gpios);

	/*RX and RTS pin not configured as only transmission needs to be performed*/
}

void USART_PeriInit(){
	USART_Config_t USART1_Config;
	USART1_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1_Config.USART_Mode = USART_ONLY_Tx;
	USART1_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1_Config.USART_NoofStopBits = USART_STOPBIT_1;
	USART1_Config.USART_ParityControl = USART_PARITY_DI;
	USART1_Config.USART_WordLength = USART_WORDLEN_8;

	usart1_peripheral.pUSARTx = USART1;
	usart1_peripheral.USARTConfig = USART1_Config;

	// Enabling the peripheral clock for USART1
	USART_PeriClkControl(usart1_peripheral.pUSARTx, ENABLE);

	// Initialize the USART1 peripheral
	USART_Init(&usart1_peripheral);
}

int main(void){
	uint32_t idx=0;

	/*Configure the button on STM32 MCU*/
	GPIO_ButtonConfig();

	/*Configure the GPIO pins for USART communication*/
	USART_GPIOInit();

	/*Configure the USART peripheral*/
	USART_PeriInit();

	/*IRQ configurations for the USART peripheral*/
	USART_IRQConfig(IRQ_USART1, 14, ENABLE);

	/*Enable the USART1 peripheral*/
	USART_Peri_EN(usart1_peripheral.pUSARTx);

	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/*Handle button de-bouncing*/
		delay();

		idx = idx%3;

		//Enable the interrupt for RXE
		while(USART_ReceiveDataIT(&usart1_peripheral, (uint8_t*)rcvBuffer, strlen(message[idx])) != USART_READY);

		//Transmit the data
		USART_TransmitData(&usart1_peripheral, (uint8_t*)&message[idx], strlen(message[idx]));

		printf("Transmitted message: %s\n", message[idx]);

		//Now check if the data has been received by the Arduino
		while(!receivedData);
		rcvBuffer[strlen(message[idx])+1] = '\0';

		printf("Received message: %s\n", rcvBuffer);

		idx++;
		receivedData = 0;
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart1_peripheral);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event){
	if(event == USART_EVENT_RX_CMPLT){
		//Data has been received
		receivedData = 1;
	}
}
