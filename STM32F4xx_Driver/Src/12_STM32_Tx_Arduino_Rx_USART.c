#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_usart_driver.h"

USART_Handle_t usart1_peripheral;

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

	uint8_t user_data[] = "USART Tx from STM32 board";

	/*Configure the GPIO pins for USART communication*/
	USART_GPIOInit();

	/*Configure the USART peripheral*/
	USART_PeriInit();

	/*Enable the USART1 peripheral*/
	USART_Peri_EN(usart1_peripheral.pUSARTx);

	/*Transmit data to Arduino*/
	USART_TransmitData(&usart1_peripheral, user_data, strlen((char*)user_data));

	while(1);
}
