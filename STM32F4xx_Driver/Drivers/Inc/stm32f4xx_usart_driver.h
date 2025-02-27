#ifndef INC_STM32F4XX_USART_DRIVER_H_
#define INC_STM32F4XX_USART_DRIVER_H_

#include "stm32f4xx.h"

/*Configuration structure for a USART peripheral*/

typedef struct{
	uint8_t USART_Mode;				//From @USART_MODES
	uint32_t USART_Baud;				//From @USART_BAUD
	uint8_t USART_NoofStopBits;		//From @USART_STOP_BITS
	uint8_t USART_WordLength;		//From @USART_WORD_LENGTH
	uint8_t USART_ParityControl;	//From @USART_PARITY_CONTROL
	uint8_t USART_HWFlowControl;	//From @USART_HWFLOW_CONTROL
}USART_Config_t;

/*Handle structure for a USART peripheral*/

typedef struct{
	USART_RegDef_t* pUSARTx;		//Base address of USARTx (USART1-6)
	USART_Config_t USARTConfig;		//Different configurations of USART
	uint8_t *pTxBuffer;				//To store the Tx buffer address
	uint8_t *pRxBuffer;				//To store the Rx buffer address
	uint32_t TxLen;					//To store the Tx data length
	uint32_t RxLen;					//To store the Rx data length
	uint8_t TxState;				//To store the Tx state
	uint8_t RxState;				//To store the Rx state
}USART_Handle_t;

/*
 * Prototype for the APIs supported by the SPI peripheral
*/

/*USART Peripheral clock setup*/
void USART_PeriClkControl(USART_RegDef_t* pUSARTx, uint8_t clkValue);

/*Initialize or De-initialize USART*/
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t* pUSARTx);

/*Data transmit and receive*/
void USART_TransmitData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t length);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length);
uint8_t USART_TransmitDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t length);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length);

/*IRQ configuration and IRQ handling*/
void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*Other USART peripheral control APIs*/
void USART_Peri_EN(USART_RegDef_t* pUSARTx);
void USART_Peri_DI(USART_RegDef_t* pUSARTx);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
void USART_ClearOREFlag(USART_RegDef_t *pUSARTx);

/*Application callback function*/
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event);

/*USART Status Flags*/
#define USART_RXNE_FLAG		(1 << USART_SR_RXNE)
#define USART_TXE_FLAG		(1 << USART_SR_TXE)
#define USART_TC_FLAG		(1 << USART_SR_TC)

/*USART peripheral states*/
#define USART_READY				0
#define USART_BUSY_IN_RX		1
#define USART_BUSY_IN_TX		2

/*USART application events*/
#define USART_EVENT_TX_CMPLT	1
#define USART_EVENT_RX_CMPLT	2
#define USART_EVENT_CTS			3
#define USART_EVENT_IDLE		4
#define USART_EVENT_ORE			5
#define USART_ERREVENT_FE		6
#define USART_ERREVENT_NE		7
#define USART_ERREVENT_ORE		8

/*
 * User friendly macros
*/

/*@USART_MODES
 * USART modes*/
#define USART_ONLY_Tx			0
#define USART_ONLY_Rx			1
#define USART_Tx_Rx				2

/*
 *@USART_BAUD
 *Possible options for USART Baud rate
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*@USART_STOP_BITS
 * USART stop bits*/
#define USART_STOPBIT_1			0
#define USART_STOPBIT_0_5		1
#define USART_STOPBIT_2			2
#define USART_STOPBIT_1_5		3

/*@USART_WORD_LENGTH
 * USART word length*/
#define USART_WORDLEN_8			0
#define USART_WORDLEN_9			1

/*@USART_PARITY_CONTROL
 * USART parity control*/
#define USART_PARITY_DI			0
#define USART_PARITY_EVEN		1
#define USART_PARITY_ODD		2

/*@USART_HWFLOW_CONTROL
 * USART hardware flow control*/
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

#endif /* INC_STM32F4XX_USART_DRIVER_H_ */
