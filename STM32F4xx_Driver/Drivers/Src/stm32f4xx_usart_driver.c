#include "stm32f4xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"

/*Helper (private) function*/
static void USART_TC_IRQHandler(USART_Handle_t *pUSARTHandle);
static void USART_TXE_IRQHandler(USART_Handle_t *pUSARTHandle);
static void USART_RXNE_IRQHandler(USART_Handle_t *pUSARTHandle);

/*
 * The function enables or disables the peripheral clock for a given USART peripheral
 * @Input USART peripheral base address
 * @Input Clock value (Enable: 1 or Disable: 0)
 * @Output void
 * */
void USART_PeriClkControl(USART_RegDef_t* pUSARTx, uint8_t clkValue){
	if(clkValue == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		} else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		} else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		} else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	} else {
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		} else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		} else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		} else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		} else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		} else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

/*
 * This function initializes the appropriate peripheral registers to the USART peripheral provided
 * @Input Handle structure of the USART
 * @Output void
 * */
void USART_Init(USART_Handle_t *pUSARTHandle){

	//USART Mode
	uint8_t mode = pUSARTHandle->USARTConfig.USART_Mode;
	if(mode == USART_ONLY_Tx){
		//Tx only
		pUSARTHandle->pUSARTx->CR1 &= ~(0xC << USART_CR1_RE);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TE);
	} else if(mode == USART_ONLY_Rx){
		//Rx only
		pUSARTHandle->pUSARTx->CR1 &= ~(0xC << USART_CR1_RE);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RE);
	} else if(mode == USART_Tx_Rx){
		//Both Tx and Rx
		pUSARTHandle->pUSARTx->CR1 |= (0xC << USART_CR1_RE);
	}

	//USART Word length
	uint8_t wordLength = pUSARTHandle->USARTConfig.USART_WordLength;
	pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_M);
	pUSARTHandle->pUSARTx->CR1 |= (wordLength << USART_CR1_M);

	//USART Parity Control
	uint8_t parityCtrl = pUSARTHandle->USARTConfig.USART_ParityControl;
	if(parityCtrl == USART_PARITY_DI){
		pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_PCE);
	} else if(parityCtrl == USART_PARITY_EVEN){
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_PS);
	} else if(parityCtrl == USART_PARITY_ODD){
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_PS);
	}

	//USART Stop bits
	uint8_t stopBits = pUSARTHandle->USARTConfig.USART_NoofStopBits;
	pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << USART_CR2_STOP);
	pUSARTHandle->pUSARTx->CR2 |= (stopBits << USART_CR2_STOP);

	//USART Hardware Flow Control
	uint8_t hwFlowCtrl = pUSARTHandle->USARTConfig.USART_HWFlowControl;
	if(hwFlowCtrl == USART_HW_FLOW_CTRL_NONE){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR2 &= ~(0x1 << USART_CR3_CTSE);
	} else if(hwFlowCtrl == USART_HW_FLOW_CTRL_CTS){
		pUSARTHandle->pUSARTx->CR2 &= ~(0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR2 |= (0x1 << USART_CR3_CTSE);
	} else if(hwFlowCtrl == USART_HW_FLOW_CTRL_RTS){
		pUSARTHandle->pUSARTx->CR2 |= (0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR2 &= ~(0x1 << USART_CR3_CTSE);
	} else if(hwFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS){
		pUSARTHandle->pUSARTx->CR2 |= (0x1 << USART_CR3_RTSE);
		pUSARTHandle->pUSARTx->CR2 |= (0x1 << USART_CR3_CTSE);
	}

	//USART Baud rate
	uint32_t baudRate = pUSARTHandle->USARTConfig.USART_Baud;
	USART_SetBaudRate(pUSARTHandle->pUSARTx, baudRate);
}

/*
 * This function resets the appropriate peripheral registers to their corresponding reset values
 * @Input Base address of USART peripheral
 * @Output void
 * */
void USART_DeInit(USART_RegDef_t* pUSARTx){
	if(pUSARTx == USART1){
		RCC->APB2RSTR |= (0x1 << 4);
		RCC->APB2RSTR &= ~(0x1 << 4);
	} else if(pUSARTx == USART2){
		RCC->APB1RSTR |= (0x1 << 17);
		RCC->APB1RSTR &= ~(0x1 << 17);
	} else if(pUSARTx == USART3){
		RCC->APB1RSTR |= (0x1 << 18);
		RCC->APB1RSTR &= ~(0x1 << 18);
	} else if(pUSARTx == UART4){
		RCC->APB1RSTR |= (0x1 << 19);
		RCC->APB1RSTR &= ~(0x1 << 19);
	} else if(pUSARTx == UART5){
		RCC->APB1RSTR |= (0x1 << 20);
		RCC->APB1RSTR &= ~(0x1 << 20);
	} else if(pUSARTx == USART6){
		RCC->APB2RSTR |= (0x1 << 5);
		RCC->APB2RSTR &= ~(0x1 << 5);
	}
}

/*
 * This function transmits the data into the external world
 * This function implements a blocking call/ polling type API (non-interrupt) as the function will not return (will be blocked)
 * until all the data is sent to the external world
 *
 * @Input Handle structure of the USART peripheral
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Output void
 * */
void USART_TransmitData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length)
{
	uint16_t *pdata;
	for(uint32_t i = 0; i < length; i++)
	{
		//Wait until TXE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TXE_FLAG));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9)
		{
			//Load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//Check USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
			{
				//No parity is used in this transfer, therefore, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer, therefore, 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;
		}
	}
	//Wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG));
}

/*
 * This function receives the data from the external world
 * This function implements a blocking call (non-interrupt) to receive the data
 *
 * @Input Handle structure of the USART peripheral
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Output void
 * */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length)
{
	for(uint32_t i = 0; i < length; i++)
	{
		//Wait until RXNE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9)
		{
			//9bit data frame
			//check for USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
			{
				//No parity is used. so, all 9bits will be of user data
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else
		{
			//8bit data frame
			//check for USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
			{
				 *pRxBuffer = pUSARTHandle->pUSARTx->DR;
			}

			else
			{
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0X7F);
			}
			pRxBuffer++;
		}
	}
}

/*
 * This function transmits the data into the external world
 * This function implements a non-blocking call (interrupt based) to transmit the data
 *
 * @Input Handle structure of the USART peripheral
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Output returns the Tx state of the USART peripheral
 * */
uint8_t USART_TransmitDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t txstate = pUSARTHandle->TxState;
	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = length;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TXEIE);

		//Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TCIE);
	}
	return txstate;
}

/*
 * This function receives the data from the external world
 * This function implements a non-blocking call (interrupt based) to receive the data
 *
 * @Input Handle structure of the USART peripheral
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Output returns the Rx state of the USART peripheral
 * */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t rxstate = pUSARTHandle->RxState;
	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = length;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		//Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}


/*
 * This function sets the UE bit of USART Control Register 1
 * @Input USART peripheral base address
 * @Output void
 * */
void USART_Peri_EN(USART_RegDef_t* pUSARTx){
	pUSARTx->CR1 |= (0x1 << USART_CR1_UE);
}

/*
 * This function clears the UE bit of USART Control Register 1
 * @Input USART peripheral base address
 * @Output void
 * */
void USART_Peri_DI(USART_RegDef_t* pUSARTx){
	pUSARTx->CR1 &= ~(0x1 << USART_CR1_UE);
}

/*
 * This function gets the value of a particular flag of USART Status Register
 * @Input USART peripheral base address
 * @Input Flag name
 * @Output returns the status of the flag
 * */
uint8_t USART_getStatusFlag(USART_RegDef_t* pUSARTx, uint8_t flagName){
	return (pUSARTx->SR & (0x1 << flagName));
}

/*
 * This function sets the baud rate of the USART communication
 * @Input Base address of USART peripheral
 * @Input desired baud rate
 * @Output void
 * */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//APB clock value (PCLK1 or PCLK2)
	uint32_t PCLKx;
	uint32_t usartdiv;

	//Mantissa and Fraction values
	uint32_t M_part,F_part;

	//Store the value of APB bus clock into PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (0x1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 (Over sampling by 8)
		usartdiv = ((25*PCLKx) / (2*BaudRate));
	}else
	{
		//OVER8 = 2 (Over sampling by 16)
		usartdiv = ((25*PCLKx) / (4*BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position in USART_BRR
	pUSARTx->BRR &= ~(0xFFF << USART_BRR_DIVMANTISSA);
	pUSARTx->BRR |= (M_part << USART_BRR_DIVMANTISSA);

	//Extract the fraction part
	F_part = (usartdiv-(M_part * 100));

	//Calculate the final fractional value
	if(pUSARTx->CR1 & (0x1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 (Over sampling by 8)
		F_part = (((F_part*8)+50)/100)&((uint8_t)0x07);
	}else
	{
		//over sampling by 16
		F_part = (((F_part*16)+50)/100)&((uint8_t)0x0F);
	}

	//Place the fractional part in appropriate bit position in USART_BRR
	pUSARTx->BRR &= ~(0xF << USART_BRR_DIVFRACTION);
	pUSARTx->BRR |= (F_part << USART_BRR_DIVFRACTION);
}

/*
 * This function gets the value of a particular flag of USART Status Register
 * @Input USART peripheral base address
 * @Input Flag name
 * @Output returns the status of the flag
 * */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){
	return (pUSARTx->SR & FlagName);
}

/*
 * This function configures the IRQ for USART peripheral
 * @Input IRQ number
 * @Input IRQ priority
 * @Input IRQ value: Enable or Disable
 * @Output void
 * */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue){

	/*Enable or Disable the IRQ number*/
	if(IRQValue == ENABLE){
		if(IRQNumber <= 31){
			//ISER0 register
			*NVIC_ISER0_ADDR |= (0x1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64){
			//ISER1 register
			*NVIC_ISER1_ADDR |= (0x1 << (IRQNumber%32));
		} else if(IRQNumber >= 64 && IRQNumber < 96){
			//ISER2 register
			*NVIC_ISER2_ADDR |= (0x1 << (IRQNumber%64));
		}
	} else{
		if(IRQNumber <= 31){
			//ICER0 register
			*NVIC_ICER0_ADDR &= ~(0x1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64){
			//ICER1 register
			*NVIC_ICER1_ADDR &= ~(0x1 << (IRQNumber%32));
		} else if(IRQNumber >= 64 && IRQNumber < 96){
			//ICER2 register
			*NVIC_ICER2_ADDR &= ~(0x1 << (IRQNumber%64));
		}
	}


	/*Setting the priority of the IRQ number*/
	uint8_t arrayIdx = IRQNumber/4;
	uint8_t bitPos = (IRQNumber % 4)*8;

	NVIC_IPR_ADDR[arrayIdx] &= ~(0xFF << bitPos);
	NVIC_IPR_ADDR[arrayIdx] |= (IRQPriority << (bitPos+4));
}

/*
 * This function handles the interrupt/events generated in the processor
 * @Input Handle structure of the USART peripheral
 * @Output void
 * */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle){

	/*********TC FLAG**********/
	//Check the TC bit
	uint32_t temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TC);

	//Check the TCEIE bit
	uint32_t temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_TCIE);

	if(temp1 && temp2){
		//Interrupt occurred because of TC
		USART_TC_IRQHandler(pUSARTHandle);
	}

	/*********TXE FLAG**********/
	//Check the TXE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TXE);

	//Check the TXEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_TXEIE);

	if(temp1 && temp2){
		//Interrupt occurred because of TXE
		USART_TXE_IRQHandler(pUSARTHandle);
	}

	/*********RXNE FLAG**********/
	//Check the RXNE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_RXNE);

	//Check the RXNEIE control bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		//Interrupt occurred because of RXNE
		USART_RXNE_IRQHandler(pUSARTHandle);
	}

	/*********CTS FLAG**********/
	//Check the CTS bit
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_CTS);

	//Check the CTSE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR3_CTSE);

	//Check the CTSIE control bit
	uint32_t temp3 = pUSARTHandle->pUSARTx->CR3 & (0x1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3){
		//Interrupt occurred because of CTS
		//Clear the CTS bit
		pUSARTHandle->pUSARTx->SR &= ~(0x1 << USART_SR_CTS);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	/*********IDLE FLAG**********/
	//Check the IDLE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_IDLE);

	//Check the IDLEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_IDLEIE);

	if(temp1 && temp2){
		//Interrupt occurred because of IDLE
		//Clear the IDLE bit
		uint32_t readDR = pUSARTHandle->pUSARTx->DR;
		(void)readDR;

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/*********OVERRUN FLAG**********/
	//Check the ORE bit
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_ORE);

	//Check the RXNEIE control bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		//Interrupt occurred because of ORE
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ORE);
	}

	/*********ERROR FLAG**********/
	temp2 = pUSARTHandle->pUSARTx->CR3 & (0x1 << USART_CR3_EIE);

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (0x1 << USART_SR_FE))
		{
			/*
					This bit is set by hardware when a de-synchronization, excessive noise or a break character
					is detected. It is cleared by a software sequence (an read to the USART_SR register
					followed by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_FE);
		}

		if(temp1 & (0x1 << USART_SR_NE))
		{
			/*
					This bit is set by hardware when noise is detected on a received frame. It is cleared by a
					software sequence (an read to the USART_SR register followed by a read to the
					USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_NE);
		}

		if(temp1 & (0x1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERREVENT_ORE);
		}
	}
}

/*
 * This function handles the interrupt generated by the TC (transmission completed) flag
 * @Input Handle structure of the USART peripheral
 * @Output void
 * */
static void USART_TC_IRQHandler(USART_Handle_t *pUSARTHandle){
	if (pUSARTHandle->TxState == USART_BUSY_IN_TX)
	{
		//Check the TxLen
		if(!pUSARTHandle->TxLen)
		{
			//Clear the TC flag
			pUSARTHandle->pUSARTx->SR &= ~(0x1 << USART_SR_TC);

			//Clear the TCIE control bit
			pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_TCIE);

			//Reset the application state
			pUSARTHandle->TxState = USART_READY;

			//Reset Buffer address to NULL
			pUSARTHandle->pTxBuffer = NULL;

			//Reset the length to zero
			pUSARTHandle->TxLen = 0;

			//Call the application callback function
			USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
		}
	}
}

/*
 * This function handles the interrupt generated by the TXE (Tx buffer empty) flag
 * @Input Handle structure of the USART peripheral
 * @Output void
 * */
static void USART_TXE_IRQHandler(USART_Handle_t *pUSARTHandle){
	if(pUSARTHandle->TxState == USART_BUSY_IN_TX){
		if(pUSARTHandle->TxLen > 0){
			//Check the USART_WordLength item for 9BIT or 8BIT in a frame
			uint16_t *pdata;
			if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9)
			{
				//Load the DR with 2bytes masking the bits other than first 9 bits
				pdata = (uint16_t*)(pUSARTHandle->pTxBuffer);
				pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

				//Check USART_ParityControl
				if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
				{
					//No parity is used in this transfer, therefore, 9bits of user data will be sent
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
					pUSARTHandle->TxLen--;
				}
				else
				{
					//Parity bit is used in this transfer, therefore, 8bits of user data will be sent
					//The 9th bit will be replaced by parity bit by the hardware
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen--;
				}
			}
			else
			{
				pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer) & (uint8_t)0xFF);
				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen--;
			}
		}

		if(pUSARTHandle->TxLen == 0){
			//Clear the TXEIE control bit
			pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_TXEIE);
		}
	}
}

/*
 * This function handles the interrupt generated by the RXNE (Rx buffer not empty) flag
 * @Input Handle structure of the USART peripheral
 * @Output void
 * */
static void USART_RXNE_IRQHandler(USART_Handle_t *pUSARTHandle){
	if(pUSARTHandle->RxState == USART_BUSY_IN_RX){
		if(pUSARTHandle->RxLen > 0){
			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9)
			{
				//9bit data frame
				//check for USART_ParityControl
				if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
				{
					//No parity is used. so, all 9bits will be of user data
					*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
					pUSARTHandle->RxLen--;
				}
				else
				{
					//Parity is used, so, 8bits will be of user data and 1 bit is parity
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen--;
				}
			}
			else
			{
				//8bit data frame
				//check for USART_ParityControl
				if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DI)
				{
					*(pUSARTHandle->pRxBuffer) = pUSARTHandle->pUSARTx->DR;
				}

				else
				{
					*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0X7F);
				}
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
		}

		if(pUSARTHandle->RxLen == 0){
			//Clear the RXNEIE control bit
			pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_RXNEIE);
			pUSARTHandle->RxState = USART_READY;
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
		}
	}
}

/*
 * The function clears the ORE flag
 * @Input Base address of USART peripheral
 * @Output void
 * */
void USART_ClearOREFlag(USART_RegDef_t *pUSARTx){
	uint32_t temp1 = pUSARTx->SR;
	uint32_t temp2 = pUSARTx->DR;

	(void)temp1;
	(void)temp2;
}
