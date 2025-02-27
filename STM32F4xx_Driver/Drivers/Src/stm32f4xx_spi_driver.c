#include "stm32f4xx_spi_driver.h"

/*Helper (private) functions*/
static void SPI_TxIRQHandler(SPI_Handle_t *pSPIHandle);
static void SPI_RxIRQHandler(SPI_Handle_t *pSPIHandle);
static void SPI_OVRIRQHandler(SPI_Handle_t *pSPIHandle);

/*
 * The function enables or disables the peripheral clock for a given SPI peripheral
 * @Input SPI peripheral base address
 * @Input Clock value (Enable: 1 or Disable: 0)
 * @Output void
 * */
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t clkValue){
	if(clkValue == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		} else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		} else if(pSPIx == SPI6){
			SPI6_PCLK_EN();
		}
	} else {
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		} else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		} else if(pSPIx == SPI6){
			SPI6_PCLK_DI();
		}
	}
}

/*
 * This function initializes the appropriate peripheral registers to the SPI peripheral provided
 * @Input Handle structure of the SPI
 * @Output void
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//SPI Device Mode
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_MSTR);
	uint8_t deviceMode = pSPIHandle->SPIConfig.SPI_DeviceMode;
	pSPIHandle->pSPIx->CR1 |= (deviceMode << SPI_CR1_MSTR);

	//SPI Bus Configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDIMODE bit should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDIMODE bit should be set
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDIMODE bit should be cleared and RXONLY bit should be set
		pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_RXONLY);
	}

	//SPI Clock Speed
	pSPIHandle->pSPIx->CR1 &= ~(0x7 << SPI_CR1_BR);
	uint8_t clkSpeed = pSPIHandle->SPIConfig.SPI_SclkSpeed;
	pSPIHandle->pSPIx->CR1 |= (clkSpeed << SPI_CR1_BR);

	//SPI DFF
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_DFF);
	uint8_t dff = pSPIHandle->SPIConfig.SPI_DFF;
	pSPIHandle->pSPIx->CR1 |= (dff << SPI_CR1_DFF);

	//SPI CPOL
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_CPOL);
	uint8_t cpol = pSPIHandle->SPIConfig.SPI_CPOL;
	pSPIHandle->pSPIx->CR1 |= (cpol << SPI_CR1_CPOL);

	//SPI CPHA
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_CPHA);
	uint8_t cpha = pSPIHandle->SPIConfig.SPI_CPHA;
	pSPIHandle->pSPIx->CR1 |= (cpha << SPI_CR1_CPHA);

	//SPI SSM
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSM);
	uint8_t ssm = pSPIHandle->SPIConfig.SPI_SSM;
	pSPIHandle->pSPIx->CR1 |= (ssm << SPI_CR1_SSM);
	if(ssm == SPI_SSM_SW){
		if(deviceMode == SPI_MASTER){
			//SSI bit should be HIGH (as NSS is influenced by SSI)
			pSPIHandle->pSPIx->CR1 |= (0x1 << SPI_CR1_SSI);
		} else{
			//SSI bit should be LOW
			pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSI);
		}
	}
}

/*
 * This function resets the appropriate peripheral registers to their corresponding reset values
 * @Input Base address of SPI peripheral
 * @Output void
 * */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		RCC->APB2RSTR |= (0x1 << 12);
		RCC->APB2RSTR &= ~(0x1 << 12);
	} else if(pSPIx == SPI2){
		RCC->APB1RSTR |= (0x1 << 14);
		RCC->APB1RSTR &= ~(0x1 << 14);
	} else if(pSPIx == SPI3){
		RCC->APB1RSTR |= (0x1 << 15);
		RCC->APB1RSTR &= ~(0x1 << 15);
	}
}

/*
 * This function transmits the data into the external world
 * This function implements a blocking call/ polling type API (non-interrupt) as the function will not return (will be blocked)
 * until all the data is sent to the external world
 *
 * @Input Base address of SPI peripheral
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Output void
 * */
void SPI_transmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t sizeOfData){
	while(sizeOfData > 0){
		uint8_t isTXbufferEmpty = (pSPIx->SR & (0x1 << SPI_SR_TXE));
		while(!isTXbufferEmpty);
		uint8_t dff = (pSPIx->CR1 & (0x1 << SPI_CR1_DFF));
		if(dff){
			//16 bit data format
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			sizeOfData--;
			sizeOfData--;
			(uint16_t*)pTxBuffer++;
		} else {
			//8 bit data format
			pSPIx->DR = *pTxBuffer;
			sizeOfData--;
			pTxBuffer++;
		}
	}
}

/*
 * This function transmits the data into the external world
 * This function implements a non-blocking call (interrupt based) to transmit the data
 *
 * @Input Handle structure of the SPI peripheral
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Output returns the Tx state of the SPI peripheral
 * */
uint8_t SPI_transmitData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t sizeOfData){
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		//Storing the txBuffer address and the length of data to be used by the ISR
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = sizeOfData;

		//Mark the SPI state as busy in Tx
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable the interrupt when TxBuffer is empty
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_TXEIE);

		//Data transmission will be handled by the ISR
	}
	return (pSPIHandle->TxState);
}

/*
 * This function receives the data from the external world
 * This function implements a blocking call (non-interrupt) to receive the data
 *
 * @Input Base address of SPI peripheral
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Output void
 * */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t sizeOfData){
	while(sizeOfData > 0){
		uint8_t isRXbufferFull = (pSPIx->SR & (0x1 << SPI_SR_RXNE));
		while(!isRXbufferFull);
		uint8_t dff = (pSPIx->CR1 & (0x1 << SPI_CR1_DFF));
		if(dff){
			//16 bit data format
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			sizeOfData--;
			sizeOfData--;
			(uint16_t*)pRxBuffer++;
		} else {
			//8 bit data format
			*pRxBuffer = pSPIx->DR;
			sizeOfData--;
			pRxBuffer++;
		}
	}
}

/*
 * This function receives the data from the external world
 * This function implements a non-blocking call (interrupt based) to receive the data
 *
 * @Input Handle structure of the SPI peripheral
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Output returns the Rx state of the SPI peripheral
 * */
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t sizeOfData){
	if(pSPIHandle->RxState != SPI_BUSY_IN_RX){
		//Storing the RxBuffer address and the length of the data
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = sizeOfData;

		//Mark the SPI state as busy in Rx
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable the interrupt when RxBuffer is full
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_RXNEIE);

		//Data reception will be handled by the ISR
	}
	return (pSPIHandle->RxState);
}

/*
 * This function configures the IRQ for SPI peripheral
 * @Input IRQ number
 * @Input IRQ priority
 * @Input IRQ value: Enable or Disable
 * @Output void
 * */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue){

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
 * This function handles the IRQ triggered by the SPI
 * @Input Handle structure of the SPI
 * @Output void
 * */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	/*Checking the TXE bit*/
	uint8_t t1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_TXE);
	uint8_t t2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_TXEIE);
	if(t1 && t2){
		//Tx triggered the interrupt
		SPI_TxIRQHandler(pSPIHandle);
	}

	/*Checking the RXNE bit*/
	t1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_RXNE);
	t2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_RXNEIE);
	if(t1 && t2){
		//Rx triggered the interrupt
		SPI_RxIRQHandler(pSPIHandle);
	}

	/*Checking the OVR bit*/
	t1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_OVR);
	t2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_ERRIE);
	if(t1 && t2){
		//Overrun triggered the interrupt
		SPI_OVRIRQHandler(pSPIHandle);
	}
}

/*
 * This function handles the interrupt triggered because of TxBuffer
 * @Input SPI peripheral handle
 * @Output void
 * */
static void SPI_TxIRQHandler(SPI_Handle_t *pSPIHandle){
	uint8_t dff = (pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF));
	if(dff){
		//16 bit data format
		pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen --;
		pSPIHandle->TxLen --;
		(uint16_t*)(pSPIHandle->pTxBuffer)++;
	} else {
		//8 bit data format
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen --;
		(pSPIHandle->pTxBuffer)++;
	}

	if(!(pSPIHandle->TxLen)){
		//Close the SPI TX
		SPI_CloseTx(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/*
 * This function handles the interrupt triggered because of RxBuffer
 * @Input SPI peripheral handle
 * @Output void
 * */
static void SPI_RxIRQHandler(SPI_Handle_t *pSPIHandle){
	uint8_t dff = (pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF));
	if(dff){
		//16 bit data format
		*((uint16_t*)(pSPIHandle->pRxBuffer)) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)(pSPIHandle->pRxBuffer)++;
	} else {
		//8 bit data format
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		(pSPIHandle->pRxBuffer)++;
	}

	if(!(pSPIHandle->RxLen)){
		//Close the SPI RX
		SPI_CloseRx(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*
 * This function handles the interrupt triggered because of Overrun error
 * @Input SPI peripheral handle
 * @Output void
 * */
static void SPI_OVRIRQHandler(SPI_Handle_t *pSPIHandle){
	uint8_t receivedData;
	//Clear the OVR bit
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		receivedData = pSPIHandle->pSPIx->DR;
		receivedData = pSPIHandle->pSPIx->SR;
	}
	(void)receivedData;

	//Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_CMPLT);
}

/*
 * This function, called by the application, clears the OVR bit of SPI Status Register
 * @Input SPI peripheral base address
 * @Output void
 * */
void SPI_Clear_OVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t receivedData;
	receivedData = pSPIx->DR;
	receivedData = pSPIx->SR;
	(void)receivedData;
}

/*
 * This function closes the SPI Tx
 * @Input SPI peripheral handle
 * @Output void
 * */
void SPI_CloseTx(SPI_Handle_t *pSPIHandle){
	//Disable the TxBuffer interrupt
	pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_TXEIE);

	//Reset the buffers
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
}

/*
 * This function closes the SPI Rx
 * @Input SPI peripheral handle
 * @Output void
 * */
void SPI_CloseRx(SPI_Handle_t *pSPIHandle){
	//Disable the RxBuffer interrupt
	pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_RXNEIE);

	//Reset the buffers
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
}

/*
 * This function sets the SPE bit of SPI Control Register 1
 * @Input SPI peripheral base address
 * @Output void
 * */
void SPI_Peri_EN(SPI_RegDef_t *pSPIx){
	pSPIx->CR1 |= (0x1 << SPI_CR1_SPE);
}

/*
 * This function clears the SPE bit of SPI Control Register 1
 * @Input SPI peripheral base address
 * @Output void
 * */
void SPI_Peri_DI(SPI_RegDef_t *pSPIx){
	pSPIx->CR1 &= ~(0x1 << SPI_CR1_SPE);
}

/*
 * This function sets the SSOE bit of SPI Control Register 2
 * @Input SPI peripheral base address
 * @Output void
 * */
void SPI_SSOutput_EN(SPI_RegDef_t *pSPIx){
	pSPIx->CR2 |= (0x1 << SPI_CR2_SSOE);
}

/*
 * This function clears the SSOE bit of SPI Control Register 2
 * @Input SPI peripheral base address
 * @Output void
 * */
void SPI_SSOutput_DI(SPI_RegDef_t *pSPIx){
	pSPIx->CR2 &= ~(0x1 << SPI_CR2_SSOE);
}

/*
 * This function gets the value of a particular flag of SPI Status Register
 * @Input SPI peripheral base address
 * @Input Flag name
 * @Output returns the status of the flag
 * */
uint8_t SPI_getStatusFlag(SPI_RegDef_t *pSPIx, uint8_t flagName){
	return (pSPIx->SR & (0x1 << flagName));
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event){
	//Weak implementation. The application should override this function
}
