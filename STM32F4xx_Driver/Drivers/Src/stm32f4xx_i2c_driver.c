#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_rcc_driver.h"

/*Helper (private) functions*/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDRBit(I2C_RegDef_t *pI2Cx);

static void I2C_SB_IRQHandler(I2C_Handle_t *pI2CHandle);
static void I2C_ADDR_IRQHandler(I2C_Handle_t *pI2CHandle);
static void I2C_BTF_IRQHandler(I2C_Handle_t *pI2CHandle);
static void I2C_STOPF_IRQHandler(I2C_Handle_t *pI2CHandle);
static void I2C_TxE_IRQHandler(I2C_Handle_t *pI2CHandle);
static void I2C_RxNE_IRQHandler(I2C_Handle_t *pI2CHandle);


/*
 * The function enables or disables the peripheral clock for a given I2C peripheral
 * @Input I2C peripheral base address
 * @Input Clock value (Enable: 1 or Disable: 0)
 * @Output void
 * */
void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t clkValue){
	if(clkValue == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	} else {
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		} else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		} else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

/*
 * This function initializes the appropriate peripheral registers to the I2C peripheral provided
 * @Input Handle structure of the I2C
 * @Output void
 * */
void I2C_Init(I2C_Handle_t *pI2CHandle){

	//ACK enabling
	uint8_t ackControl = pI2CHandle->I2C_Config.I2C_ACKControl;
	pI2CHandle->pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= (ackControl << I2C_CR1_ACK);

	//Serial clock (SCL) configuration
	pI2CHandle->pI2Cx->CR2 &= ~(0x3F << I2C_CR2_FREQ);
	uint32_t apb1BusClk = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (apb1BusClk << I2C_CR2_FREQ);

	//CCR calculations
	uint16_t ccr_value = 0;
	uint32_t sclSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;

	if(sclSpeed <= I2C_SCL_STANDARD){
		//Standard mode (duty cycle = 50%)
		pI2CHandle->pI2Cx->CCR &= ~(0x1 << I2C_CCR_FSMODE);
		ccr_value = RCC_GetPCLK1Value()/(2*sclSpeed);
	} else {
		//Fast mode (Duty cycle = 2 or 16/9)
		pI2CHandle->pI2Cx->CCR |= (0x1 << I2C_CCR_FSMODE);

		//FM duty cycle
		uint8_t dutyCycle = pI2CHandle->I2C_Config.I2C_FMDutyCycle;
		pI2CHandle->pI2Cx->CCR &= ~(0x1 << I2C_CCR_DUTY);
		pI2CHandle->pI2Cx->CCR |= (dutyCycle << I2C_CCR_DUTY);

		if(dutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value()/(3*sclSpeed);
		} else if(dutyCycle == I2C_FM_DUTY_16BY9){
			ccr_value = RCC_GetPCLK1Value()/(25*sclSpeed);
		}
	}

	//CRR configuration
	pI2CHandle->pI2Cx->CCR &= ~(0xFFF << I2C_CCR_CCR);
	pI2CHandle->pI2Cx->CCR |= (ccr_value << I2C_CCR_CCR);

	//Device address (when the device is slave)
	uint8_t slaveAddr = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	pI2CHandle->pI2Cx->OAR1 &= ~(0xFE << I2C_OAR1_ADDR);
	pI2CHandle->pI2Cx->OAR1 |= (slaveAddr << I2C_OAR1_ADDR);

	//TRISE configuration
	uint8_t trise=0;
	if(sclSpeed <= I2C_SCL_STANDARD){
		//Standard mode
		trise = ((RCC_GetPCLK1Value()*1)/1000000U)+1;
	} else {
		//Fast mode
		trise = ((RCC_GetPCLK1Value()*0.3)/1000000U)+1;
	}
	pI2CHandle->pI2Cx->TRISE &= ~(0x3F << I2C_TRISE);
	pI2CHandle->pI2Cx->TRISE |= (trise << I2C_TRISE);
}

/*
 * This function resets the appropriate peripheral registers to their corresponding reset values
 * @Input Base address of I2C peripheral
 * @Output void
 * */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		RCC->APB1RSTR |= (0x1 << 21);
		RCC->APB1RSTR &= ~(0x1 << 21);
	} else if(pI2Cx == I2C2){
		RCC->APB1RSTR |= (0x1 << 22);
		RCC->APB1RSTR &= ~(0x1 << 22);
	} else if(pI2Cx == I2C3){
		RCC->APB1RSTR |= (0x1 << 23);
		RCC->APB1RSTR &= ~(0x1 << 23);
	}
}

/*
 * This function sets the PE bit of I2C Control Register 1
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_Peri_EN(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (0x1 << I2C_CR1_PE);
}

/*
 * This function resets the PE bit of I2C Control Register 1
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_Peri_DI(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 &= ~(0x1 << I2C_CR1_PE);
}

/*
 * This function clears the ACK bit and thus disables the acking functionality
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_ClearACKbit(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
}

/*
 * This function sets the ACK bit and thus enables the acking functionality
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_SetACKbit(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
}

/*
 * This function gets the value of a particular flag of I2C Status Register
 * @Input I2C peripheral base address
 * @Input Flag name
 * @Output returns the status of the flag
 * */
uint8_t I2C_getStatusFlag(I2C_RegDef_t *pI2Cx, uint32_t flagName){
	return (pI2Cx->SR1 & flagName);
}

/*
 * This function transmits the data into the external world
 * This function implements a blocking call/ polling type API (non-interrupt) as the function will not return (will be blocked)
 * until all the data is sent to the external world
 *
 * @Input I2C peripheral handle structure
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Input slave address
 * @Input stop bit or repeated start
 * @Output void
 * */
void I2C_MasterTransmitData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop){
	//Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Wait till the SB bit is set
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//Send the slave address with r/nw bit as 0 (WRITE) [Total 8 bits]
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddress);

	//Confirm that the ACK bit is received by checking the ADDR bit (it should be set)
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//Clearing the ADDR bit by reading the SR1 and SR2 registers
	I2C_ClearADDRBit(pI2CHandle->pI2Cx);

	//Load the data into the Data Register till length becomes 0
	while(length > 0){
		while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

		//Once the TXE bit is set, the data register is empty
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		length--;
	}

	//When length becomes 0, wait for TXE=1 and BTF=1 before generating STOP condition
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_BTF_FLAG));

	if(isStop == ENABLE){
		//Generate the STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/*
 * This function receives the data from the external world
 * This function implements a blocking call (non-interrupt) to receive the data
 *
 * @Input I2C peripheral handler structure
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Input slave address
 * @Input stop bit or repeated start
 * @Output void
 * */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop){
	//Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Wait till the SB bit is set
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//Send the slave address with r/nw bit as 1 (READ) [Total 8 bits]
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddress);

	//Confirm that the ACK bit is received by checking the ADDR bit (it should be set)
	while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	if(length == 1){
		//Clearing the ACK bit
		I2C_ClearACKbit(pI2CHandle->pI2Cx);

		//Clearing the ADDR bit by reading the SR1 and SR2 registers
		I2C_ClearADDRBit(pI2CHandle->pI2Cx);

		while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		if(isStop){
			//Setting the STOP bit
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Reading the data from the Data Register
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(length > 1){
		//Clearing the ADDR bit by reading the SR1 and SR2 registers
		I2C_ClearADDRBit(pI2CHandle->pI2Cx);

		while(length > 0){
			//Wait till the RXNE bit is set
			while(!I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			if(length == 2){
				//Clearing the ACK bit
				I2C_ClearACKbit(pI2CHandle->pI2Cx);

				if(isStop){
					//Setting the STOP bit
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//Once the RXNE bit is set, the data register is full
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
			length--;
		 }
	}
	//Revert to initial condition
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE){
		//Setting the ACK bit
		I2C_SetACKbit(pI2CHandle->pI2Cx);
	}
}

/*
 * This function transmits the data into the external world
 * This function implements a non-blocking call (interrupt based) to transmit the data
 *
 * @Input Handle structure of the I2C peripheral
 * @Input Pointer to the data which has to be transmitted
 * @Input size of the data that the user wants to send (in bytes)
 * @Input slave address
 * @Input stop bit or repeated start
 * @Output returns the Tx state of the I2C peripheral
 * */
uint8_t I2C_MasterTransmitDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop){
	uint8_t state = pI2CHandle->txRxState;

	if((state != I2C_BUSY_IN_RX) && (state != I2C_BUSY_IN_TX)){
		pI2CHandle->deviceAddr = slaveAddress;
		pI2CHandle->isNotRepeatedStart = isStop;
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = length;
		pI2CHandle->txRxState = I2C_BUSY_IN_TX;

		//Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//Enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);

		//Enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
	}
	return state;
}

/*
 * This function receives the data from the external world
 * This function implements a non-blocking call (interrupt based) to receive the data
 *
 * @Input Handle structure of the I2C peripheral
 * @Input Pointer to the data receive address
 * @Input size of the data (in bytes)
 * @Input slave address
 * @Input stop bit or repeated start
 * @Output returns the Rx state of the I2C peripheral
 * */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop){
	uint8_t state = pI2CHandle->txRxState;

	if((state != I2C_BUSY_IN_RX) && (state != I2C_BUSY_IN_TX)){
		pI2CHandle->deviceAddr = slaveAddress;
		pI2CHandle->isNotRepeatedStart = isStop;
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = length;
		pI2CHandle->rxSize = length;
		pI2CHandle->txRxState = I2C_BUSY_IN_RX;

		//Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable the ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

		//Enable the ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);

		//Enable the ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
	}
	return state;
}

/*
 * This function sends the data in the slave mode (the data is sent byte by byte after receiving an ACK from master)
 * @Input I2C peripheral base address
 * @Input data (1 byte)
 * @Output void
 * */
void I2C_SlaveTransmitData(I2C_RegDef_t *pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

/*
 * This function receives the data in the slave mode
 * (the data is sent byte by byte from the master as after every byte an ACK has to be returned by the slave)
 * @Input I2C peripheral base address
 * @Output data (1 byte)
 * */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return pI2Cx->DR;
}

/*
 * This function configures the IRQ for I2C peripheral
 * @Input IRQ number
 * @Input IRQ priority
 * @Input IRQ value: Enable or Disable
 * @Output void
 * */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue){

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
 * This function handles the Event interrupt triggered by the I2C peripheral
 * @Input Handle structure of the I2C
 * @Output void
 * */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt Handler for events generated by both master and slave

	uint8_t temp1 = (pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITEVTEN));
	uint8_t temp2 = (pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITBUFEN));

	// 1. Handle interrupt generated by SB (Start bit) event
	// Note: SB is only applicable in Master mode
	uint8_t temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_SB_FLAG);
	if(temp1 && temp3){
		// SB bit is set
		I2C_SB_IRQHandler(pI2CHandle);
	}

	// 2. Handle interrupt generated by ADDR (Address bit) event
	// Note: When master mode: Address is successfully sent
	//		 When slave mode: Address is matched successfully
	temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_ADDR_FLAG);
	if(temp1 && temp3){
		// ADDR flag is set
		I2C_ADDR_IRQHandler(pI2CHandle);
	}

	// 3. Handle interrupt generated by BTF (Byte Transfer Finished) event
	temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_BTF_FLAG);
	if(temp1 && temp3){
		// BTF is set
		I2C_BTF_IRQHandler(pI2CHandle);
	}

	// 4. Handle interrupt generated by STOPF event
	// Note: Only applicable to slave
	temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_STOPF_FLAG);
	if(temp1 && temp3){
		// STOPF is set
		I2C_STOPF_IRQHandler(pI2CHandle);
	}

	// 5. Handle interrupt generated by TxE event
	temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_TXE_FLAG);
	if(temp1 && temp2 && temp3){
		//TxE is set
		I2C_TxE_IRQHandler(pI2CHandle);
	}

	// 6. Handle interrupt generated by RxNE event
	temp3 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_RXNE_FLAG);
	if(temp1 && temp2 && temp3){
		//RxNE is set
		I2C_RxNE_IRQHandler(pI2CHandle);
	}
}

/*
 * This function handles the Error interrupt triggered by the I2C peripheral
 * @Input Handle structure of the I2C
 * @Output void
 * */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt Handler for errors generated by both master and slave

	uint8_t temp1 = (pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITEVTEN));

	//1. Checking for any bus error (BERR)
	uint8_t temp2 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_BERR_FLAG);
	if(temp1 && temp2){
		//Bus error occurred
		//Clearing the bus error (BERR) bit
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_BERR);

		//Notify the application about the bus error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	//2. Checking for any arbitration loss error (ARLO)
	temp2 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_ARLO_FLAG);
	if(temp1 && temp2){
		//Arbitration loss error occurred
		//Clearing the arbitration loss (ARLO) bit
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_ARLO);

		//Notify the application about the arbitration loss error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//3. Checking for any ACK failure error (AF)
	temp2 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_AF_FLAG);
	if(temp1 && temp2){
		//ACK failure error occurred
		//Clearing the ACK failure (AF) bit
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_AF);

		//Notify the application about the ACK failure error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//4. Checking for any overrun/underrun error (OVR)
	temp2 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_OVR_FLAG);
	if(temp1 && temp2){
		//overrun/underrun error occurred
		//Clearing the overrun/underrun (OVR) bit
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_OVR);

		//Notify the application about the overrun/underrun error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	//5. Checking for any timeout error (TIMEOUT)
	temp2 = I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_TIMEOUT_FLAG);
	if(temp1 && temp2){
		//Timeout error occurred
		//Clearing the timeout (TIMEOUT) bit
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_TIMEOUT);

		//Notify the application about the timeout error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/*
 * This function generates the start bit necessary for starting the I2C transmission/reception
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
}

/*
 * This function generates the stop bit necessary for ending the I2C transmission/reception
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);
}

/*
 * This function sets the control bits necessary for generating the interrupt
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_SetIRQControlBits(I2C_RegDef_t *pI2Cx){
	//Enable the ITBUFEN control bit
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);

	//Enable the ITEVFEN control bit
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);

	//Enable the ITERREN control bit
	pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
}

/*
 * This function clears the control bits
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_ClearIRQControlBits(I2C_RegDef_t *pI2Cx){
	//Disable the ITBUFEN control bit
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);

	//Disable the ITEVFEN control bit
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	//Enable the ITERREN control bit
	pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);
}

/*
 * This function closes the transmission and initializes the members to their initial values
 * @Input I2C peripheral base address
 * @Output void
 * */
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle){
	//Disable the ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);

	//Disable the ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;
	pI2CHandle->txRxState = I2C_READY;
}

/*
 * This function handles the interrupt generated by the Start bit (SB)
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_SB_IRQHandler(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->txRxState == I2C_BUSY_IN_TX){
		I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->deviceAddr);
	} else if(pI2CHandle->txRxState == I2C_BUSY_IN_RX){
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->deviceAddr);
	}
}

/*
 * This function handles the interrupt generated by the ADDR bit
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_ADDR_IRQHandler(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->txRxState == I2C_BUSY_IN_RX){
		if(pI2CHandle->rxSize == 1){
			//Clear the ACK bit before clearing the ADDR bit
			I2C_ClearACKbit(pI2CHandle->pI2Cx);
		}
	}
	//Clearing the ADDR bit
	I2C_ClearADDRBit(pI2CHandle->pI2Cx);
}

/*
 * This function handles the interrupt generated by the BTF (byte transmitted flag)
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_BTF_IRQHandler(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->txRxState == I2C_BUSY_IN_TX){
		//Check the status of the TxE bit and the length of data that was supposed to be transmitted
		if((I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_TXE_FLAG)) && (pI2CHandle->txLen == 0)){
			// BTF, TxE are set then STOP the transmission and reset the handler structure

			if(pI2CHandle->isNotRepeatedStart){
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			} else{
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
			}

			I2C_CloseTransmission(pI2CHandle);

			//Notify the application about the end of TX
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}
	} else if(pI2CHandle->txRxState == I2C_BUSY_IN_RX){
		//Check the status of the RxNE bit
		if(I2C_getStatusFlag(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)){
			// BTF, RxNE are set then nothing has to be done as this is not how the reception is closed
			return;
		}
	}
}

/*
 * This function handles the interrupt generated by the STOPF bit (valid in slave mode)
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_STOPF_IRQHandler(I2C_Handle_t *pI2CHandle){
	//Clearing the STOPF flag: Read SR1 (already done while storing temp3) and write to CR1
	pI2CHandle->pI2Cx->CR1 |= 0x0000;

	//Notify the application about the detection of STOP condition by master
	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
}

/*
 * This function handles the interrupt generated by the TXE bit
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_TxE_IRQHandler(I2C_Handle_t *pI2CHandle){
	//Check for the device mode
	if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL)){
		//MASTER mode
		if(pI2CHandle->txRxState == I2C_BUSY_IN_TX){
			//Load the data into the Data Register and update the handle structure for I2C
			if(pI2CHandle->txLen > 0){
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				pI2CHandle->pTxBuffer++;
				pI2CHandle->txLen--;
			}
		}
	} else{
		//SLAVE mode
		//Checking the mode of the slave device (if the slave is a transmitter or receiver)
		if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA)){
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}
}

/*
 * This function handles the interrupt generated by the RXNE bit
 * @Input I2C peripheral handle structure
 * @Output void
 * */
static void I2C_RxNE_IRQHandler(I2C_Handle_t *pI2CHandle){
	//Checking the device mode
	if(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL)){
		//MASTER mode
		if(pI2CHandle->txRxState == I2C_BUSY_IN_RX){
			//Checking the length of the data that has to be received
			if(pI2CHandle->rxSize == 1){
				//Reading the data from the Data Register
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->rxLen--;
			}

			if(pI2CHandle->rxSize > 1){
				if(pI2CHandle->rxLen == 2){
					//Clearing the ACK bit
					I2C_ClearACKbit(pI2CHandle->pI2Cx);
				}

				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->rxLen--;
			}

			if(pI2CHandle->rxLen == 0){
				//Close the I2C data reception and notify the application
				if(pI2CHandle->isNotRepeatedStart){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				} else{
					I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
				}

				//Disable the ITBUFEN control bit
				pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);

				//Disable the ITEVFEN control bit
				pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

				pI2CHandle->pRxBuffer = NULL;
				pI2CHandle->rxLen = 0;
				pI2CHandle->rxSize = 0;
				pI2CHandle->txRxState = I2C_READY;

				if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE){
					//Enable the ACK bit
					I2C_SetACKbit(pI2CHandle->pI2Cx);
				}

				//Notify the application about the end of TX
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
		}
	} else{
		//SLAVE mode
		//Checking the mode of the slave device (if the slave is a transmitter or receiver)
		if(!(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA))){
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}
}

/*
 * This function generates the address phase in case of master transmitting the data to the slave
 * @Input I2C peripheral base address
 * @Input slave address
 * @Output void
 * */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	uint8_t addressPhase = (slaveAddr << 1);
	addressPhase &= ~(0x00000001);
	pI2Cx->DR = addressPhase;
}

/*
 * This function generates the address phase in case of master receiving the data from the slave
 * @Input I2C peripheral base address
 * @Input slave address
 * @Output void
 * */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	uint8_t addressPhase = (slaveAddr << 1);
	addressPhase |= (0x01);
	pI2Cx->DR = addressPhase;
}

/*
 * This function clears the ADDR bit
 * @Input I2C peripheral base address
 * @Output void
 * */
static void I2C_ClearADDRBit(I2C_RegDef_t *pI2Cx){
	uint32_t statusReg = pI2Cx->SR1;
	statusReg = pI2Cx->SR2;
	(void)statusReg;
}
