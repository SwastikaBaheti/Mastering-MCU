#include "stm32f4xx_gpio_driver.h"

/*
 * The function enables or disables the peripheral clock for a given GPIO port
 * @Input GPIO port base address
 * @Input Clock value (Enable: 1 or Disable: 0)
 * @Output void
 * */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t clkValue){
	if(clkValue == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		} else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	} else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		} else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * This function initializes the appropriate peripheral registers to the GPIO provided
 * @Input Handle structure of a GPIO
 * @Output void
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint8_t pinNumber = pGPIOHandle->GPIOPinConfig.GPIO_PinNumber;

	//Mode
	uint8_t pinMode = pGPIOHandle->GPIOPinConfig.GPIO_PinMode;
	if(pinMode <=3){
		//Non-interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pinNumber*2));
		pGPIOHandle->pGPIOx->MODER |= (pinMode << (pinNumber*2));
	} else {
		//Interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pinNumber*2));

		if(pinMode == IN_FE_MODE){
			//Falling edge
			EXTI->FTSR |= (0x1 << pinNumber); //Set
			EXTI->RTSR &= ~(0x1 << pinNumber); //Clear
		} else if(pinMode == IN_RE_MODE){
			//Rising edge
			EXTI->RTSR |= (0x1 << pinNumber); //Set
			EXTI->FTSR &= ~(0x1 << pinNumber); //Clear
		} else if(pinMode == IN_RFE_MODE){
			//Rising + Falling edge
			EXTI->FTSR |= (0x1 << pinNumber); //Set
			EXTI->RTSR |= (0x1 << pinNumber); //Set
		}

		//Select the particular port to issue interrupt on a particular EXTI line
		SYSCFG_PCLK_EN();
		uint8_t arrayIdx = pinNumber/4;
		SYSCFG->EXTICR[arrayIdx] &= ~(0xF << ((pinNumber % 4)*4));
		if(pGPIOHandle->pGPIOx == GPIOA){
			SYSCFG->EXTICR[arrayIdx] |= (0x0 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOB){
			SYSCFG->EXTICR[arrayIdx] |= (0x1 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOC){
			SYSCFG->EXTICR[arrayIdx] |= (0x2 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOD){
			SYSCFG->EXTICR[arrayIdx] |= (0x3 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOE){
			SYSCFG->EXTICR[arrayIdx] |= (0x4 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOF){
			SYSCFG->EXTICR[arrayIdx] |= (0x5 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOG){
			SYSCFG->EXTICR[arrayIdx] |= (0x6 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOH){
			SYSCFG->EXTICR[arrayIdx] |= (0x7 << ((pinNumber % 4)*4));
		} else if(pGPIOHandle->pGPIOx == GPIOI){
			SYSCFG->EXTICR[arrayIdx] |= (0x8 << ((pinNumber % 4)*4));
		}

		//Un-mask the EXTI line
		EXTI->IMR |= (0x1 << pinNumber);
	}

	if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode == OUTPUT_MODE){
		//Output type
		uint8_t outputType = pGPIOHandle->GPIOPinConfig.GPIO_PinOPType;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= (outputType << pinNumber);

		//Output Speed
		uint8_t outputSpeed = pGPIOHandle->GPIOPinConfig.GPIO_PinSpeed;
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (pinNumber*2));
		pGPIOHandle->pGPIOx->OSPEEDR |= (outputSpeed << (pinNumber*2));
	}

	//Pull-up Pull-down settings
	uint8_t pupd = pGPIOHandle->GPIOPinConfig.GPIO_PinPuPdControl;
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pinNumber*2));
	pGPIOHandle->pGPIOx->PUPDR |= (pupd << (pinNumber*2));

	//Alternate functionality
	if(pinMode == ALT_FUNC_MODE){
		uint8_t altFuncNumber = pGPIOHandle->GPIOPinConfig.GPIO_PinAltFuncMode;
		if(pinNumber <= GPIO_PIN_7){
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (pinNumber*4));
			pGPIOHandle->pGPIOx->AFRL |= (altFuncNumber << (pinNumber*4));
		} else{
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (pinNumber*4));
			pGPIOHandle->pGPIOx->AFRH |= (altFuncNumber << ((pinNumber%8)*4));
		}
	}
}

/*
 * This function resets the appropriate peripheral registers to their corresponding reset values
 * @Input GPIO port base address
 * @Output void
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		RCC->AHB1RSTR |= (0x1 << 0);
		RCC->AHB1RSTR &= ~(0x1 << 0);
	} else if(pGPIOx == GPIOB){
		RCC->AHB1RSTR |= (0x1 << 1);
		RCC->AHB1RSTR &= ~(0x1 << 1);
	} else if(pGPIOx == GPIOC){
		RCC->AHB1RSTR |= (0x1 << 2);
		RCC->AHB1RSTR &= ~(0x1 << 2);
	} else if(pGPIOx == GPIOD){
		RCC->AHB1RSTR |= (0x1 << 3);
		RCC->AHB1RSTR &= ~(0x1 << 3);
	} else if(pGPIOx == GPIOE){
		RCC->AHB1RSTR |= (0x1 << 4);
		RCC->AHB1RSTR &= ~(0x1 << 4);
	} else if(pGPIOx == GPIOF){
		RCC->AHB1RSTR |= (0x1 << 5);
		RCC->AHB1RSTR &= ~(0x1 << 5);
	} else if(pGPIOx == GPIOG){
		RCC->AHB1RSTR |= (0x1 << 6);
		RCC->AHB1RSTR &= ~(0x1 << 6);
	} else if(pGPIOx == GPIOH){
		RCC->AHB1RSTR |= (0x1 << 7);
		RCC->AHB1RSTR &= ~(0x1 << 7);
	} else if(pGPIOx == GPIOI){
		RCC->AHB1RSTR |= (0x1 << 8);
		RCC->AHB1RSTR &= ~(0x1 << 8);
	}
}

/*
 * This function reads the data from the GPIO Input pin
 * @Input GPIO port base address
 * @Input GPIO pin number
 * @Output Input value: 0 or 1 (boolean)
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber){
	uint8_t inputValue = (uint8_t)((pGPIOx->IDR >> GPIOPinNumber) & 0x00000001);
	return inputValue;
}

/*
 * This function reads the data from the GPIO port
 * @Input GPIO port base address
 * @Output Input value
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return (uint16_t)pGPIOx->IDR;
}

/*
 * This function writes the data (boolean) to the GPIO Output pin
 * @Input GPIO port base address
 * @Input GPIO pin number
 * @Input data (boolean)
 * @Output void
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber, uint8_t value){
	pGPIOx->ODR &= ~(0x1 << GPIOPinNumber);
	pGPIOx->ODR |= (value << GPIOPinNumber);
}

/*
 * This function writes the data to the GPIO Output port
 * @Input GPIO port base address
 * @Input data
 * @Output void
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/*
 * This function toggles the GPIO Output pin
 * @Input GPIO port base address
 * @Input GPIO pin number
 * @Output void
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t GPIOPinNumber){
	pGPIOx->ODR ^= (0x1 << GPIOPinNumber);
}

/*
 * This function configures the IRQ
 * @Input IRQ number
 * @Input IRQ priority
 * @Input IRQ value: Enable or Disable
 * @Output void
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue){

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
 * This function handles the IRQ triggered by the GPIO
 * @Input GPIO pin number which triggered the IRQ
 * @Output void
 * */
void GPIO_IRQHandling(uint8_t GPIOPinNumber){
	//Clear the EXTI Pending Register
	EXTI->PR |= (0x1 << GPIOPinNumber);
}
