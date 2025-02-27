#include "stm32f4xx_rcc_driver.h"

uint16_t AHB_preScalerValues[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_preScalerValues[4] = {2, 4, 8, 16};
uint16_t APB2_preScalerValues[4] = {2, 4, 8, 16};

/*
 * The function returns the value of peripheral clock supplied to the APB1 bus
 * @Output clock value
 * */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, systemClock;
	uint8_t ahbPrescaler, apb1PreScaler;

	//Determine the SYSCLK based on the source
	uint8_t clkSrc = (RCC->CFGR & (0x3 << 2) >> 2);
	if(clkSrc == 0){
		//HSI
		systemClock = 16000000U;
	} else if(clkSrc == 1){
		//HSE
		systemClock = 8000000U;
	} else if(clkSrc == 2){
		//PLL
		systemClock = RCC_GetPLLClk();
	}

	uint8_t temp1 = (RCC->CFGR & (0xF << 4) >> 4);
	if(temp1 < 8){
		ahbPrescaler = 1;
	} else {
		ahbPrescaler = AHB_preScalerValues[temp1-8];
	}

	uint8_t temp2 = (RCC->CFGR & (0x7 << 10) >> 10);
	if(temp2 < 4){
		apb1PreScaler = 1;
	} else {
		apb1PreScaler = APB1_preScalerValues[temp2-4];
	}

	pclk1 = ((systemClock/ahbPrescaler)/apb1PreScaler);
	return pclk1;
}

/*
 * The function returns the value of peripheral clock supplied to the APB2 bus
 * @Output clock value
 * */
uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2, systemClock;
	uint8_t ahbPrescaler, apb2PreScaler;

	//Determine the SYSCLK based on the source
	uint8_t clkSrc = (RCC->CFGR & (0x3 << 2) >> 2);
	if(clkSrc == 0){
		//HSI
		systemClock = 16000000U;
	} else if(clkSrc == 1){
		//HSE
		systemClock = 8000000U;
	} else if(clkSrc == 2){
		//PLL
		systemClock = RCC_GetPLLClk();
	}

	uint8_t temp1 = (RCC->CFGR & (0xF << 4) >> 4);
	if(temp1 < 8){
		ahbPrescaler = 1;
	} else {
		ahbPrescaler = AHB_preScalerValues[temp1-8];
	}

	uint8_t temp2 = (RCC->CFGR & (0x7 << 13) >> 13);
	if(temp2 < 4){
		apb2PreScaler = 1;
	} else {
		apb2PreScaler = APB2_preScalerValues[temp2-4];
	}

	pclk2 = ((systemClock/ahbPrescaler)/apb2PreScaler);
	return pclk2;
}

uint32_t RCC_GetPLLClk(void){
	return 0;
}
