#ifndef MAIN_H_
#define MAIN_H_

#define INTERRUPT_DISABLE()	do{__asm volatile("MOV R0, 0X01");	__asm volatile("MSR PRIMASK, R0"); }while(0)
#define INTERRUPT_ENABLE() do{__asm volatile("MOV R0, 0X00");	__asm volatile("MSR PRIMASK, R0"); }while(0)

/* Stack memory calculation */
#define SIZE_TASK_STACK			1024U
#define SIZE_SCHEDULER_STACK 	1024U

#define SRAM_START				0x20000000U
#define SIZE_SRAM				((128)*(1024))
#define SRAM_END				((SRAM_START) + (SIZE_SRAM))

#define T1_STACK_START			SRAM_END
#define T2_STACK_START			((SRAM_END) - (1*SIZE_TASK_STACK))
#define T3_STACK_START			((SRAM_END) - (2*SIZE_TASK_STACK))
#define T4_STACK_START			((SRAM_END) - (3*SIZE_TASK_STACK))
#define IDLE_TASK_STACK_START	((SRAM_END) - (4*SIZE_TASK_STACK))
#define SCHEDULER_STACK_START	((SRAM_END) - (5*SIZE_TASK_STACK))

#define TICK_HZ					1000U
#define SYSTICK_TIMER_CLOCK_HZ	16000000U
#define SYST_RVR_ADDRESS		0xE000E014U
#define SYST_CSR_ADDRESS		0xE000E010U
#define ICSR_ADDRESS			0xE000ED04U

#define MAX_TASKS				5U
#define TASK_READY_STATE		0x00
#define TASK_BLOCKED_STATE		0x01

#define SHCSR_ADDRESS			0xE000ED24U

#define RCC_ABHIENR_ADDR 		((RCC_ABH1ENR_t*) 0x40023830)
#define GPIOD_MODE_ADDR 		((GPIOx_MODE_t*) 0x40020C00)
#define GPIOD_ODR_ADDR 			((GPIOx_ODR_t*) 0x40020C14)

#define CLOCK_ENABLE 	(1)
#define PIN_OUTPUT 		(1)
#define PIN_ENABLE 		(1)
#define PIN_DISABLE 	(0)

#define DELAY_COUNT_GREEN 	(16000000UL)
#define DELAY_COUNT_ORANGE 	(8000000UL)
#define DELAY_COUNT_RED 	(4000000UL)
#define DELAY_COUNT_BLUE 	(2000000UL)

typedef struct{
	uint32_t gpioa_en: 			1;
	uint32_t gpiob_en: 			1;
	uint32_t gpioc_en: 			1;
	uint32_t gpiod_en: 		    1;
	uint32_t gpioe_en: 			1;
	uint32_t gpiof_en: 			1;
	uint32_t gpiog_en: 			1;
	uint32_t gpioh_en: 			1;
	uint32_t gpioi_en: 			1;
	uint32_t reserved_1: 		3;
	uint32_t crc_en: 			1;
	uint32_t reserved_2: 		3;
	uint32_t reserved_3: 		2;
	uint32_t bkp_sram_en: 		1;
	uint32_t reserved_4: 		1;
	uint32_t ccm_dataram_en: 	1;
	uint32_t dma1_en: 			1;
	uint32_t dma2_en: 			1;
	uint32_t reserved_5: 		2;
	uint32_t ethmac_en: 		1;
	uint32_t ethmactx_en: 		1;
	uint32_t ethmacrx_en: 		1;
	uint32_t ethmacptp_en: 		1;
	uint32_t otghs_en: 			1;
	uint32_t otghsulpi_en: 		1;
	uint32_t reserved_6: 		1;
}RCC_ABH1ENR_t;

typedef struct{
	uint32_t pin0: 		2;
	uint32_t pin1: 		2;
	uint32_t pin2: 		2;
	uint32_t pin3: 		2;
	uint32_t pin4: 		2;
	uint32_t pin5: 		2;
	uint32_t pin6: 		2;
	uint32_t pin7: 		2;
	uint32_t pin8: 		2;
	uint32_t pin9: 		2;
	uint32_t pin10: 	2;
	uint32_t pin11: 	2;
	uint32_t pin12: 	2;
	uint32_t pin13: 	2;
	uint32_t pin14: 	2;
	uint32_t pin15: 	2;
}GPIOx_MODE_t;

typedef struct{
	uint32_t pin0: 		1;
	uint32_t pin1: 		1;
	uint32_t pin2: 		1;
	uint32_t pin3: 		1;
	uint32_t pin4: 		1;
	uint32_t pin5: 		1;
	uint32_t pin6: 		1;
	uint32_t pin7: 		1;
	uint32_t pin8: 		1;
	uint32_t pin9: 		1;
	uint32_t pin10: 	1;
	uint32_t pin11: 	1;
	uint32_t pin12: 	1;
	uint32_t pin13: 	1;
	uint32_t pin14: 	1;
	uint32_t pin15: 	1;
	uint32_t reserved:  16;
}GPIOx_ODR_t;

#endif /* MAIN_H_ */
