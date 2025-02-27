/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "main.h"

void task1_handler(void);	//Task-1
void task2_handler(void);	//Task-2
void task3_handler(void);	//Task-3
void task4_handler(void);	//Task-4

void init_systick_timer(uint32_t);
void init_scheduler_stack(uint32_t);
void init_tasks_stack(void);
void switch_to_psp(void);
void enable_processor_faults(void);
void set_next_task(void);
void init_all_LEDs(void);
void init_tasks(void);
void task_delay(uint32_t);
void idle_task(void);
void update_global_tick_count(void);
void unblock_tasks(void);
void schedule(void);

uint8_t current_task = 1;
uint32_t global_tick_count = 0;

typedef struct {
	uint32_t psp_value;
	uint8_t current_state;
	uint32_t block_count;
	void (*task_handler)(void);
}TCB_t;

TCB_t user_tasks[MAX_TASKS];

RCC_ABH1ENR_t volatile *const pClockRegister = RCC_ABHIENR_ADDR;
GPIOx_MODE_t volatile *const pGpiodModeRegister = GPIOD_MODE_ADDR;
GPIOx_ODR_t volatile *const pGpiodOutputDataRegister = GPIOD_ODR_ADDR;

int main(void)
{
	//Enabling processor faults
	enable_processor_faults();

	//Initialize the scheduler stack pointer
	init_scheduler_stack(SCHEDULER_STACK_START);

	//Initialize tasks
	init_tasks();

	//Initialize stack frames for all the tasks
	init_tasks_stack();

	//Initialize the LED pins and ports
	init_all_LEDs();

	//Configure the sysTick timer
	init_systick_timer(TICK_HZ);

	//Setting PSP as the stack pointer in thread mode
	switch_to_psp();

	//Launch Task-1
	task1_handler();

    /* Loop forever */
	for(;;);
}

void idle_task(void){
	while(1){
		printf("Idle Task running");
	}
}

void task1_handler(void){
	//Toggle green LED in every 1 second
	while(1){
		pGpiodOutputDataRegister->pin12 ^= PIN_DISABLE;
		task_delay(1000);
	}
}

void task2_handler(void){
	//Toggle orange LED in every 0.5 second
	while(1){
		pGpiodOutputDataRegister->pin13 ^= PIN_DISABLE;
		task_delay(500);
	}
}

void task3_handler(void){
	//Toggle red LED in every 0.25 second
	while(1){
		pGpiodOutputDataRegister->pin14 ^= PIN_DISABLE;
		task_delay(250);
	}
}

void task4_handler(void){
	//Toggle blue LED in every 0.125 second
	while(1){
		pGpiodOutputDataRegister->pin15 ^= PIN_DISABLE;
		task_delay(125);
	}
}

void init_systick_timer(uint32_t tick_hz){
	uint32_t countValue = (SYSTICK_TIMER_CLOCK_HZ/tick_hz)-1;
	uint32_t *pSYST_RVR = (uint32_t *)SYST_RVR_ADDRESS;
	uint32_t *pSYST_CSR = (uint32_t *)SYST_CSR_ADDRESS;

	//Clear the value of SYST_RVR
	*pSYST_RVR &= ~(0x00FFFFFF);

	//Loading the count value in SYST_RVR
	*pSYST_RVR |= countValue;

	//Enable the systick Timer
	*pSYST_CSR |= (0x00000007);
}

void update_global_tick_count(void){
	global_tick_count += 1;
}

void unblock_tasks(void){
	for(int i=1; i<MAX_TASKS; i++){
		if(user_tasks[i].current_state != TASK_READY_STATE){
			if(user_tasks[i].block_count == global_tick_count){
				user_tasks[i].current_state = TASK_READY_STATE;
			}
		}
	}
}

void SysTick_Handler(void){

	//Update the global tick count
	update_global_tick_count();

	//Unblock task
	unblock_tasks();

	//Pend pendSV using Interrupt Control and Status Register
	schedule();
}

__attribute__((naked))void PendSV_Handler(void){

	/*1. Saving the data of the current task*/
	__asm volatile("MRS R0, PSP");
	//Storing the registers
	__asm volatile("STMDB R0!, {R4-R11}");
	//Storing the PSP value
	__asm volatile("PUSH {LR}");
	__asm volatile("BL save_psp_value");

	/*2. Retrieve the data of the next task*/
	//Update the next task
	__asm volatile("BL set_next_task");
	//PSP value of the next task
	__asm volatile("BL get_psp");
	//Load the values of the register from the stack
	__asm volatile("LDMIA R0!, {R4-R11}");
	//Update the value of the PSP
	__asm volatile("MSR PSP, R0");
	__asm volatile("POP {LR}");
	__asm volatile ("BX LR");
}

__attribute__((naked)) void init_scheduler_stack(uint32_t schedularStackBaseAddress){
	__asm volatile("MSR MSP, r0");
	__asm volatile("BX LR");
}

void init_tasks_stack(void){
	uint32_t *pPSP;
	for(int i=0; i< MAX_TASKS; i++){
		pPSP = (uint32_t *)user_tasks[i].psp_value;
		pPSP--;
		*pPSP = 0x01000000;	//Storing xPSR
		pPSP--;
		*pPSP = (uint32_t)user_tasks[i].task_handler;	//Storing PC(Return Address)
		pPSP--;
		*pPSP = 0xFFFFFFFD;	//Storing LR

		//Storing the registers from R0-R12
		for(int j=0; j<13; j++){
			pPSP--;
			*pPSP = 0x0;
		}
		//Save the current PSP value
		user_tasks[i].psp_value = (uint32_t)pPSP;
	}
}

uint32_t get_psp(void){
	return user_tasks[current_task].psp_value;
}

void set_next_task(void){
	for(int i=0; i< MAX_TASKS; i++){
		current_task++;
		current_task %= MAX_TASKS;
		if((user_tasks[current_task].current_state == TASK_READY_STATE) && (current_task !=0)){
			break;
		}
	}

	if(user_tasks[current_task].current_state != TASK_READY_STATE){
		current_task = 0;
	}
}

void save_psp_value(uint32_t psp_value){
	user_tasks[current_task].psp_value = psp_value;
}

__attribute__((naked)) void switch_to_psp(void){
	__asm volatile("PUSH {LR}");

	__asm volatile("BL get_psp");
	__asm volatile("MSR PSP, r0");
	__asm volatile("POP {LR}");

	__asm volatile("MOV r0, #0x02");
	__asm volatile("MSR CONTROL, r0");

	__asm volatile("BX LR");
}

void enable_processor_faults(void){
	uint32_t *pSHCSR = (uint32_t *)SHCSR_ADDRESS;
	*pSHCSR |= (0x07 << 16);
}

void BusFault_Handler(void){
	printf("Bus Fault Exception took place");
}

void MemManage_Handler(void){
	printf("Memory Manage Fault Exception took place");
}

void UsageFault_Handler(void){
	printf("Usage Fault Exception took place\n");

	uint32_t *pUFSR = (uint32_t*)0xE000ED2AU;
	printf("The fault information is: %lx", (*pUFSR)&0xFFFF);
}

void HardFault_Handler(void){
	printf("Hard Fault Exception took place");
}

void init_all_LEDs(void){
	//Enable the clock for GPIOD
	pClockRegister->gpiod_en = CLOCK_ENABLE;

	//Configure the GPIOD pin 12 as OUTPUT
	pGpiodModeRegister->pin12 = PIN_OUTPUT;

	//Configure the GPIOD pin 13 as OUTPUT
	pGpiodModeRegister->pin13 = PIN_OUTPUT;

	//Configure the GPIOD pin 14 as OUTPUT
	pGpiodModeRegister->pin14 = PIN_OUTPUT;

	//Configure the GPIOD pin 15 as OUTPUT
	pGpiodModeRegister->pin15 = PIN_OUTPUT;

	//Set the pin 12 as 0
	pGpiodOutputDataRegister->pin12 = PIN_DISABLE;

	//Set the pin 13 as 0
	pGpiodOutputDataRegister->pin13 = PIN_DISABLE;

	//Set the pin 14 as 0
	pGpiodOutputDataRegister->pin14 = PIN_DISABLE;

	//Set the pin 15 as 0
	pGpiodOutputDataRegister->pin15 = PIN_DISABLE;
}

void init_tasks(void){

	//Initialize user task-1
	user_tasks[0].current_state = TASK_READY_STATE;
	user_tasks[0].psp_value = IDLE_TASK_STACK_START;
	user_tasks[0].task_handler = idle_task;

	//Initialize user task-1
	user_tasks[1].current_state = TASK_READY_STATE;
	user_tasks[1].psp_value = T1_STACK_START;
	user_tasks[1].task_handler = task1_handler;

	//Initialize user task-2
	user_tasks[2].current_state = TASK_READY_STATE;
	user_tasks[2].psp_value = T2_STACK_START;
	user_tasks[2].task_handler = task2_handler;

	//Initialize user task-3
	user_tasks[3].current_state = TASK_READY_STATE;
	user_tasks[3].psp_value = T3_STACK_START;
	user_tasks[3].task_handler = task3_handler;

	//Initialize user task-4
	user_tasks[4].current_state = TASK_READY_STATE;
	user_tasks[4].psp_value = T4_STACK_START;
	user_tasks[4].task_handler = task4_handler;
}

void task_delay(uint32_t tick_count){

	//Disable all the interrupts
	INTERRUPT_DISABLE();

	if(current_task){
		user_tasks[current_task].block_count = global_tick_count + tick_count;
		user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
		schedule();
	}

	//Enable all the interrupts
	INTERRUPT_ENABLE();
}

void schedule(void){
	uint32_t *pICSR = (uint32_t *)ICSR_ADDRESS;
	*pICSR |= (1<<28);
}
