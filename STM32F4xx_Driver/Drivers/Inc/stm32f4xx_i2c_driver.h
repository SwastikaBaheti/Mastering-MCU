#ifndef INC_STM32F4XX_I2C_DRIVER_H_
#define INC_STM32F4XX_I2C_DRIVER_H_

#include "stm32f4xx.h"

/*I2C Configuration structure for I2C peripheral*/

typedef struct{
	uint32_t I2C_SCLSpeed;				//From @I2C_SCL_SPEEDS
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;				//From @I2C_ACK_CONTROL
	uint8_t I2C_FMDutyCycle;			//From @I2C_FMDutyCycle
}I2C_Config_t;

/*I2C Handle structure for I2C peripheral*/

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t txLen;
	uint32_t rxLen;
	uint8_t deviceAddr;			//To store the device/slave address
	uint8_t txRxState;
	uint32_t rxSize;
	uint8_t isNotRepeatedStart;
}I2C_Handle_t;

/*
 * Prototype for the APIs supported by the SPI peripheral
*/

/*I2C Peripheral clock setup*/
void I2C_PeriClkControl(I2C_RegDef_t *pI2Cx, uint8_t clkValue);

/*Initialize or De-initialize I2C*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*Data transmit and Receive
 * Case-1: MASTER
 * Case-2: SLAVE*/
void I2C_MasterTransmitData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop);

/*These APIs return the application state*/
uint8_t I2C_MasterTransmitDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t length, uint8_t slaveAddress, uint8_t isStop);

void I2C_SlaveTransmitData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);

/*IRQ configuration and IRQ handling for EV (events) and ER (errors)*/
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_SetIRQControlBits(I2C_RegDef_t *pI2Cx);
void I2C_ClearIRQControlBits(I2C_RegDef_t *pI2Cx);

/*Other I2C peripheral control APIs*/
void I2C_Peri_EN(I2C_RegDef_t *pI2Cx);
void I2C_Peri_DI(I2C_RegDef_t *pI2Cx);

uint8_t I2C_getStatusFlag(I2C_RegDef_t *pI2Cx, uint32_t flagName);

void I2C_ClearACKbit(I2C_RegDef_t *pI2Cx);
void I2C_SetACKbit(I2C_RegDef_t *pI2Cx);

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*Application callback function*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);

/*I2C Status Flags*/
#define I2C_SB_FLAG			(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG		(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG		(1 << I2C_SR1_BTF)
#define I2C_TXE_FLAG		(1 << I2C_SR1_TXE)
#define I2C_RXNE_FLAG		(1 << I2C_SR1_RXNE)
#define I2C_STOPF_FLAG		(1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG		(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG		(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG			(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG		(1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG 	(1 << I2C_SR1_TIMEOUT)

/*I2C application states*/
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*SPI application events*/
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR		3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/*
 * User friendly macros
*/

/*@I2C_SCL_SPEEDS
 * I2C clock speed*/
#define I2C_SCL_STANDARD	100000U
#define I2C_SCL_FAST2K		200000U
#define I2C_SCL_FAST4K		400000U

/*@I2C_ACK_CONTROL
 * I2C ack control*/
#define I2C_ACK_DISABLE		0
#define I2C_ACK_ENABLE		1

/*@I2C_FMDutyCycle
 * I2C FM Duty Cycle*/
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16BY9	1

#define I2C_STOP_DI			0
#define I2C_STOP_EN			1

#endif /* INC_STM32F4XX_I2C_DRIVER_H_ */
