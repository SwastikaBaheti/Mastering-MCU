#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include "stm32f4xx.h"

/*Configuration structure for a SPI peripheral*/

typedef struct{
	uint8_t SPI_DeviceMode;			//From @SPI_DEVICE_MODES
	uint8_t SPI_BusConfig;			//From @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;			//From @SPI_CLK_SPEED
	uint8_t SPI_DFF;				//From @SPI_DATA_FRAME_FORMAT
	uint8_t SPI_CPOL;				//From @SPI_CPOL
	uint8_t SPI_CPHA;				//From @SPI_CPHA
	uint8_t SPI_SSM;				//FROM @SPI_SLAVE_M
}SPI_Config_t;

/*Handle structure for a SPI peripheral*/

typedef struct{
	SPI_RegDef_t* pSPIx;	//Base address of SPIx (SPI1, SPI2 or SPI3)
	SPI_Config_t SPIConfig;	//Different configurations of SPI
	uint8_t *pTxBuffer;		//To store the Tx buffer address
	uint8_t *pRxBuffer;		//To store the Rx buffer address
	uint32_t TxLen;			//To store the Tx data length
	uint32_t RxLen;			//To store the Rx data length
	uint8_t TxState;		//To store the Tx state
	uint8_t RxState;		//To store the Rx state
}SPI_Handle_t;

/*
 * Prototype for the APIs supported by the SPI peripheral
*/

/*SPI Peripheral clock setup*/
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t clkValue);

/*Initialize or De-initialize SPI*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Data transmit and Receive
 *Method-1: Blocking call (Non-interrupt)
 *Method-2: Non-blocking call (Interrupt based)*/

void SPI_transmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t sizeOfData);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t sizeOfData);

uint8_t SPI_transmitData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t sizeOfData);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t sizeOfData);


/*IRQ configuration and IRQ handling*/
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t IRQValue);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/*Other SPI peripheral control APIs*/
void SPI_Peri_EN(SPI_RegDef_t *pSPIx);
void SPI_Peri_DI(SPI_RegDef_t *pSPIx);
void SPI_SSOutput_EN(SPI_RegDef_t *pSPIx);
void SPI_SSOutput_DI(SPI_RegDef_t *pSPIx);
uint8_t SPI_getStatusFlag(SPI_RegDef_t *pSPIx, uint8_t flagName);
void SPI_Clear_OVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTx(SPI_Handle_t *pSPIHandle);
void SPI_CloseRx(SPI_Handle_t *pSPIHandle);

/*Application callback function*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);

/*
 * User friendly macros
*/

/*@SPI_DEVICE_MODES
 * SPI device modes*/
#define SPI_MASTER		1
#define SPI_SLAVE		0

/*@SPI_BUS_CONFIG
 * SPI bus configuration*/
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*@SPI_CLK_SPEED
 * SPI clock speed*/
#define SPI_SCLK_DIVIDE_2		0
#define SPI_SCLK_DIVIDE_4		1
#define SPI_SCLK_DIVIDE_8		2
#define SPI_SCLK_DIVIDE_16		3
#define SPI_SCLK_DIVIDE_32		4
#define SPI_SCLK_DIVIDE_64		5
#define SPI_SCLK_DIVIDE_128		6
#define SPI_SCLK_DIVIDE_256		7

/*@SPI_DATA_FRAME_FORMAT
 * SPI clock speed*/
#define SPI_DFF_BYTE			0
#define SPI_DFF_HALF_WORD		1

/*@SPI_CPOL
 * SPI clock polarity*/
#define SPI_CPOL_LOW			0
#define SPI_CPOL_HIGH			1

/*@SPI_CPHA
 * SPI clock phase*/
#define SPI_CPHA_LOW			0
#define SPI_CPHA_HIGH			1

/*@SPI_SLAVE_M
 * SPI slave management*/
#define SPI_SSM_HW				0
#define SPI_SSM_SW				1

/*SPI peripheral states*/
#define SPI_READY				0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2

/*SPI application events*/
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_CMPLT		3

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
