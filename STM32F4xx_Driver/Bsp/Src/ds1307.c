#include <string.h>
#include <stdint.h>
#include "ds1307.h"

I2C_Handle_t ds1307_i2c_handle;

/*Helper (private) functions*/
static void DS1307_I2C_GpioPinInit(void);
static void DS1307_I2C_Config(void);
static void DS1307_write(uint8_t dataValue, uint8_t registerAddr);
static uint8_t DS1307_read(uint8_t registerAddr);
static uint8_t ConvertBCDtoBinary(uint8_t bdcValue);
static uint8_t ConvertBinarytoBCD(uint8_t binaryValue);

/*Initialize the DS1307 RTC module*/
uint8_t DS1307_init(void){

	//1. Initialize the I2C pins
	DS1307_I2C_GpioPinInit();

	//2. Initialize the I2C peripheral
	DS1307_I2C_Config();

	//3. Enable the oscillator for DS1307 module (CH bit clear)
	DS1307_write(0x00, DS1307_ADDR_SEC);

	//4. Read the contents of the Seconds register to check if CH bit = 0
	uint8_t clockState = DS1307_read(DS1307_ADDR_SEC);

	/*
	 * If the return value if 1, then the CH bit is still set, otherwise it got cleared with our write operation
	 * CH=1: Init failed
	 * CH=0: Init successful
	 *
	 * */
	return ((clockState >> 7) & 0x1);
}

/*Set the current time*/
void DS1307_setCurrentTime(RTC_Time_t *pRTCTime){

	/*Set the current seconds*/
	uint8_t currentSeconds = ConvertBinarytoBCD(pRTCTime->seconds);
	//Clear the CH bit, otherwise the oscillator will get disabled
	currentSeconds &= ~(0x1 << 7);
	DS1307_write(currentSeconds, DS1307_ADDR_SEC);

	/*Set the current minutes*/
	uint8_t currentMinutes = ConvertBinarytoBCD(pRTCTime->minutes);
	DS1307_write(currentMinutes, DS1307_ADDR_MIN);

	/*Set the current hour with specifying time format*/
	uint8_t currentHours = ConvertBinarytoBCD(pRTCTime->hours);
	if(pRTCTime->timeFormat == TIME_FORMAT_12HRS_AM){
		//12 hours format
		currentHours |= (0x1 << 6);
		//AM
		currentHours &= ~(0x1 << 5);
	} else if(pRTCTime->timeFormat == TIME_FORMAT_12HRS_PM){
		//12 hours format
		currentHours |= (0x1 << 6);
		//PM
		currentHours |= (0x1 << 5);
	} else if(pRTCTime->timeFormat == TIME_FORMAT_24HRS){
		//24 hours format
		currentHours &= ~(0x1 << 6);
		//Re-enter the value
		currentHours = ConvertBinarytoBCD(pRTCTime->hours);
	}
	DS1307_write(currentHours, DS1307_ADDR_HRS);
}

/*Get the current time*/
void DS1307_getCurrentTime(RTC_Time_t *pRTCTime){

	/*Get the current seconds*/
	uint8_t currentSeconds = DS1307_read(DS1307_ADDR_SEC);
	//Clear the CH bit, otherwise the oscillator will get disabled
	currentSeconds &= ~(0x1 << 7);
	currentSeconds = ConvertBCDtoBinary(currentSeconds);

	/*Get the current minutes*/
	uint8_t currentMinutes = DS1307_read(DS1307_ADDR_MIN);
	currentMinutes = ConvertBCDtoBinary(currentMinutes);

	/*Get the current hours*/
	uint8_t currentHours = DS1307_read(DS1307_ADDR_HRS);
	if(currentHours & (0x1 << 6)){
		//12 hours format with AM/PM
		pRTCTime->timeFormat = !((currentHours & (0x1 << 5)) == 0);
	} else{
		//24 hours format
		pRTCTime->timeFormat = TIME_FORMAT_24HRS;
	}

	currentHours &= ~(0x3 < 5);
	currentHours = ConvertBCDtoBinary(currentHours);

	pRTCTime->seconds = currentSeconds;
	pRTCTime->minutes = currentMinutes;
	pRTCTime->hours = currentHours;
}

/*Set the current date*/
void DS1307_setCurrentDate(RTC_Date_t *pRTCDate){

	/*Set the current date*/
	uint8_t currentDate = ConvertBinarytoBCD(pRTCDate->date);
	DS1307_write(currentDate, DS1307_ADDR_DATE);

	/*Set the current month*/
	uint8_t currentMonth = ConvertBinarytoBCD(pRTCDate->month);
	DS1307_write(currentMonth, DS1307_ADDR_MONTH);

	/*Set the current year*/
	uint8_t currentYear = ConvertBinarytoBCD(pRTCDate->year);
	DS1307_write(currentYear, DS1307_ADDR_YEAR);

	/*Set the current day*/
	uint8_t currentDay = ConvertBinarytoBCD(pRTCDate->day);
	DS1307_write(currentDay, DS1307_ADDR_DAY);
}

/*Get the current date*/
void DS1307_getCurrentDate(RTC_Date_t *pRTCDate){

	/*Get the current date*/
	uint8_t currentDate = DS1307_read(DS1307_ADDR_DATE);
	currentDate = ConvertBCDtoBinary(currentDate);

	/*Get the current month*/
	uint8_t currentMonth = DS1307_read(DS1307_ADDR_MONTH);
	currentMonth = ConvertBCDtoBinary(currentMonth);

	/*Get the current year*/
	uint8_t currentYear = DS1307_read(DS1307_ADDR_YEAR);
	currentYear = ConvertBCDtoBinary(currentYear);

	/*Get the current day*/
	uint8_t currentDay = DS1307_read(DS1307_ADDR_DAY);
	currentDay = ConvertBCDtoBinary(currentDay);

	pRTCDate->date = currentDate;
	pRTCDate->month = currentMonth;
	pRTCDate->year = currentYear;
	pRTCDate->day = currentDay;
}

/*Initialize the GPIO pins that needs to be connected to the RTC module*/
static void DS1307_I2C_GpioPinInit(void){
	GPIO_Handle_t i2c_sda, i2c_scl;
	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/* I2C pins
	 * I2C1_SCL => PB6
	 * I2C1_SDA => PB7
	 * */

	/*Initialize I2C1_SCL pin*/
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIOPinConfig.GPIO_PinAltFuncMode = AF4;
	i2c_sda.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	i2c_sda.GPIOPinConfig.GPIO_PinNumber = DS1307_I2C_SCL;
	i2c_sda.GPIOPinConfig.GPIO_PinOPType = OUT_OPEN_DRAIN;
	i2c_sda.GPIOPinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;

	/*Initialize I2C1_SDA pin*/
	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIOPinConfig.GPIO_PinAltFuncMode = AF4;
	i2c_sda.GPIOPinConfig.GPIO_PinMode = ALT_FUNC_MODE;
	i2c_sda.GPIOPinConfig.GPIO_PinNumber = DS1307_I2C_SDA;
	i2c_sda.GPIOPinConfig.GPIO_PinOPType = OUT_OPEN_DRAIN;
	i2c_sda.GPIOPinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;

	GPIO_PeriClkControl(DS1307_I2C_GPIO_PORT, ENABLE);
	GPIO_Init(&i2c_sda);
	GPIO_Init(&i2c_sda);
}

/*Configure the I2C*/
static void DS1307_I2C_Config(void){
	ds1307_i2c_handle.pI2Cx = I2C1;

	ds1307_i2c_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	ds1307_i2c_handle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_PeriClkControl(DS1307_I2C, ENABLE);
	I2C_Init(&ds1307_i2c_handle);
}

/*Write into the DS1307 RTC module*/
static void DS1307_write(uint8_t dataValue, uint8_t registerAddr){
	uint8_t txData[2];
	txData[0] = registerAddr;
	txData[1] = dataValue;

	I2C_MasterTransmitData(&ds1307_i2c_handle, txData, sizeof(txData), DS1307_I2C_ADDR, I2C_STOP_EN);
}

/*Read from the DS1307 RTC module*/
static uint8_t DS1307_read(uint8_t registerAddr){
	uint8_t txData = registerAddr;
	uint8_t rxData;

	I2C_MasterTransmitData(&ds1307_i2c_handle, &txData, 1, DS1307_I2C_ADDR, I2C_STOP_DI);
	I2C_MasterReceiveData(&ds1307_i2c_handle, &rxData, 1, DS1307_I2C_ADDR, I2C_STOP_EN);

	return rxData;
}

/*Convert BCD to Binary value*/
static uint8_t ConvertBCDtoBinary(uint8_t bcdValue){
	uint8_t binaryValue;

	if(bcdValue >= 10){
		uint8_t t = (uint8_t)(bcdValue >> 4);
		uint8_t u = (uint8_t)(bcdValue & 0x0F);
		binaryValue = ((10*t) + u);
	} else{
		binaryValue = bcdValue;
	}

	return binaryValue;
}

/*Convert binary to BCD value*/
static uint8_t ConvertBinarytoBCD(uint8_t binaryValue){
	uint8_t bcdValue;

	if(binaryValue >= 10){
		uint8_t t = binaryValue/10;
		uint8_t u = binaryValue%10;
		bcdValue = (uint8_t)((t << 4) | u);
	} else{
		bcdValue = binaryValue;
	}

	return bcdValue;
}
