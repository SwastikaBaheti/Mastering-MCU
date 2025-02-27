#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include "stm32f4xx.h"

/*This file contains the device related information like register information, function prototypes and
 * application configurable items related to DS1307, a RTC (Real-time clock) module
 * */

/*Register Addresses of DS1307 RTC module*/
#define DS1307_ADDR_SEC				0x00U
#define DS1307_ADDR_MIN				0x01U
#define DS1307_ADDR_HRS				0x02U
#define DS1307_ADDR_DAY				0x03U
#define DS1307_ADDR_DATE			0x04U
#define DS1307_ADDR_MONTH			0x05U
#define DS1307_ADDR_YEAR			0x06U

/*Time format macros*/
#define TIME_FORMAT_12HRS_AM		0
#define TIME_FORMAT_12HRS_PM		1
#define TIME_FORMAT_24HRS			2

/*Slave address*/
#define DS1307_I2C_ADDR				0x68U

/*Day macros*/
#define SUNDAY						1
#define MONDAY						2
#define TUESDAY						3
#define WEDNESDAY					4
#define THURSDAY					5
#define FRIDAY						6
#define SATURDAY					7

/*Structures to hold the time and date information*/
typedef struct{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_Date_t;

typedef struct{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t timeFormat;
}RTC_Time_t;

/*API prototypes for the application to use*/

uint8_t DS1307_init(void);
void DS1307_setCurrentTime(RTC_Time_t *pRTCTime);
void DS1307_getCurrentTime(RTC_Time_t *pRTCTime);

void DS1307_setCurrentDate(RTC_Date_t *pRTCDate);
void DS1307_getCurrentDate(RTC_Date_t *pRTCDate);

/*Application configurable macros*/
#define DS1307_I2C					I2C1 /*I2C peripheral to which the MCU will be connected to the RTC module*/
#define DS1307_I2C_GPIO_PORT		GPIOB
#define DS1307_I2C_SDA				GPIO_PIN_7
#define DS1307_I2C_SCL				GPIO_PIN_6
#define DS1307_I2C_SPEED			I2C_SCL_STANDARD
#define DS1307_I2C_PUPD				ONLY_PU

#endif /* INC_DS1307_H_ */
