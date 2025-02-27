#include <string.h>
#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK		16000000UL

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter
}

char* get_day(uint8_t i){
	char* days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	return days[i-1];
}

void number_to_string(uint8_t num, char* addr){
	if(num < 10){
		addr[0] = '0';
		addr[1] = num+48;
	} else{
		addr[0] = (num/10)+48;
		addr[1] = (num%10)+48;
	}
}

char* time_to_string(RTC_Time_t *pRTCTime){
	//hh:mm:ss format

	static char time[9];
	time[2] = ':';
	time[5] = ':';

	number_to_string(pRTCTime->hours, &time[0]);
	number_to_string(pRTCTime->minutes, &time[3]);
	number_to_string(pRTCTime->seconds, &time[6]);

	time[8] = '\0';

	return time;
}

char* date_to_string(RTC_Date_t *pRTCDate){
	//dd/mm/yy format

	static char date[9];
	date[2] = '/';
	date[5] = '/';

	number_to_string(pRTCDate->date, &date[0]);
	number_to_string(pRTCDate->month, &date[3]);
	number_to_string(pRTCDate->year, &date[6]);

	date[8] = '\0';

	return date;
}

int main(void){

	RTC_Time_t rtcTime;
	RTC_Date_t rtcDate;

	/*Initialize the LCD*/
	LCD_init();

	LCD_send_string("DS1307 RTC Module Testing\n");

	if(DS1307_init()){
		//Non-zero value which means CH=1
		printf("Initialization failed for DS1307 RTC module");
		while(1);
	}

	//Zero value which means CH=0

	/*Initialize the Systick timer*/
	init_systick_timer(1);

	rtcTime.hours = 10;
	rtcTime.minutes = 27;
	rtcTime.seconds = 0;
	rtcTime.timeFormat = TIME_FORMAT_12HRS_PM;

	rtcDate.date = 25;
	rtcDate.month = 2;
	rtcDate.year = 25;
	rtcDate.day = TUESDAY;

	DS1307_setCurrentTime(&rtcTime);
	DS1307_setCurrentDate(&rtcDate);

	while(1);
}

void SysTick_Handler(void){
	//Interrupt handler for systick interrupt

	RTC_Time_t rtcTime;
	RTC_Date_t rtcDate;

	/*Clearing the LCD display*/
	LCD_clear_display();
	LCD_display_return_home();

	/*Get the time related information and print*/
	DS1307_getCurrentTime(&rtcTime);

	char *am_pm;
	if(rtcTime.timeFormat != TIME_FORMAT_24HRS){
		//12 hour format
		am_pm = (rtcTime.timeFormat)? "PM": "AM";
		printf("Current Time: %s %s\n", time_to_string(&rtcTime), am_pm);

		//Printing the current time on LCD in the 1st row
		LCD_set_cursor(1,1);
		LCD_send_string(time_to_string(&rtcTime));
		LCD_send_string(am_pm);
	} else{
		printf("Current Time: %s\n", time_to_string(&rtcTime));

		//Printing the current time on LCD in the 1st row
		LCD_set_cursor(1,1);
		LCD_send_string(time_to_string(&rtcTime));
	}

	/*Get the date related information and print*/
	DS1307_getCurrentDate(&rtcDate);
	printf("Current Date: %s <%s>\n", date_to_string(&rtcDate), get_day(rtcDate.day));

	//Printing the current date on LCD in the 2nd row
	LCD_set_cursor(2,1);
	LCD_send_string(date_to_string(&rtcDate));
	LCD_send_char('<');
	LCD_send_string(get_day(rtcDate.day));
	LCD_send_char('>');
}
