#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx.h"

/*API prototypes for the application to use*/
void LCD_init(void);
void LCD_clear_display(void);
void LCD_display_return_home(void);
void LCD_send_command(uint8_t cmd);
void LCD_send_string(char* message);
void LCD_send_char(uint8_t data);
void LCD_set_cursor(uint8_t row, uint8_t column);

/*Application configurable macros*/
#define LCD_GPIO_PORT 				GPIOD
#define LCD_GPIO_RS					GPIO_PIN_0
#define LCD_GPIO_RW					GPIO_PIN_1
#define LCD_GPIO_EN					GPIO_PIN_2
#define LCD_GPIO_D4					GPIO_PIN_3
#define LCD_GPIO_D5					GPIO_PIN_4
#define LCD_GPIO_D6					GPIO_PIN_5
#define LCD_GPIO_D7					GPIO_PIN_6

/*LCD Command Codes*/
#define LCD_CMD_4DL_2N_5X8F			0x28	//Function set command
#define LCD_CMD_DON_CURON			0x0E	//Display ON, Cursor ON
#define LCD_CMD_INCADD				0x06	//Increment (entry mode set)
#define LCD_CMD_DIS_CLEAR			0x01	//Display clear
#define LCD_CMD_DIS_RETURN_HOME		0x02	//Display return home

#endif /* INC_LCD_H_ */
