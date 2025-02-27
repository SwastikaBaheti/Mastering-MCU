#include "lcd.h"

static void mdelay(uint8_t delayValue);
static void udelay(uint8_t delayValue);
static void write_bits(uint8_t data);
static void lcd_enable(void);

/*Initializing the LCD*/
void LCD_init(void){

	/*1. Configure the GPIO pins connected to the LCD*/
	GPIO_Handle_t lcd_gpio_handler;
	lcd_gpio_handler.pGPIOx = LCD_GPIO_PORT;

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinMode = OUTPUT_MODE;
	lcd_gpio_handler.GPIOPinConfig.GPIO_PinOPType = OUT_PUSH_PULL;
	lcd_gpio_handler.GPIOPinConfig.GPIO_PinPuPdControl = NO_PUPD;
	lcd_gpio_handler.GPIOPinConfig.GPIO_PinSpeed = HIGH_SPEED;

	GPIO_PeriClkControl(LCD_GPIO_PORT, ENABLE);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_gpio_handler);

	lcd_gpio_handler.GPIOPinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_gpio_handler);

	//Setting the state of output pins as LOW initially
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, 0);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, 0);

	/*2. LCD initialization (initially sending instructions)*/

	//Give 40ms delay to the LCD after powering the LCD
	mdelay(40);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, 0); //RS=0: Instruction register
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, 0); //RW=0: Writing into the LCD

	//Set the values of D4-D7 and RS, RW bits
	write_bits(0x3);

	//Give 4.1ms delay to the LCD
	mdelay(5);

	//Set the values of D4-D7 and RS, RW bits
	write_bits(0x3);

	//Give 100 microseconds delay to the LCD
	udelay(150);

	//Set the values of D4-D7 and RS, RW bits
	write_bits(0x3);

	//Set the values of D4-D7 and RS, RW bits
	write_bits(0x2);

	/*3. Sending LCD commands*/
	LCD_send_command(LCD_CMD_4DL_2N_5X8F);
	LCD_send_command(LCD_CMD_DON_CURON);
	LCD_clear_display();
	LCD_send_command(LCD_CMD_INCADD);
}

/*Send the command code (1 byte) to the LCD*/
void LCD_send_command(uint8_t cmd){

	/* NOTE
	 * RS=0 for LCD command
	 * RW=0 while writing the command
	 * */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, 0); //RS=0: Instruction register
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, 0); //RW=0: Writing into the LCD

	//Sending the higher nibble to the LCD over the 4 data lines
	write_bits((cmd >> 4) & 0XF);

	//Sending the lower nibble to the LCD over the 4 data lines
	write_bits(cmd & 0XF);
}

/*Send the data (1 byte) to the LCD*/
void LCD_send_char(uint8_t data){

	/* NOTE
	 * RS=1 for LCD data
	 * RW=0 while writing the data
	 * */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, 1); //RS=1: Data register
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, 0); //RW=0: Writing into the LCD

	//Sending the higher nibble to the LCD over the 4 data lines
	write_bits((data >> 4) & 0XF);

	//Sending the lower nibble to the LCD over the 4 data lines
	write_bits(data & 0XF);
}

/*Send a string (multiple bytes) to the LCD*/
void LCD_send_string(char* message){

	while(*message != '\0'){
		LCD_send_char((uint8_t)*message);
		message++;
	}
}

/*Sets the position of the cursor to the given row and column co-ordinates
 * Row: 1-2
 * Column: 1-16
 * Every co-ordinate is associated to a DDRAM address that needs to be
 * accessed when setting a co-ordinate
 * */
void LCD_set_cursor(uint8_t row, uint8_t column){
	column--;
	switch(row){
	/*When the row 1 is selected*/
	case 1: LCD_send_command((column |= 0x80));
	break;
	/*When the row 2 is selected*/
	case 2: LCD_send_command((column |= 0xC0));
	break;
	default: break;
	}
}

/*Clear the data displayed on the LCD*/
void LCD_clear_display(void){
	LCD_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}

/*Set the cursor to the original position*/
void LCD_display_return_home(void){
	LCD_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}

/*Delay for x milliseconds*/
static void mdelay(uint8_t delayValue){
	for(uint32_t i=0; i<(delayValue*1000); i++);
}

/*Delay for x microseconds*/
static void udelay(uint8_t delayValue){
	for(uint32_t i=0; i<delayValue; i++);
}

/*Write 4 bits of data into the data lines D4, D5, D6 and D7*/
static void write_bits(uint8_t data){
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((data >> 0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((data >> 1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((data >> 2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((data >> 3) & 0x1));

	lcd_enable();
}

/*After writing data into the data lines, the LCD has to be enabled in order to load the instruction/data*/
static void lcd_enable(void){

	//High-to-low transition (pulse) for latching the instruction/data
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, 1);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, 0);
	udelay(100);
}
