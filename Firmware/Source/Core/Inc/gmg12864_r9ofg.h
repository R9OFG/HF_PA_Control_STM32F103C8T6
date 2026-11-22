/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * gmg12864_r9ofg.h
 *
 */

#ifndef INC_GMG12864_R9OFG_H_
#define INC_GMG12864_R9OFG_H_

/* инклюды */

#include "main.h"

/* дефайны */

/*
 * для совместимости с контроллерами STM32F4хх и другими,
 * для выключения ножки, используется регистр BSRR,
 * для т.к. регистр BRR в них уже отсутствует,
 * собственно, ниже, так и сделано (c)Олег Волков Soldering_Iron
 */
#define CS_set() 	LCD_CS_GPIO_Port->BSRR 	= (uint32_t)LCD_CS_Pin<<16u;	//CS  - 0
#define CS_reset() 	LCD_CS_GPIO_Port->BSRR	= LCD_CS_Pin;					//CS  - 1
#define RST_set() 	LCD_RST_GPIO_Port->BSRR = (uint32_t)LCD_RST_Pin<<16u;	//RST - 0
#define RST_reset() LCD_RST_GPIO_Port->BSRR = LCD_RST_Pin;					//RST - 1
#define DC_set() 	LCD_DC_GPIO_Port->BSRR 	= (uint32_t)LCD_DC_Pin<<16u;	//DC  - 0
#define DC_reset() 	LCD_DC_GPIO_Port->BSRR 	= LCD_DC_Pin;					//DC  - 1

/* установка позиции по X */
#define LCD_Set_xPos(x) LCD_Send_Command((((x)&0xF0)>>4)|(0x10)); LCD_Send_Command((x)&0x0F)
/* установка позиции по Y */
#define LCD_Set_yPos(y) LCD_Send_Command(((y)&0x07)|(0xB0))

#define LCD_Width 	128		//ширина дисплея в пикселях
#define LCD_Height 	64		//высота дисплея в пикселях

#define pxView_On	1		//закраска пикселя On
#define pxView_Off	0		//закраска пикселя Off

#define ChInvers	1		//инверсия символа шрифта On
#define ChUnInvers	0		//инверсия символа шрифта Off

/* прототипы функций */

void LCD_Clean_Buffer_Frame(void);
void LCD_Update(void);
void LCD_Init(void);
void LCD_Draw_Pixel(uint8_t xPos, uint8_t yPos, uint8_t pxColor);
void LCD_Inversion_Pos_Buffer_Frame(uint16_t xPos_Start, uint16_t xPos_End);

void LCD_Draw_Char_1_Byte(uint8_t xPos, uint8_t yPos, char Symbol, uint8_t Inversion);
void LCD_Output_String(uint8_t xPos, uint8_t yPos, char *String, uint8_t setChSpacing, uint8_t Inversion);

void LCD_Draw_Line(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t xPos_End, uint8_t yPos_End, uint8_t pxColor);
void LCD_Draw_Rectangle(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t rectangle_Width, uint8_t rectangle_Height, uint8_t pxColor);
void LCD_Draw_Rectangle_Filled(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t rectangle_Width, uint8_t rectangle_Height, uint8_t pxColor);

#endif /* INC_GMG12864_R9OFG_H_ */
