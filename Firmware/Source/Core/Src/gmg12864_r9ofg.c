/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * gmg12864_r9ofg.c
 *
 * Autor: R9OFG.RU
 * Ver: 1.2
 * Data: 09-10-2023
 *
 * Библиотека для дисплея GMG12864-06D
 *
 * Для разработки библиотеки иcпользовались материалы:
 * https://www.youtube.com/watch?v=ajEqZN5s5xc
 * https://narodstream.ru/stm-urok-37-displej-tft-240x320-8bit-chast-1/
 * https://hubstub.ru/display/126-vyvod-simvolov-i-strok-na-tft-displey-na-primere-ili9341.html
 */

#include <string.h>

#include "gmg12864_r9ofg.h"
#include "font_tahoma_8_prop.h"

/* переменные */

uint8_t Buffer_Frame[1024] 	= {0,0};	//буфер кадра
uint8_t chSpacing			= 0;		//межсимвольный интервал в px
uint8_t lastPxString_X		= 0;		//последняя координата вывода строки по X

static uint8_t frame_dirty	= 1;		//признак изменения буфера кадра

/* Настройки */

extern SPI_HandleTypeDef hspi1;			//используемая шина SPI

/* основные функции */

/*
 * функция отправки команды на дисплей
 * параметр sendCommand - 8 бит данных
 */
static void LCD_Send_Command(uint8_t sendCommand)
{
	DC_set();
	HAL_SPI_Transmit(&hspi1, &sendCommand, 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {};
	DC_reset();
}

/*
 * Функция отправки данных на дисплей
 * параметр sendData - 8 бит данных
 */
static void LCD_Send_Data(uint8_t sendData)
{
	HAL_SPI_Transmit(&hspi1, &sendData, 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {};
}

/* функция очистки буфера кадра */
void LCD_Clean_Buffer_Frame(void)
{
	memset(Buffer_Frame, 0x00, sizeof(Buffer_Frame));
	frame_dirty = 1;
}

/* функция вывода буфера кадра на дисплей (обновление дисплея) */
void LCD_Update(void)
{
	//если в буфере кадра ничего не изменилось, обновлять дисплей не нужно
	if(frame_dirty == 0)
	{
		return;
	}
	frame_dirty = 0;

	CS_set();
	for (uint8_t y=0;y<8;y++)
	{
		LCD_Set_xPos(0);
		LCD_Set_yPos(y);
		for (uint8_t x=0;x<LCD_Width;x++)
		{
			LCD_Send_Data(Buffer_Frame[x + LCD_Width * y]);
		}
	}
	CS_reset();
}

/* функция инициализации дисплея */
void LCD_Init(void)
{
	CS_set();
	RST_set();
	HAL_Delay(10);
	RST_reset();
	HAL_Delay(10);
	//установка рабочего цикла ( 1/7 или 1/9 ) в зависимости от физического ЖК-дисплея
	LCD_Send_Command(0xA2);
	HAL_Delay(1);
	//установка горизонтальную и вертикальную ориентацию в известное состояние
	LCD_Send_Command(0xA0); //ADC selection(SEG0->SEG128)
	LCD_Send_Command(0xC8); //SHL selection(COM0->COM64)
	HAL_Delay(1);
	//делитель внутреннего резистора установлен на 7 (от 0..7)
	LCD_Send_Command(0x20 | 0x7);
	HAL_Delay(1);
	//управление питанием, все внутренние блоки включены (от 0..7)
	LCD_Send_Command(0x28 | 0x7);
	HAL_Delay(1);
	//режим динамического контраста
	LCD_Send_Command(0x81);
	LCD_Send_Command(15); //настройка контраста. 15-19 норм, максимум 63
	HAL_Delay(1);
	LCD_Send_Command(0x40);
	HAL_Delay(1);
	//включение дисплея
	LCD_Send_Command(0xAF);
	//инвертирование экрана (0xA6 - nomal, 0xA7 - revers)
	LCD_Send_Command(0xA6);
	HAL_Delay(1);
	CS_reset();

	//очищаем буфер кадра
	LCD_Clean_Buffer_Frame();
	//обновляем дисплей
	LCD_Update();
}

/* функция прорисовки пикселя */
void LCD_Draw_Pixel(uint8_t xPos, uint8_t yPos, uint8_t pxColor)
{
	if ((xPos >= LCD_Width)||(yPos >= LCD_Height))
	{
		//если значения по x и y вышли за пределы экрана, то выходим из функции
		return;
	}
	else
	{
		uint16_t arrayPos = xPos + ((yPos/8)*LCD_Width);

		//заполняем буфер кадра
		if (pxColor)
		{
			Buffer_Frame[arrayPos] |= 1 << (yPos % 8);
		}
		else
		{
			Buffer_Frame[arrayPos] &= 0xFF ^ 1 << (yPos % 8);
		}

		//буфер кадра изменился
		frame_dirty = 1;
	}
}

/* функция инверсии любой точки в буфере кадра */
void LCD_Inversion_Pos_Buffer_Frame(uint16_t xPos_Start, uint16_t xPos_End)
{
	for (; xPos_Start < xPos_End; xPos_Start++)
	{
		Buffer_Frame[xPos_Start] = ~(Buffer_Frame[xPos_Start]);
	}
	frame_dirty = 1;
}

/* работа со шрифтами */

/* функция прорисовки однобайтового символа шрифта */
void LCD_Draw_Char_1_Byte(uint8_t xPos, uint8_t yPos, char Symbol, uint8_t Inversion)
{
	/*
	 * выгрузка кода шрифта в "h" файл из программы matrixFont
	 * https://habr.com/ru/articles/575332/
	 * параметры выгрузки:
	 * сначала строки, справа на лево, сверху вниз, пропорциональный, HEX, нули, 8(uint8_t, unsigned char), С99
	 * ниже циклы для однобайтового символа шрифта, т.е. ширина символа не более 8 пикселей
	 */

	/* если ASCII номер символа в диапазоне 32...255, то рисуем символы шрифта */
	if ((Symbol >= 32) && (Symbol <= 255))
	{
		uint8_t pxChView;
		uint8_t pxBkView;

		//включение/выключении инверсии символа
		if (!(Inversion & ChInvers))
		{
			pxChView = pxView_On;
			pxBkView = pxView_Off;
		}
		else
		{
			pxChView = pxView_Off;
			pxBkView = pxView_On;
		}

		//извлекаем ширину символа + 1px для минимального межсимвольного интервала
		uint8_t chWidth = font_tahoma_8[(Symbol - 0x20) * (1 + FONT_TAHOMA_8_CHAR_HEIGHT)] + 1;
		//запоминаем ширину данного символа
		chSpacing = chWidth;

		//рисуем заданный символ шрифта
		for (uint8_t y = 1; y <= FONT_TAHOMA_8_CHAR_HEIGHT; y++) //высота холста шрифта в пикселях
		{
			for (uint8_t x = 0; x < chWidth; x++) //ширина холста символа шрифта
			{
				/*
				 * извлекаем из массива шрифта байт строки пикселей символа
				 * и с помощью операции побитового И определяем где 1 и где 0
				 * 0x20 на 32
				 */

				if (font_tahoma_8[((Symbol - 0x20) * (1 + FONT_TAHOMA_8_CHAR_HEIGHT)) + y] >> (7 - x) & 0x01)
				{
					LCD_Draw_Pixel(xPos + x, yPos + y, pxChView);
				}
				else
				{
					LCD_Draw_Pixel(xPos + x, yPos + y, pxBkView);
				}
			}

			/*
			 * заполняем фоном справа от символа если ширина символа меньше ширины холста шрифта
			 * с учетом заданного минимального межсимвольного интервала в 1px
			 */
			if (chWidth < FONT_TAHOMA_8_CHAR_WIDTH)
			{
				for (uint8_t n = chWidth; n < (FONT_TAHOMA_8_CHAR_WIDTH + 1); n++)
				{
					if (!(Inversion & ChInvers))
					{
						LCD_Draw_Pixel(xPos + n, yPos + y, pxBkView);
					}
					else
					{
						LCD_Draw_Pixel(xPos + n, yPos + y, pxChView);
					}
				}
			}
		}
	}
}

/*
 * Преобразование одного символа UTF-8 в байт кодовой страницы CP1251.
 *
 * Исходники хранятся в UTF-8, а шрифт дисплея GMG12864 построен по таблице
 * 1251 (1251 ANSI - кириллица). Конвертируем на лету при выводе строки,
 * чтобы не зависеть от iconv в тулчейне (-fexec-charset).
 *
 * Возвращает количество байт, которое символ занимает в исходной строке.
 */
static uint8_t utf8_to_cp1251(const char *string, uint8_t *code)
{
	uint32_t codepoint = 0;
	uint8_t  length    = 1;
	uint8_t  first     = (uint8_t)string[0];

	//обычный ASCII
	if(first < 0x80)
	{
		*code = first;
		return 1;
	}

	if((first & 0xE0) == 0xC0)		{ codepoint = first & 0x1F; length = 2; }
	else if((first & 0xF0) == 0xE0)	{ codepoint = first & 0x0F; length = 3; }
	else if((first & 0xF8) == 0xF0)	{ codepoint = first & 0x07; length = 4; }
	else
	{
		*code = '?';
		return 1;
	}

	for(uint8_t i = 1; i < length; i++)
	{
		if(((uint8_t)string[i] & 0xC0) != 0x80)
		{
			*code = '?';
			return 1;
		}
		codepoint = (codepoint << 6) | ((uint8_t)string[i] & 0x3F);
	}

	if((codepoint >= 0x0410) && (codepoint <= 0x044F))
	{
		//А-я
		*code = (uint8_t)(codepoint - 0x0410 + 0xC0);
	}
	else if(codepoint == 0x0401)	{ *code = 0xA8; }	//Ё
	else if(codepoint == 0x0451)	{ *code = 0xB8; }	//ё
	else if(codepoint == 0x00B0)	{ *code = 0xB0; }	//градус
	else if(codepoint == 0x2116)	{ *code = 0xB9; }	//№
	else							{ *code = '?'; }

	return length;
}

/* функция вывода строки на дисплей */
void LCD_Output_String(uint8_t xPos, uint8_t yPos, char *String, uint8_t setChSpacing, uint8_t Inversion)
{
	/*
	 * вывод строки шрифтом font_tahoma_8
	 * параметры функции:
	 * xPos			- позиция по X
	 * yPos			- позиция по Y
	 * *String		- строка для вывода (UTF-8, конвертируется в CP1251)
	 * setChSpacing	- межсимвольный интервал в px
	 * Inversion	- включение/выключени инверсии
	 */

	//посимвольный вывод строки
	while((*String != '\n')&&(*String != '\0'))
	{
		uint8_t code = '?';

		//преобразуем символ UTF-8 в код символа шрифта (CP1251)
		String += utf8_to_cp1251(String, &code);

		//вывод текущего символа строки
		LCD_Draw_Char_1_Byte(xPos, yPos, (char)code, Inversion);

		//изменяем координату для отрисовки следующего символа
		xPos += chSpacing - 1 + setChSpacing;
	}

	//запоминаем последнюю координату по X
	lastPxString_X = xPos;
}

/* ширина строки в пикселях для текущего шрифта */
static uint16_t LCD_String_Width(const char *String, uint8_t setChSpacing)
{
	uint16_t width = 0;

	while((*String != '\n') && (*String != '\0'))
	{
		uint8_t code = '?';

		String += utf8_to_cp1251(String, &code);

		if((code >= 32) && (code <= 255))
		{
			width += font_tahoma_8[(code - 0x20) * (1 + FONT_TAHOMA_8_CHAR_HEIGHT)];
			width += setChSpacing;
		}
	}

	return width;
}

/* функция вывода строки по центру дисплея */
void LCD_Output_String_Center(uint8_t yPos, char *String, uint8_t setChSpacing, uint8_t Inversion)
{
	uint16_t width = LCD_String_Width(String, setChSpacing);
	uint8_t xPos = 0;

	if(width < LCD_Width)
	{
		xPos = (uint8_t)((LCD_Width - width) / 2);
	}

	LCD_Output_String(xPos, yPos, String, setChSpacing, Inversion);
}

/* геометрические примитивы */

/* функция рисования линии */
void LCD_Draw_Line(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t xPos_End, uint8_t yPos_End, uint8_t pxColor)
{
	int dx = (xPos_End >= xPos_Start) ? xPos_End - xPos_Start : xPos_Start - xPos_End;
	int dy = (yPos_End >= yPos_Start) ? yPos_End - yPos_Start : yPos_Start - yPos_End;
	int sx = (xPos_Start < xPos_End) ? 1 : -1;
	int sy = (yPos_Start < yPos_End) ? 1 : -1;
	int err = dx - dy;

	for (;;)
	{
		LCD_Draw_Pixel(xPos_Start, yPos_Start, pxColor);
		if (xPos_Start == xPos_End && yPos_Start == yPos_End)
			break;
		int e2 = err + err;
		if (e2 > -dy)
		{
			err -= dy;
			xPos_Start += sx;
		}
		if (e2 < dx)
		{
			err += dx;
			yPos_Start += sy;
		}
	}
}

/* функция рисования пустотелого прямоугольника */
void LCD_Draw_Rectangle(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t rectangle_Width, uint8_t rectangle_Height, uint8_t pxColor)
{
	/* рисуем стороны прямоугольника */
	//левая сторона прямоугольника
	LCD_Draw_Line(xPos_Start, yPos_Start, xPos_Start, yPos_Start + rectangle_Height, pxColor);
	//верх прямоугольника
	LCD_Draw_Line(xPos_Start, yPos_Start, xPos_Start + rectangle_Width, yPos_Start, pxColor);
	//правая сторона прямоугольника
	LCD_Draw_Line(xPos_Start + rectangle_Width, yPos_Start, xPos_Start + rectangle_Width, yPos_Start + rectangle_Height, pxColor);
	//низ прямоугольника
	LCD_Draw_Line(xPos_Start, yPos_Start + rectangle_Height, xPos_Start + rectangle_Width, yPos_Start + rectangle_Height, pxColor);
}

/* функция рисования закрашенного прямоугольника */
void LCD_Draw_Rectangle_Filled(uint8_t xPos_Start, uint8_t yPos_Start, uint8_t rectangle_Width, uint8_t rectangle_Height, uint8_t pxColor)
{
	for (uint8_t i = 0; i <= rectangle_Height; i++)
	{
		LCD_Draw_Line(xPos_Start, yPos_Start + i, xPos_Start + rectangle_Width, yPos_Start + i, pxColor);
	}
}
