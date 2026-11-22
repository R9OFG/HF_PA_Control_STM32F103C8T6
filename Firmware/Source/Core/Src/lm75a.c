/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * lm75ad.c
 *
 * Created on: Jan 26, 2024
 * Author: r9ofg.ru
 */

#include "main.h"
#include "lm75a.h"

uint16_t lm75a_error_cnt = 0;		//счетчик подряд идущих ошибок обмена
static int16_t lastTemperature = 0;	//последнее достоверное значение

int16_t LM75_temperature(void)
{
	_RAW Data;
	uint8_t LM75_Data[2];
	uint16_t Raw;
	int16_t temperature;

	if(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LM75_ADDR, (uint16_t)LM75_TEMP,
			I2C_MEMADD_SIZE_8BIT, LM75_Data, 2, 10) != HAL_OK)
	{
		//ошибка обмена - возвращаем последнее достоверное значение
		if(lm75a_error_cnt < 0xFFFF) {lm75a_error_cnt++;}
		return lastTemperature;
	}

	lm75a_error_cnt = 0;

	Data.Bytes[1] = LM75_Data[0];
	Data.Bytes[0] = LM75_Data[1];

	Raw = Data.Word >> 7;
	if (Raw & 0x0100)
	{
		//отрицательная температура (старший бит 9-битного значения)
		temperature = (-10 * (((~(uint8_t)(Raw & 0xFE) + 1) & 0x7F) >> 1) - (Raw & 0x01) * 5) / 10;
	}
	else
	{
		//положительная температура
		temperature = (((Raw & 0xFE) >> 1) * 10 + (Raw & 0x01) * 5) / 10;
	}

	lastTemperature = temperature;
	return temperature;
}
