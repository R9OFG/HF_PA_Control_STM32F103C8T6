/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * lm75a.h
 *
 *  Created on: Jan 26, 2024
 *  Author: r9ofg.ru
 */

#ifndef INC_LM75A_H_
#define INC_LM75A_H_

extern I2C_HandleTypeDef hi2c1;

#define LM75_ADDR  0x90	// 7-битный адрес LM75 на шине i2c, левое выравнивание MSbit
#define LM75_TEMP  0x00	// регистр измеренного значения температуры

extern uint16_t lm75a_error_cnt;	//счетчик подряд идущих ошибок обмена с LM75A

typedef union
{
	uint8_t Bytes[2];
	int16_t Word;
} _RAW;

int16_t LM75_temperature(void);

#endif /* INC_LM75A_H_ */
