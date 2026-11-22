/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * ina226.c
 *
 * Created on: Sep 29, 2023
 * Author: r9ofg.ru
 */

#include "main.h"
#include "ina226.h"

/* счетчик подряд идущих ошибок обмена по I2C (для контроля потери датчика) */
uint16_t ina226_error_cnt = 0;

/* последние достоверные значения */
static float lastVbus = 0.0f;
static float lastCurrent = 0.0f;

/* чтение 16-битного регистра INA226, возвращает 1 при успехе */
static uint8_t INA226_Read_Word(uint8_t reg, uint16_t *value)
{
	uint8_t buf[2];

	if(HAL_I2C_Master_Transmit(&hi2c1, INA226_I2C_ADDRESS_WRITE, &reg, 1, INA226_I2C_TIMEOUT) != HAL_OK)
	{
		return 0;
	}

	if(HAL_I2C_Master_Receive(&hi2c1, INA226_I2C_ADDRESS_READ, buf, 2, INA226_I2C_TIMEOUT) != HAL_OK)
	{
		return 0;
	}

	/* INA226 передает старший байт первым */
	*value = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];

	return 1;
}

void INA226_Init(uint16_t AVG, uint16_t Time_Vbus, uint16_t Time_Vshunt, uint16_t ModeOperation)
{
	uint8_t datos[3];
	uint16_t REGISTER;

	REGISTER = NO_RESET_INA226|(AVG << 9)|(Time_Vbus << 6)|(Time_Vshunt << 3)|ModeOperation;

	datos[0] = INA_CONFIGURATION_REGISTER;
	datos[1] = (uint8_t)(REGISTER >> 8);
	datos[2] = (uint8_t)(REGISTER & 0xFF);

	if(HAL_I2C_Master_Transmit(&hi2c1, INA226_I2C_ADDRESS_WRITE, datos, 3, INA226_I2C_TIMEOUT) != HAL_OK)
	{
		if(ina226_error_cnt < 0xFFFF) {ina226_error_cnt++;}
	}
	else
	{
		ina226_error_cnt = 0;
	}
}

float INA226_Vbus(void)
{
	uint16_t raw = 0;

	if(INA226_Read_Word(INA_BUS_VOLTAGE_REGISTER, &raw))
	{
		ina226_error_cnt = 0;
		lastVbus = (float)raw * (INA_BUS_VOLTAGE_LSB / 100000.0f); //вольты
	}
	else if(ina226_error_cnt < 0xFFFF)
	{
		ina226_error_cnt++;
	}

	return lastVbus;
}

float INA226_Current(void)
{
	uint16_t raw = 0;

	if(INA226_Read_Word(INA_SHUNT_VOLTAGE_REGISTER, &raw))
	{
		ina226_error_cnt = 0;

		/* при обрыве нагрузки регистр может быть равен 0xFFFF,
		 * отрицательные значения (старший бит) для усилителя мощности не используем
		 */
		if(raw >= 0x8000)
		{
			lastCurrent = 0.0f;
		}
		else
		{
			lastCurrent = ((float)raw * (INA_SHUNT_VOLTAGE_LSB / 10000000.0f)) / R_SHUNT; //амперы
		}
	}
	else if(ina226_error_cnt < 0xFFFF)
	{
		ina226_error_cnt++;
	}

	return lastCurrent;
}
