/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * _prog_process.c
 *
 *  Created on: Sep 29, 2023
 *  Author: r9ofg.ru
 */

/* инклюды */

#include <stdio.h>

#include "_program_mode.h"
#include "gmg12864_r9ofg.h"
#include "Float_transform.h"
#include "ina226.h"
#include "lm75a.h"
#include "eeprom.h"
#include "i2c_er.h"
#include "pa_protocol.h"

/*
 * Строки для дисплея хранятся в UTF-8 прямо в исходниках.
 * Шрифт дисплея построен по таблице CP1251, поэтому LCD_Output_String()
 * преобразует UTF-8 в CP1251 на лету (см. gmg12864_r9ofg.c).
 */

/* переменные и массивы*/

BUTTON_PRESS lastButton				= NO_BUT;			//предыдущая нажатая кнопка

PROG_STATE 	 PROG_state				= PROG_STATE_WORK;	//флаг режима работы программы
volatile uint8_t force_work		= 0;				//принудительный возврат в рабочий режим по команде WORK
BUTTON_PRESS BUT_press				= NO_BUT;			//флаг нажатой кнопки

BAND_INDEX LPF_active				= _10;				//актитвный ФНЧ по умолчанию (в соответствии с массивом LPF_band_string)
static BAND_INDEX old_band			= (BAND_INDEX)0xFF;	//последний применённый диапазон (0xFF — ещё не применялся)

char LPF_band_string[9][12]			= {{"160м\n"},{"80м\n"},{"40м\n"},{"30м\n"},{"20м\n"},{"17м\n"},{"15м\n"},{"12м\n"},{"10м\n"}};	//строковые метки диапазонов ФНЧ
char SETUP_1_index[4][36]			= {{"Подсветка\n"},{"Изм. мощность\n"},{"Упр. ФНЧ\n"}, {"Вкл. охлаждение\n"}};					//пункты парметров первой страницы настроек
char SETUP_1_param_index[5][16]		= {{"вкл\n"},{"выкл\n"},{"ручн\n"},{"ABCD\n"},{"авто\n"}}; 										//значения параметров первой страницы настроек пунктов меню настроек RUS
char SETUP_2_index[4][38]			= {{"Напряжение мин.\n"},{"Напряжение макс.\n"},{"Ток макс.\n"},{"КСВ макс.\n"}};				//пункты парметров второй страницы настроек
char SETUP_3_index[4][28]			= {{"АТТ вх. 160м dB\n"},{"АТТ вх.  80м dB\n"},{"АТТ вх.  40м dB\n"},{"АТТ вх.  30м dB\n"}};	//пункты парметров третьей страницы настроек
char SETUP_4_index[4][28]			= {{"АТТ вх.  20м dB\n"},{"АТТ вх.  17м dB\n"},{"АТТ вх.  15м dB\n"},{"АТТ вх.  12м dB\n"}};	//пункты парметров четвертой страницы настроек
char SETUP_5_index[2][44]			= {{"АТТ вх.  10м dB\n"},{"Сброс настроек\n"}};													//пункты парметров пятой страницы настроек,

char strValueU[16]					= "";				//строка текущего напряжения
char strValueCurrent[13]			= "";				//строка текущего тока потребления
char strValuePwr[7]					= "";				//строка текущей выходной мощности
char strValuePwrOut[7]				= "";				//строка выходной мощности для дисплея
char strValueSwr[16]				= "";				//строка текущего КСВ
char strValueT[13]					= "";				//строка текущей температуры
char strValue[26]					= "";				//строка для перемещений по меню настроек

volatile uint8_t sel_param 			= 0;				//активный пункт меню настроек
volatile uint8_t sel_work_param 	= 0;				//активный пункт параметра в рабочем режиме
volatile uint8_t flag_ptt_on		= 0;				//флаг внешнего сигнала PTT
volatile uint8_t fan_on				= 0;				//флаг ключа вентилятора
volatile uint8_t fan_test_override	= 0;				//принудительное включение вентилятора (проверка ключа)
volatile uint8_t led_auto_off		= 0;				//флаг автоматического отключения подсветки дисплея
volatile uint8_t led_disp_on			= 0;				//фактическое состояние подсветки: 1 - включена
static uint32_t led_auto_start		= 0;				//метка времени последней активности для авто отключения подсветки

volatile uint8_t att_data 			= 0;				//управляющие биты аттенюатора PE4302
volatile uint8_t band_att_data[9] 	= {0,};				//затухнаие аттенюатора PE4302 по диапазонам
volatile uint8_t bnd_in_att_data[9] = {0,};				//настройки входного затухнаия аттенюатора PE4302 по диапазонам

float rawFor 						= 0;				//напряжение с датчика КСВ FOR
float rawRef						= 0;				//напряжение с датчика КСВ REF
float powerFor						= 0;				//значение падающей мощности
float powerRef						= 0;				//значение отраженной мощности
float currentPwr					= 0;				//мгновенное значение мощности в нагрузке с учетом КСВ
float powerDisp						= 0;				//сглаженное значение мощности для индикации
float swrDisp						= 1.0f;				//сглаженное значение КСВ для индикации
float voltDisp						= 0;				//сглаженное значение напряжения для индикации
float currDisp						= 0;				//сглаженное значение тока для индикации

/* дополнительно сглаженные значения только для дисплея (телеметрия в приложение идёт из *Disp) */
static float powerLcd				= 0;				//мощность для дисплея и "градусника"
static float voltLcd				= 0;				//напряжение для дисплея
static float currLcd				= 0;				//ток для дисплея
static float powerShown				= -1.0f;			//последнее показанное значение мощности
static float voltShown				= -1.0f;			//последнее показанное напряжение
static float currShown				= -1.0f;			//последний показанный ток

uint8_t flag_ina226					= 0;				//флаг датчика INA226
uint8_t flag_lm75a					= 0;				//флаг датчика LM75A

/* состояние системы защиты */
PA_FAULT pa_fault					= PA_FAULT_NONE;	//текущая авария

static uint16_t u_fault_cnt			= 0;				//счетчик подтверждения аварии по напряжению
static uint16_t i_fault_cnt			= 0;				//счетчик подтверждения аварии по току
static uint16_t t_fault_cnt			= 0;				//счетчик подтверждения перегрева
static uint16_t sensor_fault_cnt	= 0;				//счетчик подтверждения потери датчиков
static uint32_t swr_bad_tick		= 0;				//метка начала превышения КСВ (0 - превышения нет)

static uint32_t sensor_poll_tick	= 0;				//метка времени последнего опроса датчиков
static uint32_t measure_poll_tick	= 0;				//метка времени последнего обновления показаний

/* структуры */

struct SETUP_PA Settings_PA;							//структура настроек контроллера УМ

/* прототипы внешних переменных */

extern IWDG_HandleTypeDef hiwdg;

extern uint8_t  lastPxString_X;
extern uint8_t  end_adc_flag;
extern uint8_t  end_adc_filter_flag;
extern volatile uint8_t flag_ptt_change;
extern volatile uint16_t adc_values[ADC_CHANNELS_NUM];
extern uint16_t adc_out[ADC_CHANNELS_NUM-1];

extern uint8_t 	sign_number;
extern int 		integer_number;
extern uint32_t fractional_number;

/*-----------------------------------------------------------------*/

/* функция определения какая кнопка нажата */
BUTTON_PRESS GET_button(uint16_t Value)
{
	if ((Value > 2700) && (Value < 2800)) 		//кнопка ">"
	{
		return  BUT_LEFT;
	}
	else if ((Value > 2000) && (Value < 2100)) 	//кнопка "<"
	{
		return BUT_RIGHT;
	}
	else if (Value < 50) 						//кнопка "F"
	{
		return BUT_FUNCTION;
	}
	else
	{
		return NO_BUT;
	}
}

/* функция обработки кнопок */
void READ_button()
{
	{
		/* обработка кнопок */

		volatile static uint8_t buttonClk = 0; //счетчик для антидребезга

		BUTTON_PRESS tempButton = GET_button(adc_values[0]);

		if (tempButton != lastButton)
		{
			buttonClk = 0;
		}
		else
		{
			//борьба с дребезгом и повторным срабатыванием пока кнопка нажата
			if (buttonClk < 255) {buttonClk++;}
			if (buttonClk == 2)
			{
				//если подсветка в AUTO и погашена, первое нажатие только включает её
				if((Settings_PA.SET_LED_DISPLAY == AUTO) && (led_disp_on == 0))
				{
					LED_auto_on();
					BUT_press = NO_BUT;
				}
				else
				{
					BUT_press = tempButton;
				}
			}
		}

		//запоминаем последнюю нажатую кнопку
		lastButton = tempButton;

		HAL_Delay(10);
	}
}

/*-----------------------------------------------------------------*/

/* функция опроса датчиков INA226 и LM75A на шине I2C */
void SEARCH_sensor_on_i2c_bus()
{
	HAL_StatusTypeDef ret;
	uint8_t bt = 0;

	/*
	 * проверка шины I2C по датчику INA226
	 * попытка борьбы с зависанием шины I2C от ВЧ наводок
	 * используется - #include "i2c_er.h"
	 */
	uint32_t status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)INA226_I2C_ADDRESS_WRITE, &bt, 1, 50);
	if(status != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 50);
	}
	/* конец проверки шины I2C */

	//опрос датчика INA226
	ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(INA226_I2C_ADDRESS_READ), 3, 50);
	if(ret == HAL_OK)
	{
		//поднимайем флаг наличия датчика на шине I2C
		flag_ina226 = 1;
	}
	else if(ret != HAL_OK)
	{
		//опускаем флаг наличия датчика на шине I2C
		flag_ina226 = 0;
	}

	//опрос датчика LM75A
	ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(LM75_ADDR), 3, 50);
	if(ret == HAL_OK)
	{
		//поднимайем флаг наличия датчика на шине I2C
		flag_lm75a = 1;
	}
	else if(ret != HAL_OK)
	{
		//опускаем флаг наличия датчика на шине I2C
		flag_lm75a = 0;
	}
}

/*-----------------------------------------------------------------*/

/* функция включения/выключения подсветки дисплея */
void LED_display(SETUP_PA_PARAM Value)
{
	switch (Value)
	{
		//выключаем подсветку дисплея и опускаем флаг автоматического отключения подсветки
		case OFF: {LED_DISP_reset(); led_auto_off = 0; led_disp_on = 0;}
		break;
		//включаем подсветку дисплея и опускаем флаг автоматического отключения подсветки
		case ON: {LED_DISP_set(); led_auto_off = 0; led_disp_on = 1;}
		break;
		//включаем подсветку дисплея и поднимаем флаг автоматического отключения подсветки
		case AUTO: {LED_DISP_set(); led_auto_off = 1; led_auto_start = HAL_GetTick(); led_disp_on = 1;}
		break;
	}
}

/* функия авто включения подсветки дисплея */
void LED_auto_on()
{
	//если выбрано автоматическое управление подсветкой дисплея
	if(Settings_PA.SET_LED_DISPLAY == AUTO)
	{
		//включаем подсветку
		LED_DISP_set();
		led_disp_on = 1;
		//запустить отсчет времени бездействия заново
		led_auto_start = HAL_GetTick();
	}
}

/*-----------------------------------------------------------------*/

/* функция рассчета падающей/отраженной мощностей */
void GET_out_power()
{
	const float Kpwr = 500.0f;	//коэффициент для коррекции рассчетной мощности

	/*
	 * Тандем-матч (TandemMatch TM1).
	 * На этой ревизии платы выходы моста приходят на МКУ так:
	 * PA2/ADC1_IN2 - отражённая волна (REF),
	 * PA3/ADC1_IN3 - падающая волна (FOR).
	 * Поэтому adc_out[0] (PA2) - reflected, adc_out[1] (PA3) - forward.
	 */
	rawFor = adc_out[1] * 3.2f / 4095.0f;
	rawRef = adc_out[0] * 3.3f / 4095.0f;

	//рассчитываем падающую мощность FORWARD
	powerFor = rawFor * rawFor * Kpwr / (float)LOAD_Z;

	//рассчитываем отраженную мощность REFLECTED
	powerRef = rawRef * rawRef * Kpwr / (float)LOAD_Z;

	//рассчитываем мощность в нагрузке
	currentPwr = powerFor - powerRef;
	if(currentPwr < 0.1f)
	{
		currentPwr = 0.0f;
	}

	/*
	 * Плавное сглаживание показаний мощности:
	 * быстрый подъем и медленный спад, чтобы цифры не "мельтешили".
	 * Функция вызывается с периодом готовности данных АЦП (25 мс).
	 */
	if(currentPwr > powerDisp)
	{
		powerDisp += (currentPwr - powerDisp) * PWR_FILTER_ATTACK;
	}
	else
	{
		powerDisp += (currentPwr - powerDisp) * PWR_FILTER_DECAY;
	}

	//мелкий шум около нуля гасим полностью
	if(powerDisp < 0.05f)
	{
		powerDisp = 0.0f;
		powerLcd = 0.0f;
	}
	else if(powerLcd < 0.1f)
	{
		//первое значение после начала передачи показываем сразу,
		//дальше ведём плавно, чтобы цифры и "градусник" не дёргались
		powerLcd = powerDisp;
	}
	else
	{
		powerLcd += (powerDisp - powerLcd) * PWR_LCD_FILTER_KOEF;
	}
}

/* функция получения и контроля текущего тока потребления */
void GET_current()
{
	float valueI 	 = 0;

	if(flag_ina226 == 1)
	{
		valueI = INA226_Current();

		//плавное сглаживание показаний тока (защита использует мгновенное значение)
		if(currDisp < 0.01f)
		{
			currDisp = valueI;
		}
		else
		{
			currDisp += (valueI - currDisp) * UI_FILTER_KOEF;
		}

		if(currLcd < 0.01f)
		{
			currLcd = currDisp;
		}
		else
		{
			currLcd += (currDisp - currLcd) * UI_LCD_FILTER_KOEF;
		}

		//показываем только заметные изменения, чтобы последняя цифра не мельтешила
		float currDelta = currLcd - currShown;

		if((currShown < 0.0f)||(currDelta >= UI_LCD_DEADBAND)||(currDelta <= -UI_LCD_DEADBAND))
		{
			currShown = currLcd;

			//для уменьшения размера во флеш используем библиотеку Float_transform
			Float_transform(currLcd, 2, &sign_number, &integer_number, &fractional_number);
			if (!sign_number)
			{
				snprintf(strValueCurrent, sizeof(strValueCurrent), "Iа: %d.%.02ld\n", integer_number, fractional_number);
			}
		}

		//контроль тока потребления с подтверждением
		if(valueI >= (float)Settings_PA.SET_MAX_I)
		{
			if(i_fault_cnt < 0xFFFF) {i_fault_cnt++;}
			if(i_fault_cnt >= I_FAULT_CNT)
			{
				PROTECTION_fault(PA_FAULT_CURRENT);
			}
		}
		else
		{
			i_fault_cnt = 0;
		}
	}
	else
	{
		//датчик INA226 не найден
		snprintf(strValueCurrent, sizeof(strValueCurrent), "Iа: ???\n");
		currShown = -1.0f;
	}
}

/*-----------------------------------------------------------------*/

/* функция вывода "градусника" мощности */
void POWER_meter()
{
	//длина "градусника" в пикселях (шкала 0...100 Вт),
	//шаг 1 пиксель — мельче ступеньки, плавнее индикация
	const uint8_t meterMax = (uint8_t)(LCD_Width - 37 - 1);
	float meter = powerShown * (float)meterMax / 100.0f;
	uint8_t valueMeter = 0;

	if(meter >= (float)meterMax)
	{
		valueMeter = meterMax;
	}
	else
	{
		valueMeter = (uint8_t)meter;
	}

	//закрашиваем предыдущий "градусник" мощности
	LCD_Draw_Rectangle_Filled(35, 50, LCD_Width - 1, 14, pxView_Off);

	//выводим значение "градусника" одной заливкой
	//(раньше рисовался в цикле, то есть квадратично по числу строк - очень медленно)
	LCD_Draw_Rectangle_Filled(37, 51, valueMeter, 9, pxView_On);

	//закрашиваем последний максималный сегмент "градусника" мощности
	LCD_Draw_Rectangle_Filled(LCD_Width - 3, 50, LCD_Width - 1, 14, pxView_Off);
}

/* функция расчета и вывода на дисплей КСВ */
void SWR_measurement()
{
	float valueSwr		 = 0;
	float denom			 = 0;

	if((flag_ptt_on != 1)||(powerFor < SWR_MIN_POWER))
	{
		//на приеме или без возбуждения КСВ не измеряется
		snprintf(strValueSwr, sizeof(strValueSwr), "---\n");
		swrDisp = 1.0f;
		swr_bad_tick = 0;
	}
	else
	{
		//режим передачи

		//разность напряжений падающей и отраженной волны
		denom = rawFor - rawRef;

		//защита от деления на ноль и от мусора при недостаточном уровне
		if(denom < 0.01f)
		{
			valueSwr = 99.9f;
		}
		else
		{
			valueSwr = (rawFor + rawRef) / denom;
		}

		if(valueSwr < 1.0f) {valueSwr = 1.0f;}
		if(valueSwr > 99.9f) {valueSwr = 99.9f;}

		//плавное сглаживание показаний КСВ
		swrDisp += (valueSwr - swrDisp) * SWR_FILTER_KOEF;

		//контроль КСВ с выдержкой на работу автотюнера
		if((valueSwr >= (float)Settings_PA.SET_MAX_SWR)&&(Settings_PA.SET_MAX_SWR != MAX_SWR))
		{
			//если порог КСВ установлен 9, контроль КСВ не осуществляется
			if(swr_bad_tick == 0)
			{
				swr_bad_tick = HAL_GetTick();
				if(swr_bad_tick == 0) {swr_bad_tick = 1;}	//HAL_GetTick() может быть равен 0
			}

			if((HAL_GetTick() - swr_bad_tick) >= SWR_GRACE_MS)
			{
				//выдержка вышла - авария
				swr_bad_tick = 0;
				PROTECTION_fault(PA_FAULT_SWR);
			}
		}
		else
		{
			//КСВ вернулся в норму - сбрасываем выдержку
			swr_bad_tick = 0;
		}

		//выводим сглаженное значение
		Float_transform(swrDisp, 1, &sign_number, &integer_number, &fractional_number);
		if (!sign_number)
		{
			snprintf(strValueSwr, sizeof(strValueSwr), "%d.%.01ld\n", integer_number, fractional_number);
		}
		else
		{
			snprintf(strValueSwr, sizeof(strValueSwr), "---\n");
		}
	}

	//выводим значение КСВ
	LCD_Output_String(90, 13, strValueSwr, 0, ChUnInvers);

	//закрашиваем после значения
	LCD_Draw_Rectangle_Filled(lastPxString_X, 13, LCD_Width - 1, 10, pxView_Off);
}

/* функция вывода на дисплей мощности */
void POWER_measurement()
{
	//выводим значение затухания аттенюатора если разрешено изменение мощности
	if (Settings_PA.SET_CHANGE_POWER == ON)
	{

		//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
		Float_transform((float)band_att_data[LPF_active] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
		if (!sign_number)
		{
			snprintf(strValuePwr, sizeof(strValuePwr), "-%d.%.01ld\n", integer_number, fractional_number);
		}

		//вертикальная разделительная линия
		LCD_Draw_Line(56, 0, 56, 11, pxView_On);

		//выводим значение затухания
		LCD_Output_String(58, 1, strValuePwr, 0, ChUnInvers);

		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 1, 85, 10, pxView_Off);

		//вертикальная разделительная линия
		LCD_Draw_Line(86, 0, 86, 11, pxView_On);
	}

	if(flag_ptt_on != 1)
	{
		//режим приема

		//обновляем текущие значения датчика КСВ (для диагностики),
		//пиковые значения в приложении запоминаются только на передаче
		GET_out_power();

		//сбрасываем сглаженное значение мощности
		powerDisp = 0.0f;
		powerLcd = 0.0f;

		//строка для вывода мощности = ---
		snprintf(strValuePwrOut, sizeof(strValuePwrOut), "---\n");
		powerShown = -1.0f;

		//закрашиваем "градусник" мощности
		LCD_Draw_Rectangle_Filled(35, 50, LCD_Width - 1, 14, pxView_Off);
	}
	else
	{
		//режим передачи

		//считываем данные с датчика КСВ и сглаживаем показания
		GET_out_power();

		//если мощность больше минимальной, рассчитываем и выводим ее на дисплей
		if(powerLcd >= 0.1f)
		{
			//обновляем цифры и "градусник" только при заметном изменении
			float powerDelta = powerLcd - powerShown;

			if((powerShown < 0.0f)||(powerDelta >= PWR_LCD_DEADBAND)||(powerDelta <= -PWR_LCD_DEADBAND))
			{
				powerShown = powerLcd;

				//для уменьшения размера во флеш используем библиотеку Float_transform
				Float_transform(powerShown, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValuePwrOut, sizeof(strValuePwrOut), "%d.%.01ld\n", integer_number, fractional_number);
				}
			}

			//выводим "градусник" мощности
			POWER_meter();
		}
		else
		{
			//выводим мощность = ---
			snprintf(strValuePwrOut, sizeof(strValuePwrOut), "---\n");
			powerShown = -1.0f;

			//гасим "градусник", чтобы не остался от предыдущего значения
			LCD_Draw_Rectangle_Filled(35, 50, LCD_Width - 1, 14, pxView_Off);
		}
	}

	//если в настройках включено изменение параметра то выделяем если изменяемый параметр активен
	if ((Settings_PA.SET_CHANGE_POWER == ON)&&(flag_ptt_on != 0))
	{
		//закрашиваем поле
		LCD_Draw_Rectangle_Filled(87, 1, LCD_Width - 1, 10, pxView_On);

		//выводим значение мощности
		LCD_Output_String(90, 1, strValuePwrOut, 0, ChInvers);

		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 1, LCD_Width - 1, 10, pxView_On);
	}
	else
	{
		//закрашиваем поле
		LCD_Draw_Rectangle_Filled(87, 1, LCD_Width - 1, 10, pxView_Off);

		//выводим значение
		LCD_Output_String(90, 1, strValuePwrOut, 0, ChUnInvers);

		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 1, LCD_Width - 1, 10, pxView_Off);
	}
}

/* функция измерения напряжения питания усилителя */
void U_measurement()
{
	float valueU 		= 0;

	if(flag_ina226 == 1)
	{
		valueU = INA226_Vbus();

		//плавное сглаживание показаний напряжения (защита использует мгновенное значение)
		if(voltDisp < 0.01f)
		{
			voltDisp = valueU;
		}
		else
		{
			voltDisp += (valueU - voltDisp) * UI_FILTER_KOEF;
		}

		if(voltLcd < 0.01f)
		{
			voltLcd = voltDisp;
		}
		else
		{
			voltLcd += (voltDisp - voltLcd) * UI_LCD_FILTER_KOEF;
		}

		//показываем только заметные изменения, чтобы последняя цифра не мельтешила
		float voltDelta = voltLcd - voltShown;

		if((voltShown < 0.0f)||(voltDelta >= UI_LCD_DEADBAND)||(voltDelta <= -UI_LCD_DEADBAND))
		{
			voltShown = voltLcd;

			//для уменьшения размера во флеш используем библиотеку Float_transform
			Float_transform(voltLcd, 2, &sign_number, &integer_number, &fractional_number);
			if (!sign_number)
			{
				snprintf(strValueU, sizeof(strValueU), "Uв: %d.%.02ld\n", integer_number, fractional_number);
			}
		}

		//контроль напряжения питания с подтверждением
		if((valueU > (float)Settings_PA.SET_MAX_U)||(valueU < (float)Settings_PA.SET_MIN_U))
		{
			if(u_fault_cnt < 0xFFFF) {u_fault_cnt++;}
			if(u_fault_cnt >= U_FAULT_CNT)
			{
				if(valueU > (float)Settings_PA.SET_MAX_U)
				{
					PROTECTION_fault(PA_FAULT_OVOLT);
				}
				else
				{
					PROTECTION_fault(PA_FAULT_UVOLT);
				}
			}
		}
		else
		{
			u_fault_cnt = 0;
		}
	}
	else
	{
		//датчик INA226 не найден
		snprintf(strValueU, sizeof(strValueU), "Uв: ???\n");
		voltShown = -1.0f;
	}

	//выводим значение
	LCD_Output_String(3, 37, strValueU, 0, ChUnInvers);
	//закрашиваем после значения
	LCD_Draw_Rectangle_Filled(lastPxString_X, 37, LCD_Width/2 - 11, 10, pxView_Off);
	//вертикальная разделительная линия
	LCD_Draw_Line(LCD_Width/2 - 1, 36, LCD_Width/2 - 1, 48, pxView_On);
}

/* функция измерения тока потребления усилителем */
void I_measurement()
{
	//получаем текущий ток потребления
	GET_current();

	//выводим значение
	LCD_Output_String(LCD_Width/2 + 2, 37, strValueCurrent, 0, ChUnInvers);
	//закрашиваем после значения
	LCD_Draw_Rectangle_Filled(lastPxString_X, 37, LCD_Width - 1, 10, pxView_Off);
}

/* функция измерения температуры */
void T_measurement()
{
	int16_t valueT 		= 0;

	if(flag_lm75a == 1)
	{
		valueT = LM75_temperature();

		snprintf(strValueT, sizeof(strValueT), "%d°C\n", valueT);

		//контроль температуры, включаем/выключаем вентилятор охлаждения
		if(fan_test_override)
		{
			//режим проверки ключа: вентилятор принудительно включён
			FAN_set();
			fan_on = 1;
		}
		else if(valueT >= Settings_PA.SET_TEMP_COOLER)
		{
			//включаем вентилятор
			FAN_set();
			fan_on = 1;

			//авто включение подсветки дисплея
			LED_auto_on();
		}
		else if(valueT < (Settings_PA.SET_TEMP_COOLER - TEMP_G))
		{
			//выключаем вентилятор
			FAN_reset();
			fan_on = 0;
		}

		//аварийное отключение усилителя при перегреве
		if(valueT >= DEF_MAX_TEMP)
		{
			FAN_set();
			fan_on = 1;

			if(t_fault_cnt < 0xFFFF) {t_fault_cnt++;}
			if(t_fault_cnt >= 2)
			{
				PROTECTION_fault(PA_FAULT_TEMP);
			}
		}
		else
		{
			t_fault_cnt = 0;
		}

		//выводим значение, выделяя его при включенном вентиляторе
		if(fan_on)
		{
			//закрашиваем поле
			LCD_Draw_Rectangle_Filled(1, 49, 33, 14, pxView_On);
			//выводим значение
			LCD_Output_String(3, 50, strValueT, 0, ChInvers);
			//закрашиваем после значения
			LCD_Draw_Rectangle_Filled(lastPxString_X, 49, 31 - lastPxString_X, 14, pxView_On);
		}
		else
		{
			//закрашиваем поле
			LCD_Draw_Rectangle_Filled(1, 49, 33, 14, pxView_Off);
			//выводим значение
			LCD_Output_String(3, 50, strValueT, 0, ChUnInvers);
			//закрашиваем после значения
			LCD_Draw_Rectangle_Filled(lastPxString_X, 49, 31 - lastPxString_X, 14, pxView_Off);
		}
	}
	else
	{
		//датчик LM75A не найден
		snprintf(strValueT, sizeof(strValueT), "???\n");

		//закрашиваем поле
		LCD_Draw_Rectangle_Filled(1, 49, 33, 14, pxView_Off);
		//выводим значение
		LCD_Output_String(3, 50, strValueT, 0, ChUnInvers);
		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 49, 31 - lastPxString_X, 14, pxView_Off);
	}

	//вертикальная разделительная линия
	LCD_Draw_Line(34, 48, 34, 62, pxView_On);
}

/*-----------------------------------------------------------------*/

/* функция вывода метки диапазона ФНЧ на дисплей*/
void LPF_label_display()
{
	uint8_t xPos;

	//определяем координату Х для вывода метки
	switch (LPF_active)
	{
		case 0: 	xPos = 96;  //160
		break;
		case 1: 	xPos = 98;  //80
		break;
		case 2: 	xPos = 98;  //40
		break;
		case 3: 	xPos = 98;  //30
		break;
		case 4: 	xPos = 98;  //20
		break;
		default:	xPos = 100; //17-10
		break;
	}

	//если в настройках включено ручное изменение параметра то выделяем если изменяемый параметр активен
	if ((sel_work_param == 1)&&(Settings_PA.SET_MAN_CONTROL_LPF == OFF)&&(flag_ptt_on != 1))
	{
		//закрашиваем до значения
		LCD_Draw_Rectangle_Filled(87, 25, LCD_Width - 1, 10, pxView_On);
		//выводим значение
		LCD_Output_String(xPos, 25, LPF_band_string[LPF_active], 0, ChInvers);
		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 25, LCD_Width - 1, 10, pxView_On);
	}
	else
	{
		//закрашиваем до значения
		LCD_Draw_Rectangle_Filled(87, 25, LCD_Width - 1, 10, pxView_Off);
		//выводим значение
		LCD_Output_String(xPos, 25, LPF_band_string[LPF_active], 0, ChUnInvers);
		//закрашиваем после значения
		LCD_Draw_Rectangle_Filled(lastPxString_X, 25, LCD_Width - 1, 10, pxView_Off);
	}
}

/* функция переключения диапазонов ФНЧ */
void LPF_change_band()
{
	/*
	 *
	 * общепринятые ABCD коды для управления 9-ти диапазонными ДПФ,
	 * биты в комментариях — ABCD (A — старший слева)
	 * 160m	= 0000
	 * 80m	= 1000
	 * 40m	= 0100
	 * 30m	= 1100
	 * 20m	= 0010
	 * 17m	= 1010
	 * 15m	= 0110
	 * 12m	= 1110
	 * 10m	= 0001
	 */

	if(LPF_active == _160) //0000
	{
		A_reset();
		B_reset();
		C_reset();
		D_reset();
	}

	if(LPF_active == _80) //1000
	{
		A_set();
		B_reset();
		C_reset();
		D_reset();
	}

	if(LPF_active == _40) //0100
	{
		A_reset();
		B_set();
		C_reset();
		D_reset();
	}

	if(LPF_active == _30) //1100
	{
		A_set();
		B_set();
		C_reset();
		D_reset();
	}

	if(LPF_active == _20) //0010
	{
		A_reset();
		B_reset();
		C_set();
		D_reset();
	}

	if(LPF_active == _17) //1010
	{
		A_set();
		B_reset();
		C_set();
		D_reset();
	}

	if(LPF_active == _15) //0110
	{
		A_reset();
		B_set();
		C_set();
		D_reset();
	}

	if(LPF_active == _12) //1110
	{
		A_set();
		B_set();
		C_set();
		D_reset();
	}

	if(LPF_active == _10) //0001
	{
		A_reset();
		B_reset();
		C_reset();
		D_set();
	}

	//устанавливаем текущее затухание АТТ
	PE4302_set_att();

	//запоминаем фактически применённый диапазон
	old_band = LPF_active;
}

/* функция чтения внешних сигналов переключения ФНЧ */
void LPF_external_command()
{
	/*
	 *
	 * общепринятые ABCD коды для управления 9-ти диапазонными ДПФ,
	 * биты в комментариях — ABCD (A — старший слева)
	 * 160m	= 0000
	 * 80m	= 1000
	 * 40m	= 0100
	 * 30m	= 1100
	 * 20m	= 0010
	 * 17m	= 1010
	 * 15m	= 0110
	 * 12m	= 1110
	 * 10m	= 0001
	 */

	//160
	if((!HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(!HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(!HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _160;
	}

	//80
	if((HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(!HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(!HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _80;
	}

	//40
	if((!HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(!HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _40;
	}

	//30
	if((HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(!HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _30;
	}

	//20
	if((!HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(!HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _20;
	}

	//17
	if((HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(!HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _17;
	}

	//15
	if((!HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _15;
	}

	//12
	if((HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(!HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _12;
	}

	//10
	if((!HAL_GPIO_ReadPin(A_EXT_GPIO_Port, A_EXT_Pin))&&(!HAL_GPIO_ReadPin(B_EXT_GPIO_Port, B_EXT_Pin))&&
			(!HAL_GPIO_ReadPin(C_EXT_GPIO_Port, C_EXT_Pin))&&(HAL_GPIO_ReadPin(D_EXT_GPIO_Port, D_EXT_Pin)))
	{
		LPF_active = _10;
	}

	//переключаем ФНЧ если диапазон сменился
	if(old_band != LPF_active)
	{
		LPF_change_band();
		//запоминаем активный диапазон
		old_band = LPF_active;
		//авто включение подсветки дисплея
		LED_auto_on();
	}
}

/*-----------------------------------------------------------------*/

/* функция управления аттенюатором PE4302 */
void PE4302_change_att(uint8_t param)
{
	static volatile uint8_t direction = 0; //направление

	if(param == 0)
	{
		//увеличиваем или уменьшаем затухание аттенюатора в зависимости от направления
		if(direction == 0) {att_data++;} //увеличиваем затухание
		if(direction == 1) {att_data--;} //уменьшаем затухание
	}

	//проверяем состояние управляющих битов и выставляем соответствующий логический уровень на пинах PE4302 V1...V6
	if(att_data & (1<<0)) {PE_V1_set();} else {PE_V1_reset();}
	if(att_data & (1<<1)) {PE_V2_set();} else {PE_V2_reset();}
	if(att_data & (1<<2)) {PE_V3_set();} else {PE_V3_reset();}
	if(att_data & (1<<3)) {PE_V4_set();} else {PE_V4_reset();}
	if(att_data & (1<<4)) {PE_V5_set();} else {PE_V5_reset();}
	if(att_data & (1<<5)) {PE_V6_set();} else {PE_V6_reset();}

	if(param == 0)
	{
		if(att_data >= 63) {direction = 1;} //уменьшение затухания
		//увеличение затухания, но не более установленного в настройках для активного диапазона
		if(att_data <= bnd_in_att_data[LPF_active])
		{
			direction = 0;
			att_data = bnd_in_att_data[LPF_active];
		}

		//запомнаем установленное значение затухания на активном диапазоне
		band_att_data[LPF_active] = att_data;
	}
}

/* функция установки затухания аттенюатора на активном диапазоне */
void PE4302_set_att()
{
	if(Settings_PA.SET_CHANGE_POWER == OFF)
	{
		/* устанавливаем указанное значение настроек АТТ для текущего диапазона
		 * если запрещено ручное изменение выходной мощности
		 */
		att_data = bnd_in_att_data[LPF_active];
		PE4302_change_att(1);
	}
}

/*-----------------------------------------------------------------*/
/* обработка сигнала PTT и система защиты                          */
/*-----------------------------------------------------------------*/

/* функция обработки изменения состояния PTT (вызывается из основного цикла) */
void PTT_process(void)
{
	if(flag_ptt_change == 0)
	{
		return;
	}

	//сбрасываем флаг изменения
	flag_ptt_change = 0;

	//светодиод PTT: при передаче горит (PC13, активный низкий уровень)
	if(flag_ptt_on == 1)
	{
		LED_PTT_reset();
	}
	else
	{
		LED_PTT_set();
	}

	//авто включение подсветки дисплея
	LED_auto_on();

	//если разрешено изменение выходной мощности
	if(Settings_PA.SET_CHANGE_POWER == ON)
	{
		if(flag_ptt_on == 1)
		{
			//на передаче возвращаем установленное затухание аттенюатора активного диапазона
			att_data = band_att_data[LPF_active];
		}
		else
		{
			//на приеме затухание не нужно
			att_data = 0;
		}

		//применяем затухание
		PE4302_change_att(1);
	}
}

/* аварийное включение вентилятора при отсутствии датчика температуры */
void FAN_fail_safe(void)
{
	if((Settings_PA.SET_FAN_NO_LM75A == ON) && (flag_lm75a != 1))
	{
		FAN_set();
		fan_on = 1;
	}
}

/* принудительное включение/выключение вентилятора для проверки ключа */
void FAN_test_set(uint8_t on)
{
	fan_test_override = (on != 0U) ? 1U : 0U;

	if(fan_test_override)
	{
		FAN_set();
		fan_on = 1;
	}
	else
	{
		FAN_reset();
		fan_on = 0;
	}
}

/* функция периодического опроса датчиков на шине I2C */
void SENSOR_poll(void)
{
	uint32_t tick = HAL_GetTick();

	if((tick - sensor_poll_tick) < SENSOR_POLL_MS)
	{
		return;
	}

	sensor_poll_tick = tick;

	//опрос датчиков
	SEARCH_sensor_on_i2c_bus();

	//если датчика температуры нет и разрешено - сразу включаем вентилятор
	FAN_fail_safe();

	//потеря датчика в рабочем режиме - авария, без датчиков защита невозможна
	if((flag_ina226 != 1)||(flag_lm75a != 1)||(ina226_error_cnt >= INA226_ERROR_LIMIT))
	{
		if(sensor_fault_cnt < 0xFFFF) {sensor_fault_cnt++;}
		if(sensor_fault_cnt >= SENSOR_FAULT_CNT)
		{
			if((flag_ina226 != 1)||(ina226_error_cnt >= INA226_ERROR_LIMIT))
			{
				PROTECTION_fault(PA_FAULT_INA226);
			}
			else
			{
				PROTECTION_fault(PA_FAULT_LM75A);
			}
		}
	}
	else
	{
		sensor_fault_cnt = 0;
	}
}

/* задержка с периодическим сбросом IWDG (для пауз больше 1 с) */
void DELAY_iwdg(uint32_t delay_ms)
{
	uint32_t tick = HAL_GetTick();

	HAL_IWDG_Refresh(&hiwdg);

	while((HAL_GetTick() - tick) < delay_ms)
	{
		HAL_IWDG_Refresh(&hiwdg);
		PA_Protocol_Poll();
		HAL_Delay(100);
	}
}

/* функция проверки периода обновления показаний U/I/T */
uint8_t MEASURE_is_due(void)
{
	uint32_t tick = HAL_GetTick();

	if((tick - measure_poll_tick) < MEASURE_PERIOD_MS)
	{
		return 0;
	}

	measure_poll_tick = tick;

	return 1;
}

/* функция перевода усилителя в аварийный режим */
void PROTECTION_fault(PA_FAULT code)
{
	if(pa_fault != PA_FAULT_NONE)
	{
		//авария уже зафиксирована
		return;
	}

	//фиксируем аварию
	pa_fault = code;

	//немедленно снимаем питание с усилителя
	CONTROL_U_reset();

	//сбрасываем счетчики подтверждения
	u_fault_cnt = 0;
	i_fault_cnt = 0;
	t_fault_cnt = 0;

	//включаем вентилятор и подсветку дисплея
	FAN_set();
	fan_on = 1;
	LED_auto_on();

	//выводим причину аварии
	LCD_Clean_Buffer_Frame();
	LCD_Output_String_Center(12, "АВАРИЯ\n", 0, ChUnInvers);

	switch(code)
	{
		case PA_FAULT_VOLTAGE:
			LCD_Output_String_Center(30, "НАПРЯЖЕНИЕ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_UVOLT:
			LCD_Output_String_Center(30, "U НИЗКОЕ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_OVOLT:
			LCD_Output_String_Center(30, "U ВЫСОКОЕ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_CURRENT:
			LCD_Output_String_Center(30, "ТОК ПОТРЕБЛЕНИЯ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_SWR:
			LCD_Output_String_Center(30, "КСВ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_TEMP:
			LCD_Output_String_Center(30, "ПЕРЕГРЕВ\n", 0, ChUnInvers);
			break;

		case PA_FAULT_SENSOR:
			LCD_Output_String_Center(30, "ДАТЧИКИ I2C\n", 0, ChUnInvers);
			break;

		case PA_FAULT_INA226:
			LCD_Output_String_Center(30, "ДАТЧИК INA226\n", 0, ChUnInvers);
			break;

		case PA_FAULT_LM75A:
			LCD_Output_String_Center(30, "ДАТЧИК LM75A\n", 0, ChUnInvers);
			break;

		default:
			break;
	}

	LCD_Output_String_Center(52, "ПИТАНИЕ ОТКЛ.\n", 0, ChUnInvers);
	LCD_Update();

	//даем время прочитать сообщение
	HAL_Delay(1000);
	HAL_IWDG_Refresh(&hiwdg);
}

/*
 * функция контроля и восстановления после аварии.
 * Возвращает 1, если авария снята и усилитель можно вернуть в рабочий режим.
 *
 * ВАЖНО: INA226 стоит после ключа CONTROL_U, поэтому напряжение и ток
 * нельзя проверить, пока питание снято. Для этих аварий питание подается
 * коротким импульсом (FAULT_TEST_PWR_MS) только при отпущенном PTT.
 */
uint8_t PROTECTION_recover(void)
{
	static uint8_t ok_started = 0;		//признак начала отсчета "все в норме"
	static uint32_t ok_tick = 0;		//метка начала отсчета "все в норме"

	uint8_t norm = 0;					//условия аварии устранены
	uint8_t power_test = 0;				//был импульс проверки питания
	uint32_t tick = 0;
	float valueU = 0;
	float valueI = 0;

	//сброс IWDG и опрос датчиков
	HAL_IWDG_Refresh(&hiwdg);
	SEARCH_sensor_on_i2c_bus();

	switch(pa_fault)
	{
		case PA_FAULT_VOLTAGE:
		case PA_FAULT_UVOLT:
		case PA_FAULT_OVOLT:
		case PA_FAULT_CURRENT:
			//проверка возможна только при отпущенном PTT
			if(flag_ptt_on != 1)
			{
				//подаем питание на время проверки
				CONTROL_U_set();
				HAL_Delay(FAULT_TEST_PWR_MS);

				valueU = INA226_Vbus();
				valueI = INA226_Current();

				//снимаем питание
				CONTROL_U_reset();

				if((pa_fault == PA_FAULT_VOLTAGE)||(pa_fault == PA_FAULT_UVOLT)||(pa_fault == PA_FAULT_OVOLT))
				{
					norm = (flag_ina226 == 1)&&(valueU >= (float)Settings_PA.SET_MIN_U)&&(valueU <= (float)Settings_PA.SET_MAX_U);
				}
				else
				{
					norm = (flag_ina226 == 1)&&(valueI < (float)Settings_PA.SET_MAX_I);
				}

				power_test = 1;
			}
			break;

		case PA_FAULT_SWR:
			//после аварии по КСВ усилитель возвращается в работу после отпускания PTT
			norm = (flag_ptt_on != 1);
			break;

		case PA_FAULT_TEMP:
			//ждем остывания ниже аварийного порога с запасом
			norm = (flag_lm75a == 1)&&(LM75_temperature() < (DEF_MAX_TEMP - 10));
			break;

		case PA_FAULT_SENSOR:
		case PA_FAULT_INA226:
		case PA_FAULT_LM75A:
			norm = (flag_ina226 == 1)&&(flag_lm75a == 1)&&(ina226_error_cnt < INA226_ERROR_LIMIT);
			break;

		default:
			norm = 1;
			break;
	}

	//возврат в работу разрешен только при отпущенном PTT
	if((flag_ptt_on == 1)&&(pa_fault != PA_FAULT_SWR))
	{
		norm = 0;
	}

	if(norm)
	{
		tick = HAL_GetTick();

		if(ok_started == 0)
		{
			ok_started = 1;
			ok_tick = tick;
		}
		else if((tick - ok_tick) >= FAULT_OK_MS)
		{
			//авария снята
			ok_started = 0;

			pa_fault = PA_FAULT_NONE;
			u_fault_cnt = 0;
			i_fault_cnt = 0;
			t_fault_cnt = 0;
			sensor_fault_cnt = 0;
			swr_bad_tick = 0;

			//выводим сообщение
			LCD_Clean_Buffer_Frame();
			LCD_Output_String_Center(25, "ВОЗВРАТ\n", 0, ChUnInvers);
			LCD_Update();

			HAL_Delay(500);

			//возвращаем питание на усилитель
			CONTROL_U_set();

			return 1;
		}
	}
	else
	{
		ok_started = 0;
	}

	//период повторной проверки
	if(power_test)
	{
		DELAY_iwdg(FAULT_TEST_MS - FAULT_TEST_PWR_MS);
	}
	else
	{
		DELAY_iwdg(500);
	}

	HAL_IWDG_Refresh(&hiwdg);

	return 0;
}

/*-----------------------------------------------------------------*/

/* функция прорисовки окна рабочего режима */
void DISPLAY_work()
{
	//рисуем рамку
	LCD_Draw_Rectangle(0, 0, LCD_Width - 1, LCD_Height - 1, pxView_On);
	//вывод значения мощности
	LCD_Output_String(3, 0, "Мощность\n", 0, ChUnInvers);
	//горизонтальная разделительная линия
	LCD_Draw_Line(0, 12, LCD_Width - 1, 12, pxView_On);
	//вывод значения КСВ
	LCD_Output_String(3, 13, "КСВ\n", 0, ChUnInvers);
	//горизонтальная разделительная линия
	LCD_Draw_Line(0, 24, LCD_Width - 1, 24, pxView_On);
	//вывод значения КСВ
	LCD_Output_String(3, 25, "Секция ФНЧ\n", 0, ChUnInvers);
	//горизонтальная разделительная линия
	LCD_Draw_Line(0, 36, LCD_Width - 1, 36, pxView_On);
	//вертикальная разделительная линия
	LCD_Draw_Line(86, 0, 86, 36, pxView_On);
	//вертикальная разделительная линия
	LCD_Draw_Line(LCD_Width/2 - 1, 36, LCD_Width/2 - 1, 48, pxView_On);
	//горизонтальная разделительная линия
	LCD_Draw_Line(0, 48, LCD_Width - 1, 48, pxView_On);
}

/* функция записи значений настроек по умолчанию */
void SETUP_pa_default()
{
	/* параметры аттенюатора по диапазонам 160...10 м */
	const uint32_t att_param[9] = {PARAM_9, PARAM_10, PARAM_11, PARAM_12, PARAM_13, PARAM_14, PARAM_15, PARAM_16, PARAM_17};

	//подсветка дисплея
	EEPROM_Write(PARAM_1, ON);
	Settings_PA.SET_LED_DISPLAY = ON;

	//изменение выходной мощности
	EEPROM_Write(PARAM_2, OFF);
	Settings_PA.SET_CHANGE_POWER = OFF;

	//управление ФНЧ
	EEPROM_Write(PARAM_3, OFF);
	Settings_PA.SET_MAN_CONTROL_LPF = OFF;

	//температура включения вентилятора охлаждения
	EEPROM_Write(PARAM_4, DEF_T);
	Settings_PA.SET_TEMP_COOLER = DEF_T;

	//минимальное напряжение
	EEPROM_Write(PARAM_5, DEF_MIN_U);
	Settings_PA.SET_MIN_U = DEF_MIN_U;

	//максимальное напряжение
	EEPROM_Write(PARAM_6, DEF_MAX_U);
	Settings_PA.SET_MAX_U = DEF_MAX_U;

	//максимальный ток потребления
	EEPROM_Write(PARAM_7, DEF_MAX_I);
	Settings_PA.SET_MAX_I = DEF_MAX_I;

	//максимальное КСВ
	EEPROM_Write(PARAM_8, DEF_MAX_SWR);
	Settings_PA.SET_MAX_SWR = DEF_MAX_SWR;

	//затухание АТТ по всем диапазонам
	for(uint8_t i = 0; i < 9; i++)
	{
		EEPROM_Write(att_param[i], 0);
		bnd_in_att_data[i] = 0;
		band_att_data[i] = 0;
	}

	//версия формата настроек
	EEPROM_Write(PARAM_FORMAT, EEPROM_FORMAT_VERSION);

	//применяем параметр подсветки дисплея
	LED_display(Settings_PA.SET_LED_DISPLAY);
}

/* функция чтения настроек усилителя */
void READ_setup_pa()
{
	uint32_t readData[18] = {0xFF,0xFF,0xFF};
	uint32_t formatVersion = 0;

	//проверяем версию формата хранения настроек
	if((EEPROM_Read(PARAM_FORMAT, &formatVersion) != EEPROM_OK)||(formatVersion != EEPROM_FORMAT_VERSION))
	{
		//формат не совпадает - записываем настройки по умолчанию
		SETUP_pa_default();
	}

	//читаем параметры из флеша
	EEPROM_Read(PARAM_1, &readData[0]);
	EEPROM_Read(PARAM_2, &readData[1]);
	EEPROM_Read(PARAM_3, &readData[2]);
	EEPROM_Read(PARAM_4, &readData[3]);
	EEPROM_Read(PARAM_5, &readData[4]);
	EEPROM_Read(PARAM_6, &readData[5]);
	EEPROM_Read(PARAM_7, &readData[6]);
	EEPROM_Read(PARAM_8, &readData[7]);
	EEPROM_Read(PARAM_9, &readData[8]);
	EEPROM_Read(PARAM_10, &readData[9]);
	EEPROM_Read(PARAM_11, &readData[10]);
	EEPROM_Read(PARAM_12, &readData[11]);
	EEPROM_Read(PARAM_13, &readData[12]);
	EEPROM_Read(PARAM_14, &readData[13]);
	EEPROM_Read(PARAM_15, &readData[14]);
	EEPROM_Read(PARAM_16, &readData[15]);
	EEPROM_Read(PARAM_17, &readData[16]);
	EEPROM_Read(PARAM_18, &readData[17]);

	//проверяем считанные значения, если некорректны, записываем значения по умолчанию
	//подсветка дисплея
	if((readData[0] != ON)&&(readData[0] != OFF)&&(readData[0] != AUTO))
	{
		EEPROM_Write(PARAM_1, ON);
		Settings_PA.SET_LED_DISPLAY = ON;
	}
	else
	{
		Settings_PA.SET_LED_DISPLAY = readData[0];
	}

	//применяем параметр подсветки дисплея
	LED_display(Settings_PA.SET_LED_DISPLAY);

	//изменение выходной мощности
	if((readData[1] != ON)&&(readData[1] != OFF))
	{
		EEPROM_Write(PARAM_2, OFF);
		Settings_PA.SET_CHANGE_POWER = OFF;
	}
	else
	{
		Settings_PA.SET_CHANGE_POWER = (uint8_t)readData[1];
	}

	//управление ФНЧ
	if((readData[2] != ON)&&(readData[2] != OFF))
	{
		EEPROM_Write(PARAM_3, OFF);
		Settings_PA.SET_MAN_CONTROL_LPF	= OFF;
	}
	else
	{
		Settings_PA.SET_MAN_CONTROL_LPF = (uint8_t)readData[2];
	}

	//температура включения вентилятора охлаждения
	if((readData[3] < MIN_T)||(readData[3] > MAX_T))
	{
		EEPROM_Write(PARAM_4, DEF_T);
		Settings_PA.SET_TEMP_COOLER = DEF_T;
	}
	else
	{
		Settings_PA.SET_TEMP_COOLER = (uint8_t)readData[3];
	}

	//минимальное напряжение
	if((readData[4] < DEF_MIN_U)||(readData[4] > DEF_MAX_U))
	{
		EEPROM_Write(PARAM_5, DEF_MIN_U);
		Settings_PA.SET_MIN_U = DEF_MIN_U;
	}
	else
	{
		Settings_PA.SET_MIN_U = (uint8_t)readData[4];
	}

	//максимальное напряжение
	if((readData[5] < DEF_MIN_U)||(readData[5] > (DEF_MAX_U + 5)))
	{
		EEPROM_Write(PARAM_6, DEF_MAX_U);
		Settings_PA.SET_MAX_U = DEF_MAX_U;
	}
	else
	{
		Settings_PA.SET_MAX_U = (uint8_t)readData[5];
	}

	//максимальный ток потребления
	if((readData[6] < DEF_MIN_I)||(readData[6] > DEF_MAX_I))
	{
		EEPROM_Write(PARAM_7, DEF_MAX_I);
		Settings_PA.SET_MAX_I = DEF_MAX_I;
	}
	else
	{
		Settings_PA.SET_MAX_I = (uint8_t)readData[6];
	}

	//максимальное КСВ
	if((readData[7] < MIN_SWR)||(readData[7] > MAX_SWR))
	{
		EEPROM_Write(PARAM_8, DEF_MAX_SWR);
		Settings_PA.SET_MAX_SWR = DEF_MAX_SWR;
	}
	else
	{
		Settings_PA.SET_MAX_SWR = (uint8_t)readData[7];
	}

	//вентилятор при отсутствии датчика температуры LM75A
	if((readData[17] != ON)&&(readData[17] != OFF))
	{
		EEPROM_Write(PARAM_18, OFF);
		Settings_PA.SET_FAN_NO_LM75A = OFF;
	}
	else
	{
		Settings_PA.SET_FAN_NO_LM75A = (uint8_t)readData[17];
	}

	//затухание АТТ настройки 160м
	if(readData[8] > 63)
	{
		EEPROM_Write(PARAM_9, 0);
		bnd_in_att_data[0] = 0;
	}
	else
	{
		bnd_in_att_data[0] = (uint8_t)readData[8];
	}

	//затухание АТТ настройки 80м
	if(readData[9] > 63)
	{
		EEPROM_Write(PARAM_10, 0);
		bnd_in_att_data[1] = 0;
	}
	else
	{
		bnd_in_att_data[1] = (uint8_t)readData[9];
	}

	//затухание АТТ настройки 40м
	if(readData[10] > 63)
	{
		EEPROM_Write(PARAM_11, 0);
		bnd_in_att_data[2] = 0;
	}
	else
	{
		bnd_in_att_data[2] = (uint8_t)readData[10];
	}

	//затухание АТТ настройки 30м
	if(readData[11] > 63)
	{
		EEPROM_Write(PARAM_12, 0);
		bnd_in_att_data[3] = 0;
	}
	else
	{
		bnd_in_att_data[3] = (uint8_t)readData[11];
	}

	//затухание АТТ настройки 20м
	if(readData[12] > 63)
	{
		EEPROM_Write(PARAM_13, 0);
		bnd_in_att_data[4] = 0;
	}
	else
	{
		bnd_in_att_data[4] = (uint8_t)readData[12];
	}

	//затухание АТТ настройки 17м
	if(readData[13] > 63)
	{
		EEPROM_Write(PARAM_14, 0);
		bnd_in_att_data[5] = 0;
	}
	else
	{
		bnd_in_att_data[5] = (uint8_t)readData[13];
	}

	//затухание АТТ настройки 15м
	if(readData[14] > 63)
	{
		EEPROM_Write(PARAM_15, 0);
		bnd_in_att_data[6] = 0;
	}
	else
	{
		bnd_in_att_data[6] = (uint8_t)readData[14];
	}

	//затухание АТТ настройки 12м
	if(readData[15] > 63)
	{
		EEPROM_Write(PARAM_16, 0);
		bnd_in_att_data[7] = 0;
	}
	else
	{
		bnd_in_att_data[7] = (uint8_t)readData[15];
	}

	//затухание АТТ настройки 10м
	if(readData[16] > 63)
	{
		EEPROM_Write(PARAM_17, 0);
		bnd_in_att_data[8] = 0;
	}
	else
	{
		bnd_in_att_data[8] = (uint8_t)readData[16];
	}

	//устанавливаем указанные значения затухания аттенюатора по диапазонам
	for(uint8_t i = 0; i < 9; i++)
	{
		band_att_data[i] = bnd_in_att_data[i];
	}
}

/*-----------------------------------------------------------------*/

/* функция закраски справа от значения выделенного параметра */
void FILL_sel_param(uint8_t yPos)
{
	LCD_Draw_Rectangle_Filled(lastPxString_X, yPos + 1, LCD_Width - 1, 9, pxView_On);
}

/* функция перемещения по пунктам меню первой страницы настроек */
void SETUP_1_move(uint8_t sel_param)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (sel_param != i)
		{
			//выводим не активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_1_index[i], 0, ChUnInvers);

			//выводим значение параметра не активного меню настроек
			if (i == 0)
			{
				if (Settings_PA.SET_LED_DISPLAY == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[0], 0, ChUnInvers);
				}
				else if (Settings_PA.SET_LED_DISPLAY == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[1], 0, ChUnInvers);
				}
				else if (Settings_PA.SET_LED_DISPLAY == AUTO)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[4], 0, ChUnInvers);
				}
			}
			else if (i == 1)
			{
				if (Settings_PA.SET_CHANGE_POWER == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[0], 0, ChUnInvers);
				}
				else if (Settings_PA.SET_CHANGE_POWER == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[1], 0, ChUnInvers);
				}
			}
			else if (i == 2)
			{
				if (Settings_PA.SET_MAN_CONTROL_LPF == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[i + 1], 0, ChUnInvers);
				}
				else if (Settings_PA.SET_MAN_CONTROL_LPF == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[i], 0, ChUnInvers);
				}
			}
			else if (i == 3)
			{
				char strValuePwr[5] = "";
				snprintf(strValuePwr, sizeof(strValuePwr), "%d\n", Settings_PA.SET_TEMP_COOLER);
				LCD_Output_String(100, (i + 1) * 12, strValuePwr, 0, ChUnInvers);
			}
		}
		else
		{
			//выводим активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_1_index[i], 0, ChInvers);

			//закрашиваем слева от пункта меню настроек
			LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, 99, 9, pxView_On);

			//выводим значение параметра активного меню настроек
			if (i == 0)
			{
				if (Settings_PA.SET_LED_DISPLAY == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[0], 0, ChInvers);
					//закрашиваем слева от пункта меню настроек
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
				else if (Settings_PA.SET_LED_DISPLAY == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[1], 0, ChInvers);
					//закрашиваем слева от пункта меню настроек
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
				else if (Settings_PA.SET_LED_DISPLAY == AUTO)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[4], 0, ChInvers);
					//закрашиваем слева от пункта меню настроек
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
			}
			else if (i == 1)
			{
				if (Settings_PA.SET_CHANGE_POWER == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[0], 0, ChInvers);
					//закрашиваем слева от значения
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
				else if (Settings_PA.SET_CHANGE_POWER == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[1], 0, ChInvers);
					//закрашиваем слева от значения
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
			}
			else if (i == 2)
			{
				if (Settings_PA.SET_MAN_CONTROL_LPF == ON)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[i + 1], 0, ChInvers);
					//закрашиваем слева от значения
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);

				}
				else if (Settings_PA.SET_MAN_CONTROL_LPF == OFF)
				{
					LCD_Output_String(100, (i + 1) * 12, SETUP_1_param_index[i], 0, ChInvers);
					//закрашиваем слева от значения
					LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
				}
			}
			else if (i == 3)
			{
				char strValuePwr[5] = "";
				snprintf(strValuePwr, sizeof(strValuePwr), "%d\n", Settings_PA.SET_TEMP_COOLER);
				LCD_Output_String(100, (i + 1) * 12, strValuePwr, 0, ChInvers);
				//закрашиваем слева от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
		}
	}

	LCD_Update();
}

/* функция перемещения по пунктам второй страницы настроек */
void SETUP_2_move(uint8_t sel_param)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (sel_param != i)
		{
			//выводим не активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_2_index[i], 0, ChUnInvers);

			//выводим значение параметра не активного меню настроек
			if (i == 0)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MIN_U);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 1)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_U);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 2)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_I);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 3)
			{
				//выводим значение корректировки тока потребления
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_SWR);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
		}
		else
		{
			//выводим активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_2_index[i], 0, ChInvers);

			//закрашиваем справа от пункта меню настроек
			LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, 99, 9, pxView_On);

			//выводим значение параметра активного меню настроек
			if (i == 0)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MIN_U);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 1)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_U);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 2)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_I);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 3)
			{
				snprintf(strValue, sizeof(strValue), "%d\n", Settings_PA.SET_MAX_SWR);
				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
		}
	}

	LCD_Update();
}

/* функция перемещения по пунктам третьей страницы настроек */
void SETUP_3_move(uint8_t sel_param)
{
	/*если включен ручной режим смены секций ФНЧ,
	  меняем на секцию в соответствии с выбранным пунктом меню настроек
	 */
	if(Settings_PA.SET_MAN_CONTROL_LPF != ON)
	{
		LPF_active = sel_param;
		LPF_change_band();
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		if (sel_param != i)
		{
			//выводим не активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_3_index[i], 0, ChUnInvers);

			//выводим значение параметра не активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[0] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 1)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[1] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 2)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[2] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 3)
			{
				//выводим значение корректировки тока потребления
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[3] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
		}
		else
		{
			//выводим активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_3_index[i], 0, ChInvers);

			//закрашиваем справа от пункта меню настроек
			LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, 99, 9, pxView_On);

			//выводим значение параметра активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[0] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 1)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[1] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 2)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[2] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 3)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[3] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
		}
	}

	LCD_Update();
}

/* функция перемещения по пунктам четвертой страницы настроек */
void SETUP_4_move(uint8_t sel_param)
{
	/*если включен ручной режим смены секций ФНЧ,
	  меняем на секцию в соответствии с выбранным пунктом меню настроек
	 */
	if(Settings_PA.SET_MAN_CONTROL_LPF != ON)
	{
		LPF_active = sel_param + 4;
		LPF_change_band();
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		if (sel_param != i)
		{
			//выводим не активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_4_index[i], 0, ChUnInvers);

			//выводим значение параметра не активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[4] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 1)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[5] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 2)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[6] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
			else if (i == 3)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[7] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
		}
		else
		{
			//выводим активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_4_index[i], 0, ChInvers);

			//закрашиваем справа от пункта меню настроек
			LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, 99, 9, pxView_On);

			//выводим значение параметра активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[4] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 1)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[5] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 2)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[6] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
			else if (i == 3)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[7] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);
				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
		}
	}

	LCD_Update();
}

/* функция перемещения по пунктам пятой страницы настроек */
void SETUP_5_move(uint8_t sel_param)
{
	/*если включен ручной режим смены секций ФНЧ,
	  меняем на секцию в соответствии с выбранным пунктом меню настроек
	 */
	if(Settings_PA.SET_MAN_CONTROL_LPF != ON)
	{
		LPF_active = _10;
		LPF_change_band();
	}

	for (uint8_t i = 0; i < 2; i++)
	{
		if (sel_param != i)
		{
			LCD_Output_String(0, (i + 1) * 12, SETUP_5_index[i], 0, ChUnInvers);

			//выводим значение параметра не активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[8] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChUnInvers);
			}
		}
		else
		{
			//выводим активный пункт меню настроек
			LCD_Output_String(0, (i + 1) * 12, SETUP_5_index[i], 0, ChInvers);

			//закрашиваем справа от пункта меню настроек
			LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, 99, 9, pxView_On);

			//выводим значение параметра активного меню настроек
			if (i == 0)
			{
				//значение затухание аттенюатора активного диапазона делим на 2, чтобы выводимый результат привести к значению в dB
				Float_transform((float)bnd_in_att_data[8] / 2.0f, 1, &sign_number, &integer_number, &fractional_number);
				if (!sign_number)
				{
					snprintf(strValue, sizeof(strValue), "-%d.%.01ld\n", integer_number, fractional_number);
				}

				LCD_Output_String(100, (i + 1) * 12, strValue, 0, ChInvers);

				//закрашиваем справа от значения
				LCD_Draw_Rectangle_Filled(lastPxString_X, ((i + 1) * 12) + 1, LCD_Width - 1, 9, pxView_On);
			}
		}
	}

	LCD_Update();
}

/* функция изменения параметра первой страницы настроек */
void SETUP_1_change_param(uint8_t sel_param)
{
	if (sel_param == 0)	//подсветка дисплея
	{
		if (Settings_PA.SET_LED_DISPLAY == ON)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_LED_DISPLAY = OFF;
		}
		else if (Settings_PA.SET_LED_DISPLAY == OFF)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_LED_DISPLAY = AUTO;
		}
		else if (Settings_PA.SET_LED_DISPLAY == AUTO)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_LED_DISPLAY = ON;
		}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_1, Settings_PA.SET_LED_DISPLAY);

		//применяем параметр подсветки дисплея
		LED_display(Settings_PA.SET_LED_DISPLAY);
	}
	else if (sel_param == 1) //изменение мощности
	{
		if (Settings_PA.SET_CHANGE_POWER == ON)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_CHANGE_POWER = OFF;
		}
		else if (Settings_PA.SET_CHANGE_POWER == OFF)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_CHANGE_POWER = ON;
		}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_2, Settings_PA.SET_CHANGE_POWER);

		//сбрасываем аттенюатор
		att_data = 0;
		PE4302_change_att(1);
	}
	else if (sel_param == 2) //управление ФНЧ
	{
		if (Settings_PA.SET_MAN_CONTROL_LPF == ON)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_MAN_CONTROL_LPF = OFF;
		}
		else if (Settings_PA.SET_MAN_CONTROL_LPF == OFF)
		{
			//меняем значение выбранного параметра меню настроек
			Settings_PA.SET_MAN_CONTROL_LPF = ON;
		}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_3, Settings_PA.SET_MAN_CONTROL_LPF);
	}
	else if (sel_param == 3) //температура включения вентилятора охлаждения
	{

		Settings_PA.SET_TEMP_COOLER++;
		if (Settings_PA.SET_TEMP_COOLER > MAX_T) {Settings_PA.SET_TEMP_COOLER = MIN_T;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_4, Settings_PA.SET_TEMP_COOLER);
	}
}

/* функция изменения параметра второй страницы настроек */
void SETUP_2_change_param(uint8_t sel_param)
{
	if (sel_param == 0)	//минимально допустимое напряжение питания
	{
		Settings_PA.SET_MIN_U--;
		if (Settings_PA.SET_MIN_U < DEF_MIN_U) {Settings_PA.SET_MIN_U = 12;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_5, Settings_PA.SET_MIN_U);
	}
	else if (sel_param == 1) //максимально допустимое напряжение питания
	{
		Settings_PA.SET_MAX_U++;
		if (Settings_PA.SET_MAX_U > DEF_MAX_U + 5) {Settings_PA.SET_MAX_U = DEF_MAX_U;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_6, Settings_PA.SET_MAX_U);
	}
	else if (sel_param == 2) //максимально допустимый ток потребления
	{
		Settings_PA.SET_MAX_I++;
		if (Settings_PA.SET_MAX_I > DEF_MAX_I) {Settings_PA.SET_MAX_I = DEF_MIN_I;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_7, Settings_PA.SET_MAX_I);
	}
	else if (sel_param == 3) //максимально допустимое КСВ
	{

		Settings_PA.SET_MAX_SWR++;
		if (Settings_PA.SET_MAX_SWR > MAX_SWR) {Settings_PA.SET_MAX_SWR = MIN_SWR;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_8, Settings_PA.SET_MAX_SWR);
	}
}

/* функция изменения параметра третьей страницы настроек */
void SETUP_3_change_param(uint8_t sel_param)
{
	if (sel_param == 0)	//затухание АТТ диапазона 160м
	{
		bnd_in_att_data[0]++;
		if (bnd_in_att_data[0] > 63) {bnd_in_att_data[0] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_9, bnd_in_att_data[0]);
	}
	else if (sel_param == 1) //затухание АТТ диапазона 80м
	{
		bnd_in_att_data[1]++;
		if (bnd_in_att_data[1] > 63) {bnd_in_att_data[1] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_10, bnd_in_att_data[1]);
	}
	else if (sel_param == 2) //затухание АТТ диапазона 40м
	{
		bnd_in_att_data[2]++;
		if (bnd_in_att_data[2] > 63) {bnd_in_att_data[2] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_11, bnd_in_att_data[2]);
	}
	else if (sel_param == 3) //затухание АТТ диапазона 30м
	{

		bnd_in_att_data[3]++;
		if (bnd_in_att_data[3] > 63) {bnd_in_att_data[3] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_12, bnd_in_att_data[3]);
	}
}

/* функция изменения параметра четвертой страницы настроек */
void SETUP_4_change_param(uint8_t sel_param)
{
	if (sel_param == 0)	//затухание АТТ диапазона 20м
	{
		bnd_in_att_data[4]++;
		if (bnd_in_att_data[4] > 63) {bnd_in_att_data[4] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_13, bnd_in_att_data[4]);
	}
	else if (sel_param == 1) //затухание АТТ диапазона 17м
	{
		bnd_in_att_data[5]++;
		if (bnd_in_att_data[5] > 63) {bnd_in_att_data[5] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_14, bnd_in_att_data[5]);
	}
	else if (sel_param == 2) //затухание АТТ диапазона 15м
	{
		bnd_in_att_data[6]++;
		if (bnd_in_att_data[6] > 63) {bnd_in_att_data[6] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_15, bnd_in_att_data[6]);
	}
	else if (sel_param == 3) //затухание АТТ диапазона 12м
	{

		bnd_in_att_data[7]++;
		if (bnd_in_att_data[7] > 63) {bnd_in_att_data[7] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_16, bnd_in_att_data[7]);
	}
}

/* функция изменения параметра пятой страницы настроек */
void SETUP_5_change_param(uint8_t sel_param)
{
	if (sel_param == 0)	//затухание АТТ диапазона 10м
	{
		bnd_in_att_data[8]++;
		if (bnd_in_att_data[8] > 63) {bnd_in_att_data[8] = 0;}

		//записываем новое значение в EEPROM
		EEPROM_Write(PARAM_17, bnd_in_att_data[8]);
	}
	else if (sel_param == 1) //сброс настроек
	{
		//устанавливаем все значения настроек по умолчанию
		SETUP_pa_default();

		//программный ресет
		NVIC_SystemReset();
	}
}

/*-----------------------------------------------------------------*/

/* функция выбора режима программы */
void PROG_mode()
{
	//защита от некорректного состояния: не даём главному циклу молча зависнуть
	if((uint8_t)PROG_state > (uint8_t)PROG_STATE_SETUP_5)
	{
		PROG_state = PROG_STATE_WORK;
		BUT_press = NO_BUT;
		sel_param = 0;
	}

	switch (PROG_state)
	{
		//режим работы
		case PROG_STATE_WORK:  PROG_mode_work();
		break;
		//режим настроек 1
		case PROG_STATE_SETUP_1: PROG_mode_setup_1();
		break;
		//режим настроек 2
		case PROG_STATE_SETUP_2: PROG_mode_setup_2();
		break;
		//режим настроек 3
		case PROG_STATE_SETUP_3: PROG_mode_setup_3();
		break;
		//режим настроек 4
		case PROG_STATE_SETUP_4: PROG_mode_setup_4();
		break;
		//режим настроек 5
		case PROG_STATE_SETUP_5: PROG_mode_setup_5();
		break;

		default:
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			break;
	}
}

/* функция цикла программы - работа */

void PROG_mode_work()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	DISPLAY_work();
	LCD_Update();
	//при входе в рабочий режим запускаем отсчет бездействия подсветки
	led_auto_start = HAL_GetTick();

	/* цикл рабочего процесса */
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		//обработка аварийной ситуации: пока не устранена, в рабочий режим не выходим
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, управление ФНЧ, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		//устанавливаем текущее затухание АТТ
		PE4302_set_att();

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		if(end_adc_filter_flag == 1)
		{
			//сбрасываем флаг готовности данных
			end_adc_filter_flag = 0;

			//измеряем выходную мощность
			POWER_measurement();

			//расчет КСВ
			SWR_measurement();
		}

		if(MEASURE_is_due() == 1)
		{
			//измеряем напряжение
			U_measurement();

			//измеряем ток потребления
			I_measurement();

			//измеряем температуру
			T_measurement();

			//вывод метки диапазона на дисплей
			LPF_label_display();
		}

		//обработка кнопок
		READ_button();

		//автоматическое отключение подсветки дисплея по времени бездействия
		//(отсчет идет по HAL_GetTick и не зависит от периода главного цикла)
		if(Settings_PA.SET_LED_DISPLAY == AUTO)
		{
			if(flag_ptt_on == 1)
			{
				//на передаче отсчет бездействия не ведем
				led_auto_start = HAL_GetTick();
			}
			else if((HAL_GetTick() - led_auto_start) >= LED_AUTO_OFF_MS)
			{
				//время бездействия вышло - отключаем подсветку
				LED_DISP_reset();
				led_disp_on = 0;
				//перезапускаем отсчет, чтобы не дергать порт в каждой итерации
				led_auto_start = HAL_GetTick();
			}
		}

		/* реакция на кнопки */
		if ((BUT_press == BUT_RIGHT)&&(flag_ptt_on != 1))
		{
			//авто включение подсветки дисплея
			LED_auto_on();

			/*
			 * перемещение по изменяемым параметрам
			 * инкремент индекса изменяемого параметра
			 * если параметры разрешены для редактирования по нажатию кнопки ">"
			 * выбираем параметр для редактирования
			 * индексируем один раз, чтобы была возможность снять выделение с параметра
			 */
			sel_work_param++;
			if (sel_work_param > 1) {sel_work_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		if ((BUT_press == BUT_LEFT))
		{

			//авто включение подсветки дисплея
			LED_auto_on();

			/* изменение активного параметра */

			if ((flag_ptt_on == 1)&&(Settings_PA.SET_CHANGE_POWER == ON)) //ПЕРЕДЕЛАТЬ!
			{
				/* изменяем значение мощности (управляемый аттенюатор на входе усилителя)
				 * только в режиме включенной передачи и если в настройках разрешено ручное управление
				 * меняем значение затухания аттенюатора если разрешено изменение выходной мощности
				 */
				PE4302_change_att(0);
			}


			if ((sel_work_param == 1)&&(flag_ptt_on != 1)&&(Settings_PA.SET_MAN_CONTROL_LPF != ON))
			{
				//только в режиме отключенной передачи
				//инкремент индекса диапазона
				LPF_active++;
				//закольцовываем смену даиапазона
				if (LPF_active > 8) {LPF_active = 0;}
				//переключаем активный диапазон ФНЧ
				LPF_change_band();
			}

			BUT_press = NO_BUT;
			return;
		}

		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			//авто включение подсветки дисплея
			LED_auto_on();

			//переходим на первую страницу настроек усилителя
			PROG_state = PROG_STATE_SETUP_1;
			BUT_press = NO_BUT;
			return;
		}

		//авария могла возникнуть во время измерений - экран аварии уже выведен,
		//текущий кадр не обновляем, чтобы не затереть сообщение
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//рисуем рамку
		LCD_Draw_Rectangle(0, 0, LCD_Width - 1, LCD_Height - 1, pxView_On);
		//обновляем дисплей
		LCD_Update();
	}
}

/* функция цикла программы - настройки первая страница */
void PROG_mode_setup_1()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(0, 0, "НАСТРОЙКИ - 1\n", 0, ChUnInvers);
	//горизонтальные разделительные линии
	LCD_Draw_Line(0, 11, LCD_Width - 1, 11, pxView_On);
	LCD_Draw_Line(0, LCD_Height - 1, LCD_Width - 1, LCD_Height - 1, pxView_On);
	/* выводим меню */
	SETUP_1_move(sel_param);

	/* цикл процесса меню настроек*/
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		if(force_work)
		{
			force_work = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			return;
		}

		//обработка аварийной ситуации
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		//авария могла возникнуть при опросе датчиков
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//на передаче меню закрываем - в рабочем режиме работают все защиты
		if(flag_ptt_on == 1)
		{
			PROG_state = PROG_STATE_WORK;
			return;
		}

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		//чтение состояния кнопок
		READ_button();

		//прорисовка пунктов настроек
		SETUP_1_move(sel_param);

		/* перемещение по пунктам меню настроек */
		if (BUT_press == BUT_RIGHT)
		{
			sel_param++;
			if (sel_param > 3) {sel_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		/* изменение значения активного параметра меню настроек */
		if (BUT_press == BUT_LEFT)
		{
			//изменение параметра настроек
			SETUP_1_change_param(sel_param);
			BUT_press = NO_BUT;
			return;
		}

		/* переходим на вторую страницу настроек */
		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			sel_param = 0;
			PROG_state = PROG_STATE_SETUP_2;
			BUT_press = NO_BUT;
			return;
		}
	}
}

/* функция цикла программы - настройки вторая страница */
void PROG_mode_setup_2()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(0, 0, "НАСТРОЙКИ - 2\n", 0, ChUnInvers);
	//горизонтальные разделительные линии
	LCD_Draw_Line(0, 11, LCD_Width - 1, 11, pxView_On);
	LCD_Draw_Line(0, LCD_Height - 1, LCD_Width - 1, LCD_Height - 1, pxView_On);
	/* выводим меню */
	SETUP_2_move(sel_param);

	/* цикл процесса меню настроек*/
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		if(force_work)
		{
			force_work = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			return;
		}

		//обработка аварийной ситуации
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		//авария могла возникнуть при опросе датчиков
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//на передаче меню закрываем - в рабочем режиме работают все защиты
		if(flag_ptt_on == 1)
		{
			PROG_state = PROG_STATE_WORK;
			return;
		}

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		//чтение состояния кнопок
		READ_button();

		//прорисовка пунктов настроек
		SETUP_2_move(sel_param);

		/* перемещение по пунктам меню настроек */
		if (BUT_press == BUT_RIGHT)
		{
			sel_param++;
			if (sel_param > 3) {sel_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		/* изменение значения активного параметра меню настроек */
		if (BUT_press == BUT_LEFT)
		{
			//изменение параметра настроек
			SETUP_2_change_param(sel_param);
			BUT_press = NO_BUT;
			return;
		}

		/* переходим на третью страницу настроек */
		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			sel_param = 0;
			PROG_state = PROG_STATE_SETUP_3;
			BUT_press = NO_BUT;
			return;
		}
	}
}

/* функция цикла программы - настройки третья страница */
void PROG_mode_setup_3()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(0, 0, "НАСТРОЙКИ - 3\n", 0, ChUnInvers);
	//горизонтальные разделительные линии
	LCD_Draw_Line(0, 11, LCD_Width - 1, 11, pxView_On);
	LCD_Draw_Line(0, LCD_Height - 1, LCD_Width - 1, LCD_Height - 1, pxView_On);
	/* выводим меню */
	SETUP_3_move(sel_param);

	/* цикл процесса меню настроек*/
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		if(force_work)
		{
			force_work = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			return;
		}

		//обработка аварийной ситуации
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		//авария могла возникнуть при опросе датчиков
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//на передаче меню закрываем - в рабочем режиме работают все защиты
		if(flag_ptt_on == 1)
		{
			PROG_state = PROG_STATE_WORK;
			return;
		}

		//устанавливаем текущее затухание АТТ
		PE4302_set_att();

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		//чтение состояния кнопок
		READ_button();

		//прорисовка пунктов настроек
		SETUP_3_move(sel_param);

		/* перемещение по пунктам меню настроек */
		if (BUT_press == BUT_RIGHT)
		{
			sel_param++;
			if (sel_param > 3) {sel_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		/* изменение значения активного параметра меню настроек */
		if (BUT_press == BUT_LEFT)
		{
			//изменение параметра настроек
			SETUP_3_change_param(sel_param);
			BUT_press = NO_BUT;
			return;
		}

		/* переходим в рабочий режим */
		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			sel_param = 0;
			PROG_state = PROG_STATE_SETUP_4;
			BUT_press = NO_BUT;
			return;
		}
	}
}

/* функция цикла программы - настройки четвертая страница */
void PROG_mode_setup_4()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(0, 0, "НАСТРОЙКИ - 4\n", 0, ChUnInvers);
	//горизонтальные разделительные линии
	LCD_Draw_Line(0, 11, LCD_Width - 1, 11, pxView_On);
	LCD_Draw_Line(0, LCD_Height - 1, LCD_Width - 1, LCD_Height - 1, pxView_On);
	/* выводим меню */
	SETUP_4_move(sel_param);

	/* цикл процесса меню настроек*/
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		if(force_work)
		{
			force_work = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			return;
		}

		//обработка аварийной ситуации
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		//авария могла возникнуть при опросе датчиков
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//на передаче меню закрываем - в рабочем режиме работают все защиты
		if(flag_ptt_on == 1)
		{
			PROG_state = PROG_STATE_WORK;
			return;
		}

		//устанавливаем текущее затухание АТТ
		PE4302_set_att();

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		//чтение состояния кнопок
		READ_button();

		//прорисовка пунктов настроек
		SETUP_4_move(sel_param);

		//устанавливаем указанное в настройках затухание аттенюатора для активного диапазона
		att_data = bnd_in_att_data[LPF_active];
		PE4302_change_att(1);

		/* перемещение по пунктам меню настроек */
		if (BUT_press == BUT_RIGHT)
		{
			sel_param++;
			if (sel_param > 3) {sel_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		/* изменение значения активного параметра меню настроек */
		if (BUT_press == BUT_LEFT)
		{
			//изменение параметра настроек
			SETUP_4_change_param(sel_param);
			BUT_press = NO_BUT;
			return;
		}

		/* переходим в рабочий режим */
		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			sel_param = 0;
			PROG_state = PROG_STATE_SETUP_5;
			BUT_press = NO_BUT;
			return;
		}
	}
}

/* функция цикла программы - настройки пятая страница */
void PROG_mode_setup_5()
{
	/* готовим дисплей */
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(0, 0, "НАСТРОЙКИ - 5\n", 0, ChUnInvers);
	//горизонтальные разделительные линии
	LCD_Draw_Line(0, 11, LCD_Width - 1, 11, pxView_On);
	LCD_Draw_Line(0, LCD_Height - 1, LCD_Width - 1, LCD_Height - 1, pxView_On);
	/* выводим меню */
	SETUP_5_move(sel_param);

	/* цикл процесса меню настроек*/
	while(1)
	{
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();

		if(force_work)
		{
			force_work = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			sel_param = 0;
			return;
		}

		//обработка аварийной ситуации
		if(pa_fault != PA_FAULT_NONE)
		{
			if(PROTECTION_recover() == 1)
			{
				PROG_state = PROG_STATE_WORK;
				return;
			}
			continue;
		}

		//обработка изменения состояния PTT
		PTT_process();

		//периодический опрос датчиков на шине I2C
		SENSOR_poll();

		//авария могла возникнуть при опросе датчиков
		if(pa_fault != PA_FAULT_NONE)
		{
			continue;
		}

		//на передаче меню закрываем - в рабочем режиме работают все защиты
		if(flag_ptt_on == 1)
		{
			PROG_state = PROG_STATE_WORK;
			return;
		}

		//устанавливаем текущее затухание АТТ
		PE4302_set_att();

		if((!flag_ptt_on)&&(Settings_PA.SET_MAN_CONTROL_LPF != OFF))
		{
			//читаем внешние сигналы ABCD, если включен соответствующий параметр настроек
			LPF_external_command();
		}

		if(end_adc_flag == 1)
		{
			//старт конверсии АЦП-1
			ADC_1_start_conversion();
		}

		//чтение состояния кнопок
		READ_button();

		//прорисовка пунктов настроек
		SETUP_5_move(sel_param);

		/* перемещение по пунктам меню настроек */
		if (BUT_press == BUT_RIGHT)
		{
			sel_param++;
			if (sel_param > 1) {sel_param = 0;}
			BUT_press = NO_BUT;
			return;
		}

		/* изменение значения активного параметра меню настроек */
		if (BUT_press == BUT_LEFT)
		{
			//изменение параметра настроек
			SETUP_5_change_param(sel_param);
			BUT_press = NO_BUT;
			return;
		}

		/* переходим в рабочий режим */
		if ((BUT_press == BUT_FUNCTION)&&(flag_ptt_on != 1))
		{
			sel_param = 0;
			PROG_state = PROG_STATE_WORK;
			BUT_press = NO_BUT;
			return;
		}
	}
}

/*-----------------------------------------------------------------*/

/* функция для задержки сброса по IWDG */
void Wait_IWDG()
{
	uint8_t cnt_iwdg = 0;

	while(cnt_iwdg <= 10)
	{
		cnt_iwdg++;
		HAL_Delay(500);
		//сброс IWDG
		HAL_IWDG_Refresh(&hiwdg);

		//протокол по USB CDC
		PA_Protocol_Poll();
	}
	//уходим в перезагрузку по iwdg
	//крутим бесконечный цикл
	while(1){}
}

/* функция вывода заставки */
void Splash()
{
	//сброс IWDG
	HAL_IWDG_Refresh(&hiwdg);
	//вывод заставки
	LCD_Output_String_Center(0, _PROGRAM_DEF, 0, ChUnInvers);
	LCD_Output_String_Center(25, _RELEASE_DEF, 0, ChUnInvers);
	LCD_Output_String_Center(53, _COPYRIGHT_DEF, 0, ChUnInvers);
	LCD_Update();
	//длительность заставки
	PA_Protocol_Poll();
	HAL_Delay(1000);
	//сброс IWDG
	HAL_IWDG_Refresh(&hiwdg);
	//еще секунду держим заставку
	PA_Protocol_Poll();
	HAL_Delay(1000);
	LCD_Clean_Buffer_Frame();
	LCD_Update();
}

/*-Конец кода------------------------------------------------------*/
