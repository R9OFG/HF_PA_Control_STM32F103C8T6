/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * _program_mode.h
 *
 *  Created on: Sep 29, 2023
 *  Author: r9ofg.ru
 */

#ifndef INC__PROGRAM_MODE_H_
#define INC__PROGRAM_MODE_H_

/* инклюды */

#include <main.h>

/* дефайны */

/*
 * Версионность.
 * Версия на заставке собирается из PA_FW_VERSION_STRING,
 * этап разработки (dev/beta/release) на дисплей не выводится.
 */
#define PA_FW_VERSION_MAJOR		2		//старшая цифра версии прошивки
#define PA_FW_VERSION_MINOR		1		//младшая цифра версии прошивки
#define PA_FW_VERSION_STAGE		"dev"	//этап разработки: dev / beta / release
#define PA_FW_VERSION_STRING	"2.1"	//версия для вывода на дисплей

#define _PROGRAM_DEF		"КВ усилитель мощности\n"
#define _RELEASE_DEF		"Версия: " PA_FW_VERSION_STRING "\n"
#define _COPYRIGHT_DEF		"R9OFG.RU\n"

#define DEF_MIN_U			10	//минимальный порог напряжения питания, В, по умолчанию
#define DEF_MAX_U			15	//максимальный порог напряжения питания, В, по умолчанию
#define DEF_MIN_I			5	//минимальный порог тока потребления, А
#define DEF_MAX_I			25	//максимальный порог тока потребления, А, по умолчанию
#define DEF_MAX_SWR			7	//максимальный порог КСВ, по умолчанию
#define MIN_SWR				3 	//минимальный порог КСВ
#define MAX_SWR				9   //максимальный порог КСВ
#define DEF_T				45	//температура включения охлаждения, по умолчанию
#define MIN_T				30	//миимальнаое значение температуры включения охлаждения при регулировке
#define MAX_T				50	//максимальное значение температуры включения охлаждения при регулировке
#define DEF_MAX_TEMP		75	//аварийное отключение усилителя при перегреве, °C
#define TEMP_G				2	//гистерезис
#define LOAD_Z				50	//сопротивление нагрузки(антенны/эквивалента)
#define LED_AUTO_OFF_MS		30000	//время бездействия до автоматического отключения подсветки дисплея, мс

/* периоды и пороги системы защиты */
#define U_FAULT_CNT			4		//подтверждение аварии по напряжению (4 * 100 мс = 400 мс)
#define I_FAULT_CNT			2		//подтверждение аварии по току (2 * 100 мс = 200 мс)
#define SWR_GRACE_MS		5000	//время на работу автотюнера при превышении КСВ, мс
#define FAULT_TEST_MS		5000	//период повторной проверки питания после аварии, мс
#define FAULT_TEST_PWR_MS	300		//длительность импульса питания при проверке, мс
#define FAULT_OK_MS			2000	//время "все в норме" для снятия аварии, мс
#define SENSOR_POLL_MS		500		//период опроса датчиков на шине I2C, мс
#define SENSOR_FAULT_CNT	6		//подтверждение потери датчика (6 * 500 мс = 3 с)
#define MEASURE_PERIOD_MS	100		//период обновления показаний U/I/T на дисплее, мс

/* сглаживание показаний мощности и КСВ */
#define PWR_FILTER_ATTACK	0.45f	//коэффициент сглаживания мощности при росте (за цикл 25 мс)
#define PWR_FILTER_DECAY	0.04f	//коэффициент сглаживания мощности при спаде
#define SWR_FILTER_KOEF		0.10f	//коэффициент сглаживания показаний КСВ
#define SWR_MIN_POWER		1.0f	//минимальная падающая мощность для расчета КСВ, Вт
#define UI_FILTER_KOEF		0.25f	//коэффициент сглаживания показаний напряжения и тока
#define PWR_LCD_FILTER_KOEF	0.06f	//дополнительное сглаживание мощности на дисплее (цикл 25 мс)
#define UI_LCD_FILTER_KOEF	0.12f	//дополнительное сглаживание напряжения и тока на дисплее (цикл 100 мс)
#define PWR_LCD_DEADBAND	0.2f	//порог изменения показанной мощности, Вт
#define UI_LCD_DEADBAND		0.02f	//порог изменения показанных напряжения/тока

#define LED_DISP_reset()	LED_DISP_GPIO_Port->BSRR = (uint32_t)LED_DISP_Pin<<16u; 	//устанавливаем лог. 0
#define LED_DISP_set()		LED_DISP_GPIO_Port->BSRR = LED_DISP_Pin;			    	//устанавливаем лог. 1

#define LED_PTT_reset()		LED_PTT_GPIO_Port->BSRR = (uint32_t)LED_PTT_Pin<<16u; 		//устанавливаем лог. 0
#define LED_PTT_set()		LED_PTT_GPIO_Port->BSRR = LED_PTT_Pin;				    	//устанавливаем лог. 1

#define CONTROL_U_reset()	CONTROL_U_GPIO_Port->BSRR = (uint32_t)CONTROL_U_Pin<<16u;	//устанавливаем лог. 0
#define CONTROL_U_set()		CONTROL_U_GPIO_Port->BSRR = CONTROL_U_Pin;			    	//устанавливаем лог. 1

#define FAN_reset()			FAN_GPIO_Port->BSRR = (uint32_t)FAN_Pin<<16u;				//устанавливаем лог. 0
#define FAN_set()			FAN_GPIO_Port->BSRR = FAN_Pin;			    				//устанавливаем лог. 1

#define A_reset() 			A_CODE_GPIO_Port->BSRR = (uint32_t)A_CODE_Pin<<16u; 		//устанавливаем лог. 0
#define A_set()   			A_CODE_GPIO_Port->BSRR = A_CODE_Pin;				  		//устанавливаем лог. 1
#define B_reset() 			B_CODE_GPIO_Port->BSRR = (uint32_t)B_CODE_Pin<<16u; 		//устанавливаем лог. 0
#define B_set()   			B_CODE_GPIO_Port->BSRR = B_CODE_Pin;				  		//устанавливаем лог. 1
#define C_reset() 			C_CODE_GPIO_Port->BSRR = (uint32_t)C_CODE_Pin<<16u; 		//устанавливаем лог. 0
#define C_set()   			C_CODE_GPIO_Port->BSRR = C_CODE_Pin;						//устанавливаем лог. 1
#define D_reset() 			D_CODE_GPIO_Port->BSRR = (uint32_t)D_CODE_Pin<<16u; 		//устанавливаем лог. 0
#define D_set()   			D_CODE_GPIO_Port->BSRR = D_CODE_Pin;				  		//устанавливаем лог. 1

#define PE_V1_reset()		PE_V1_GPIO_Port->BSRR = (uint32_t)PE_V1_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V1_set()			PE_V1_GPIO_Port->BSRR = PE_V1_Pin;				  			//устанавливаем лог. 1
#define PE_V2_reset()		PE_V2_GPIO_Port->BSRR = (uint32_t)PE_V2_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V2_set()			PE_V2_GPIO_Port->BSRR = PE_V2_Pin;				  			//устанавливаем лог. 1
#define PE_V3_reset()		PE_V3_GPIO_Port->BSRR = (uint32_t)PE_V3_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V3_set()			PE_V3_GPIO_Port->BSRR = PE_V3_Pin;				  			//устанавливаем лог. 1
#define PE_V4_reset()		PE_V4_GPIO_Port->BSRR = (uint32_t)PE_V4_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V4_set()			PE_V4_GPIO_Port->BSRR = PE_V4_Pin;				  			//устанавливаем лог. 1
#define PE_V5_reset()		PE_V5_GPIO_Port->BSRR = (uint32_t)PE_V5_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V5_set()			PE_V5_GPIO_Port->BSRR = PE_V5_Pin;				  			//устанавливаем лог. 1
#define PE_V6_reset()		PE_V6_GPIO_Port->BSRR = (uint32_t)PE_V6_Pin<<16u; 			//устанавливаем лог. 0
#define PE_V6_set()			PE_V6_GPIO_Port->BSRR = PE_V6_Pin;				  			//устанавливаем лог. 1

/* перечисления */

typedef enum {
	PROG_STATE_WORK	 		= 0, //режим работы
	PROG_STATE_SETUP_1		= 1, //режим настроек первая страницв
	PROG_STATE_SETUP_2		= 2, //режим настроек вторая страница
	PROG_STATE_SETUP_3		= 3, //режим настроек третья страница
	PROG_STATE_SETUP_4		= 4, //режим настроек четвертая страница
	PROG_STATE_SETUP_5		= 5  //режим настроек пятая страница
} PROG_STATE;

typedef enum {
	NO_BUT					= 0, //ни одна кнопка не нажата
	BUT_FUNCTION			= 1, //функциональная кнопка
	BUT_LEFT 				= 2, //кнопка влево
	BUT_RIGHT				= 3  //кнопка вправо
} BUTTON_PRESS;

typedef enum {
	OFF						= 0, //параметр отключен
	ON						= 1, //параметр включен
	AUTO					= 2  //параметр авто
} SETUP_PA_PARAM;

typedef enum {
	_160					= 0, //160m
	_80						= 1, //80m
	_40						= 2, //40m
	_30						= 3, //30m
	_20						= 4, //20m
	_17						= 5, //17m
	_15						= 6, //15m
	_12						= 7, //12m
	_10						= 8  //10m
} BAND_INDEX;

typedef enum {
	PA_FAULT_NONE		= 0, //аварии нет
	PA_FAULT_VOLTAGE	= 1, //напряжение питания вне нормы (общий код)
	PA_FAULT_CURRENT	= 2, //ток потребления выше нормы
	PA_FAULT_SWR		= 3, //КСВ выше нормы
	PA_FAULT_TEMP		= 4, //перегрев усилителя
	PA_FAULT_SENSOR		= 5, //потеря датчиков на шине I2C (общий код)
	PA_FAULT_UVOLT		= 6, //пониженное напряжение питания
	PA_FAULT_OVOLT		= 7, //повышенное напряжение питания
	PA_FAULT_INA226		= 8, //нет связи с INA226
	PA_FAULT_LM75A		= 9  //нет связи с LM75A
} PA_FAULT;

/* структуры */

struct SETUP_PA {
	/* структура настроек контроллера */
	SETUP_PA_PARAM 			SET_LED_DISPLAY;
	SETUP_PA_PARAM 			SET_CHANGE_POWER;
	SETUP_PA_PARAM 			SET_MAN_CONTROL_LPF;
	uint8_t					SET_TEMP_COOLER;
	uint8_t					SET_MIN_U;
	uint8_t					SET_MAX_U;
	uint8_t					SET_MAX_I;
	uint8_t					SET_MAX_SWR;
	SETUP_PA_PARAM			SET_FAN_NO_LM75A;	//включать вентилятор при отсутствии LM75A
};

/* прототипы функций */
void LED_auto_on();
void SEARCH_sensor_on_i2c_bus();
void FAN_fail_safe(void);
void FAN_test_set(uint8_t on);
void READ_setup_pa();
void LPF_change_band();
void PE4302_change_att(uint8_t param);
void PE4302_set_att();
void DISPLAY_work();
void PROG_mode();
void PROG_mode_work();
void PROG_mode_setup_1();
void PROG_mode_setup_2();
void PROG_mode_setup_3();
void PROG_mode_setup_4();
void PROG_mode_setup_5();

/* обработка PTT, опрос датчиков и система защиты */
void PTT_process(void);
void SENSOR_poll(void);
uint8_t MEASURE_is_due(void);
void DELAY_iwdg(uint32_t delay_ms);
void PROTECTION_fault(PA_FAULT code);
uint8_t PROTECTION_recover(void);
void SETUP_pa_default(void);

void Wait_IWDG();
void Splash();

#endif /* INC__PROGRAM_MODE_H_ */
