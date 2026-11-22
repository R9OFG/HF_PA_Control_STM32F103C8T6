/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include <stdbool.h>

#include "gmg12864_r9ofg.h"
#include "_program_mode.h"
#include "ina226.h"
#include "lm75a.h"
#include "eeprom.h"
#include "usb_device.h"
#include "pa_protocol.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define START_ADC_PERIOD_MS 			1						//период в миллисекундах для запуска новых преобразований АЦП-1, подбирал по наилучшему результату
#define CALC_ADC_PERIOD_MS				25						//период в миллисекундах после которого будем рассчитывать среднее значение данных АЦП-1, подбирал по наилучшему результату
#define MEDIAN_NUM_READ					10						//порядок медианы для фильтра данных от АЦП-1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint8_t  wwdg_flag						= 0;					//флаг перезагрузки МК по IWDG

uint8_t  end_adc_flag					= 1;					//флаг завершения преобразования АЦП-1
uint8_t  end_adc_filter_flag 			= 0;					//флаг готовности обработанных данных от АЦП-1
uint32_t start_adc_cnt 					= START_ADC_PERIOD_MS;	//вспомогательный счетчик для запуска периодических операций усреднения данных АЦП-1
uint32_t calc_adc_cnt 					= CALC_ADC_PERIOD_MS;	//вспомогательный счетчик для запуска периодических операций усреднения данных АЦП-1
uint32_t adc_samples_cnt				= 0;					//счетчик сэмплов для усреднения данных от АЦП-1

volatile uint16_t adc_values[ADC_CHANNELS_NUM]	= {0,};					//массив значений с каналов АЦП-1 (обновляет DMA)
uint32_t adc_sum[ADC_CHANNELS_NUM-1]	= {0,};					//массив значений с каналов АЦП-1 для суммирования
uint16_t adc_out[ADC_CHANNELS_NUM-1]	= {0,};					//массив значений с каналов АЦП-1 для усредненных значений

volatile uint8_t flag_ptt_change		= 0;					//флаг изменения состояния сигнала PTT (обрабатывается в основном цикле)

extern PROG_STATE 		PROG_state;								//текущий режим работы программы

extern BAND_INDEX       LPF_active;								//актитвный ФНЧ по умолчанию (в соответствии с массивом LPF_band_string)

extern struct SETUP_PA  Settings_PA;							//структура настроек контроллера УМ

extern volatile uint8_t flag_ptt_on;							//флаг внешнего сигнала PTT

extern volatile uint8_t att_data;								//управляющие биты аттенюатора PE4302
extern volatile uint8_t band_att_data[9];						//затухание аттенюатора PE4302 по диапазонам

extern uint8_t flag_ina226;										//флаг датчика INA226
extern uint8_t flag_lm75a;										//флаг датчика LM75A

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* callback чтения внешнего cигнала PTT */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t ptt_state = 0;

	//проверяем callback от двух пинов PTT_ON_Pin или PTT_GND_Pin
	if((GPIO_Pin != PTT_ON_Pin)&&(GPIO_Pin != PTT_GND_Pin))
	{
		return;
	}

	//сигнал PTT активен, если на любом из входов лог. 1
	if(HAL_GPIO_ReadPin(PTT_ON_GPIO_Port, PTT_ON_Pin)) {ptt_state = 1;}
	if(HAL_GPIO_ReadPin(PTT_GND_GPIO_Port, PTT_GND_Pin)) {ptt_state = 1;}

	//индикация на BluePill: на передаче светодиод погашен, на приеме - горит
	if(ptt_state) {LED_PTT_reset();} else {LED_PTT_set();}

	//в прерывании только фиксируем смену состояния,
	//вся остальная обработка (аттенюатор, подсветка) выполняется в основном цикле
	flag_ptt_on = ptt_state;
	flag_ptt_change = 1;
}

/* функция медианного фильтра */
uint16_t findMedianN_optim_mod(uint16_t adc_channel, uint16_t newVal)
{
	/*
	 * ©Виталий Емельянов mod AlexGyver
	 * https://alexgyver.ru/lessons/filters/
	 *
	 * mod R9OFG.RU - буфер значений изменил на двумерный массив,
	 * т.к. мне нужна фильтрация данных с трех каналов АЦП-1
	 *
	 * Счетчик заполнения у каждого канала свой: раньше он был общим,
	 * из-за чего каждый канал заполнял только половину буфера,
	 * а медиана считалась по смеси свежих и устаревших значений.
	 */

	static uint16_t buffer[ADC_CHANNELS_NUM-1][MEDIAN_NUM_READ];	//статический двумерный буфер
	static uint16_t count[ADC_CHANNELS_NUM-1] = {0,};				//счетчик заполнения для каждого канала

	uint16_t *buf = buffer[adc_channel];
	uint16_t *cnt = &count[adc_channel];

	buf[*cnt] = newVal;

	if((*cnt < MEDIAN_NUM_READ - 1)&&(buf[*cnt] > buf[*cnt + 1]))
	{
		for (int i = *cnt; i < MEDIAN_NUM_READ - 1; i++)
		{
	    	if (buf[i] > buf[i + 1])
	    	{
	    		uint16_t buff = buf[i];
	    		buf[i] = buf[i + 1];
	    		buf[i + 1] = buff;
	    	}
	    }
	}
	else
	{
		if((*cnt > 0)&&(buf[*cnt - 1] > buf[*cnt]))
		{
	    	for (int i = *cnt; i > 0; i--)
	    	{
	    		if (buf[i] < buf[i - 1])
	    		{
	    			uint16_t buff = buf[i];
	    			buf[i] = buf[i - 1];
	    			buf[i - 1] = buff;
	    		}
	    	}
	    }
	}

	if (++(*cnt) >= MEDIAN_NUM_READ) *cnt = 0;

	return buf[(int)MEDIAN_NUM_READ / 2];
}

/* функция старта преобразования от АЦП-1 */
void ADC_1_start_conversion(void)
{
	/*
	 * ©MicroTechnics
	 * https://microtechnics.ru/filtracziya-i-usrednenie-dannyh-aczp/
	 *
	 * Как видите, мы записываем в переменные startAdcCnt и calcAdcCnt величину периода,
	 * и, как только счетчики досчитали до нуля, выполняем необходимые операции.
	 * Весь вопрос в том, где мы будем уменьшать эти счетчики. И поскольку у нас в программе уже есть обработчик прерывания SysTick_Handler,
	 * который вызывается каждую миллисекунду, то в нем и будем работать. Но перед этим объявим переменные в файле stm32f1xx_it.c:
	 *
	 * -USER CODE BEGIN EV-
	 * extern uint32_t startAdcCnt;
	 * extern uint32_t calcAdcCnt;
	 * -USER CODE END EV-
	 *
	 * -USER CODE BEGIN SysTick_IRQn 1-
	 * if (startAdcCnt > 0)
	 * {
	 * 	  startAdcCnt--;
	 * }
	 *
	 * if (calcAdcCnt > 0)
	 * {
	 *    calcAdcCnt--;
	 * }
	 * -USER CODE END SysTick_IRQn 1-
	 */

	if(start_adc_cnt == 0)
    {
		//АЦП и DMA работают непрерывно; здесь только накапливаем
		//отсчёты из циклического буфера: PA2 - REF, PA3 - FOR
		start_adc_cnt = START_ADC_PERIOD_MS;
		adc_sum[0] += adc_values[1];
		adc_sum[1] += adc_values[2];
		adc_samples_cnt++;
    }

    if (calc_adc_cnt == 0)
    {
    	//усредняем только при наличии новых отсчетов (защита от деления на ноль)
    	if(adc_samples_cnt > 0)
    	{
    		//старт нового усреднения данных АЦП-1
    		for(uint8_t i = 0; i < ADC_CHANNELS_NUM-1; i++)
			{
    			adc_out[i] = findMedianN_optim_mod(i, (uint16_t)(adc_sum[i] / adc_samples_cnt));
				adc_sum[i] = 0;
			}

    		adc_samples_cnt = 0;
    	}

    	calc_adc_cnt = CALC_ADC_PERIOD_MS;
    	end_adc_filter_flag = 1;
    }
}

/* DMA работает в циклическом режиме, TC-прерывание отключено */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	(void)hadc;
}

/* Ошибка DMA/ADC: перезапускаем циклический буфер */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(&hadc1);

		if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_CHANNELS_NUM) == HAL_OK)
		{
			__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  //инициализация USB CDC (виртуальный COM-порт)
  MX_USB_DEVICE_Init();

#ifdef USB_ONLY_TEST
  /*
   * Тестовая прошивка: запускается только USB CDC.
   * Мигание LED_PTT (PC13) показывает, что ядро живо.
   * Вся остальная логика (EEPROM, I2C, дисплей, датчики) не запускается.
   */
  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_PTT_GPIO_Port, LED_PTT_Pin);
    HAL_Delay(250);
  }
#endif

  //выключаем вентилятор
  FAN_reset();

  //инициализируем эмулятор EEPROM
  EEPROM_Init();

  //калибровка АЦП-1
  HAL_ADCEx_Calibration_Start(&hadc1);

  //ADC + DMA непрерывно, в циклическом режиме:
  //порядок каналов в буфере всегда PA1, PA2, PA3
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_CHANNELS_NUM) != HAL_OK)
  {
      Error_Handler();
  }

  //TC-прерывание на каждый проход DMA не нужно, буфер читаем по SysTick
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);

  //сброс внешнего сигнала PTT
  HAL_GPIO_EXTI_Callback(PTT_ON_Pin);
  HAL_GPIO_EXTI_Callback(PTT_GND_Pin);

  //инициализация датчика INA226 (AVG_16, время преобразования 1.1 мс:
  //обновление показаний примерно каждые 35 мс - быстрее реакция защиты)
  INA226_Init(AVG_16, T_Vbus_1_1ms, T_Vshunt_1_1ms, MODE_SHUNT_BUS_CONTINUOUS);

  //чтение настроек
  READ_setup_pa();

  //устанавливаем секцию ФНЧ по умолчанию
  LPF_active = _10;
  LPF_change_band();

  //применяем состояние сигнала PTT (аттенюатор, подсветка)
  PTT_process();

  //инициализация дисплея
  LCD_Init();

  //проверка был ли ресет по IWDG
  if (READ_BIT(RCC->CSR, RCC_CSR_IWDGRSTF) != RESET)
  {
  	SET_BIT(RCC->CSR, RCC_CSR_RMVF);

  	wwdg_flag = 1;

  	LCD_Output_String(45, 25, "АВАРИЯ\n", 0, ChUnInvers);
  	LCD_Update();
  	HAL_Delay(1000);
  }

  //сброс IWDG
  HAL_IWDG_Refresh(&hiwdg);

  if(wwdg_flag == 0)
  {
	  LCD_Output_String(25, 25, "ДИАГНОСТИКА\n", 0, ChUnInvers);
	  LCD_Update();
	  HAL_Delay(1000);
  }

  //сброс IWDG
  HAL_IWDG_Refresh(&hiwdg);

  HAL_Delay(1000);

  //сброс IWDG
  HAL_IWDG_Refresh(&hiwdg);

  //проверяем датчики на шине I2C.
  //Раньше при отсутствии датчика МК уходил в бесконечную перезагрузку.
  //Теперь просто ждем появления датчика и не подаем питание на усилитель.
  SEARCH_sensor_on_i2c_bus();
  FAN_fail_safe();

  while((flag_ina226 != 1)||(flag_lm75a != 1))
  {
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(15, 20, "ОШИБКА ДАТЧИКА\n", 0, ChUnInvers);

	if(flag_ina226 != 1)
	{
		LCD_Output_String(45, (flag_lm75a != 1) ? 30 : 34, "INA226\n", 0, ChUnInvers);
	}

	if(flag_lm75a != 1)
	{
		LCD_Output_String(48, (flag_ina226 != 1) ? 42 : 34, "LM75A\n", 0, ChUnInvers);
	}

	LCD_Update();

	//держим сообщение и продолжаем опрос шины
	HAL_IWDG_Refresh(&hiwdg);

	//протокол по USB CDC (приложение должно отвечать и без датчиков)
	PA_Protocol_Poll();

	HAL_Delay(500);
	SEARCH_sensor_on_i2c_bus();
	FAN_fail_safe();
  }

  //************

  //подаем питание на усилитель и проверяем напряжение питания.
  //INA226 измеряет напряжение ПОСЛЕ ключа CONTROL_U, поэтому проверка
  //выполняется импульсами: подали питание - измерили - сняли при аварии.
  while(1)
  {
	//протокол по USB CDC
	PA_Protocol_Poll();

	CONTROL_U_set();

	//ждем стабилизации напряжения питания
	HAL_Delay(FAULT_TEST_PWR_MS);

	//сброс IWDG
	HAL_IWDG_Refresh(&hiwdg);

	float valueU = INA226_Vbus();

	if((valueU >= Settings_PA.SET_MIN_U)&&(valueU <= Settings_PA.SET_MAX_U))
	{
		//напряжение в норме
		break;
	}

	//отключаем питание на усилитель
	CONTROL_U_reset();

	//выводим предупреждение
	LCD_Clean_Buffer_Frame();
	LCD_Output_String(30, 20, "НАПРЯЖЕНИЕ\n", 0, ChUnInvers);
	LCD_Output_String(32, 34, "НЕ В НОРМЕ\n", 0, ChUnInvers);
	LCD_Update();

	//ждем перед следующей попыткой
	DELAY_iwdg(FAULT_TEST_MS - FAULT_TEST_PWR_MS);
  }

  LCD_Clean_Buffer_Frame();
  LCD_Update();


  if(wwdg_flag == 0)
  {
	  //выводим заставку
	  Splash();
  }
  else
  {
	  //сбрасываем флаг перезагрузки по IWDG
	  wwdg_flag = 0;
  }

  //сброс IWDG
  HAL_IWDG_Refresh(&hiwdg);

  //выбираем режим работы программы
  PROG_state = PROG_STATE_WORK;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//протокол по USB CDC
	PA_Protocol_Poll();

	//перебираем режимы работы
	PROG_mode();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PTT_GPIO_Port, LED_PTT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CONTROL_U_Pin|PE_V1_Pin|PE_V2_Pin|PE_V3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, A_CODE_Pin|B_CODE_Pin|LCD_CS_Pin|LCD_RST_Pin
                          |LCD_DC_Pin|LED_DISP_Pin|C_CODE_Pin|D_CODE_Pin
                          |PE_V4_Pin|PE_V5_Pin|PE_V6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PTT_Pin */
  GPIO_InitStruct.Pin = LED_PTT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PTT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONTROL_U_Pin FAN_Pin PE_V1_Pin PE_V2_Pin
                           PE_V3_Pin */
  GPIO_InitStruct.Pin = CONTROL_U_Pin|FAN_Pin|PE_V1_Pin|PE_V2_Pin
                          |PE_V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PTT_GND_Pin PTT_ON_Pin */
  GPIO_InitStruct.Pin = PTT_GND_Pin|PTT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_CODE_Pin B_CODE_Pin LCD_CS_Pin LCD_RST_Pin
                           LCD_DC_Pin LED_DISP_Pin C_CODE_Pin D_CODE_Pin
                           PE_V4_Pin PE_V5_Pin PE_V6_Pin */
  GPIO_InitStruct.Pin = A_CODE_Pin|B_CODE_Pin|LCD_CS_Pin|LCD_RST_Pin
                          |LCD_DC_Pin|LED_DISP_Pin|C_CODE_Pin|D_CODE_Pin
                          |PE_V4_Pin|PE_V5_Pin|PE_V6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A_EXT_Pin(PB2) C_EXT_Pin(PB14) D_EXT_Pin(PB15) */
  GPIO_InitStruct.Pin = A_EXT_Pin|C_EXT_Pin|D_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B_EXT_Pin (PC14) */
  GPIO_InitStruct.Pin = B_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
