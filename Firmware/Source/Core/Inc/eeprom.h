/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Eeprom emulation driver
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */

#ifndef EEPROM_H
#define EEPROM_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/

#define PAGE_DATA_OFFSET	8
#define PAGE_DATA_SIZE      8

#define PARAM_1             0x11111111 //идентификатор ячейки
#define PARAM_2             0x22222222
#define PARAM_3             0x33333333
#define PARAM_4             0x44444444
#define PARAM_5             0x55555555
#define PARAM_6             0x66666666
#define PARAM_7             0x77777777
#define PARAM_8             0x88888888
#define PARAM_9             0x99999999
#define PARAM_10            0xA1111111
#define PARAM_11            0xA2222222
#define PARAM_12            0xA3333333
#define PARAM_13            0xA4444444
#define PARAM_14            0xA5555555
#define PARAM_15            0xA6666666
#define PARAM_16            0xA7777777
#define PARAM_17            0xA8888888
#define PARAM_18            0xA9999999
#define PARAM_FORMAT        0xB1111111 //идентификатор ячейки версии формата настроек
#define VAR_NUM             19

/*
 * Версия формата хранения настроек.
 * 2 - формат 2.0 (введена версионность, добавлен PARAM_FORMAT).
 * При несовпадении версии настройки сбрасываются в значения по умолчанию.
 */
#define EEPROM_FORMAT_VERSION   2

#define PAGE_0_ADDRESS      0x0801F800
#define PAGE_1_ADDRESS      0x0801FC00
#define PAGE_SIZE           1024



typedef enum {
  PAGE_CLEARED 				= 0xFFFFFFFF,
  PAGE_ACTIVE 				= 0x00000000,
  PAGE_RECEIVING_DATA		= 0x55555555,
} PageState;

typedef enum {
  PAGE_0 					= 0,
  PAGE_1 					= 1,
  PAGES_NUM 				= 2,
} PageIdx;

typedef enum {
  EEPROM_OK 				= 0,
  EEPROM_ERROR 				= 1,
} EepromResult;

/* Functions -----------------------------------------------------------------*/

extern EepromResult EEPROM_Init();
extern EepromResult EEPROM_Read(uint32_t varId, uint32_t *varValue);
extern EepromResult EEPROM_Write(uint32_t varId, uint32_t varValue);

#endif // #ifndef EEPROM_H
