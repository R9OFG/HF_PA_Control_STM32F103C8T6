/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

#ifndef PA_PROTOCOL_H
#define PA_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Приём байт из USB CDC (вызывается из прерывания USB). */
void PA_Protocol_Push(const uint8_t *data, uint32_t len);

/* Обработка накопленных команд и потока телеметрии (вызывается из основного цикла). */
void PA_Protocol_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* PA_PROTOCOL_H */
