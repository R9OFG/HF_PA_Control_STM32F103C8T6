/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * i2c_er.h
 *
 * Created on: Mar 8, 2020
 * Author: https://istarik.ru/blog/stm32/123.html
 */

#ifndef I2C_ER_H_
#define I2C_ER_H_

/* пины I2C1: PB6 - SCL, PB7 - SDA (см. схему контроллера) */
#define SCL_PIN     GPIO_PIN_6
#define SCL_PORT    GPIOB

#define SDA_PIN     GPIO_PIN_7
#define SDA_PORT    GPIOB

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
    {
        if(timeout != HAL_MAX_DELAY) // Check for the timeout
        {
            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
        }

        asm("nop");
    }
    return ret;
}

static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout)
{
	// 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Clear PE bit.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    // 2. Deinit I2C: тактирование I2C1 и выводы при этом выключаются
    HAL_I2C_DeInit(hi2c);

    // 2.1. Сразу возвращаем тактирование и выводы I2C1 (иначе все последующие
    //      обращения к регистрам не работают - периферия остается без clock)
    HAL_I2C_MspInit(hi2c);

    // 3. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin = SCL_PIN;
    HAL_GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDA_PIN;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStructure);

    // 4. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_SET, timeout);

    // 5. Configure the SDA I/O as General Purpose Output Open-Drain, Low level
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);

    // 6. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_RESET, timeout);

    // 7. Configure the SCL I/O as General Purpose Output Open-Drain, Low level
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);

    // 8. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_RESET, timeout);

    // 9. Configure the SCL I/O as General Purpose Output Open-Drain, High level
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);

    // 10. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(SCL_PORT, SCL_PIN, GPIO_PIN_SET, timeout);

    // 11. Configure the SDA I/O as General Purpose Output Open-Drain, High level
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);

    // 12. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(SDA_PORT, SDA_PIN, GPIO_PIN_SET, timeout);

    // 13. Configure the SCL and SDA I/Os as Alternate function Open-Drain
    HAL_I2C_MspInit(hi2c);

    // 14. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    // 15. Clear SWRST bit in I2Cx_CR1 register.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    // 16. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(hi2c);
}

#endif /* I2C_ER_H_ */
