/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * pa_protocol.c
 *
 * Текстовый протокол обмена по USB CDC (см. _Docs/protocol.md).
 * Приём — построчно, команды обрабатываются в контексте основного цикла.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "_program_mode.h"
#include "eeprom.h"
#include "lm75a.h"
#include "Float_transform.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "pa_protocol.h"

/* ------------------------------------------------------------------ внешние данные */

extern float voltDisp;
extern float currDisp;
extern float powerDisp;
extern float swrDisp;
extern float rawFor;
extern float rawRef;
extern float powerFor;
extern float powerRef;
extern float currentPwr;
extern volatile uint16_t adc_values[ADC_CHANNELS_NUM];
extern uint16_t adc_out[ADC_CHANNELS_NUM-1];

extern uint8_t sign_number;
extern int integer_number;
extern uint32_t fractional_number;

extern volatile uint8_t flag_ptt_on;
extern volatile uint8_t flag_ptt_change;
extern volatile uint8_t fan_on;
extern PROG_STATE PROG_state;
extern uint8_t sel_param;
extern BUTTON_PRESS BUT_press;
extern BUTTON_PRESS lastButton;
extern volatile uint8_t force_work;
extern volatile uint8_t att_data;
extern volatile uint8_t band_att_data[9];
extern volatile uint8_t bnd_in_att_data[9];
extern struct SETUP_PA Settings_PA;
extern PA_FAULT pa_fault;
extern BAND_INDEX LPF_active;
extern uint8_t flag_lm75a;
extern uint8_t flag_ina226;

/* не объявлена в _program_mode.h */
extern void LED_display(SETUP_PA_PARAM Value);

/* ------------------------------------------------------------------ константы */

#define PROTO_RING_SIZE     256U
#define PROTO_LINE_SIZE     128U

static const int16_t s_bandNumbers[9] = {160, 80, 40, 30, 20, 17, 15, 12, 10};

static const uint32_t s_attParam[9] =
{
    PARAM_9, PARAM_10, PARAM_11, PARAM_12, PARAM_13, PARAM_14, PARAM_15, PARAM_16, PARAM_17
};

/* ------------------------------------------------------------------ состояние */

static volatile uint16_t s_rxHead;
static volatile uint16_t s_rxTail;
static uint8_t s_rxRing[PROTO_RING_SIZE];

static char s_line[PROTO_LINE_SIZE];
static uint16_t s_lineLength;



/* ------------------------------------------------------------------ утилиты */

static uint8_t StrEqualCI(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        char ca = (*a >= 'a' && *a <= 'z') ? (char)(*a - 'a' + 'A') : *a;
        char cb = (*b >= 'a' && *b <= 'z') ? (char)(*b - 'a' + 'A') : *b;

        if (ca != cb)
        {
            return 0U;
        }

        a++;
        b++;
    }

    return (uint8_t)((*a == '\0') && (*b == '\0'));
}

static void Send(const char *text)
{
    uint16_t length = (uint16_t)strlen(text);

    for (uint8_t attempt = 0U; attempt < 20U; attempt++)
    {
        if (CDC_Transmit_FS((uint8_t *)text, length) == USBD_OK)
        {
            return;
        }

        HAL_Delay(1U);
    }
}

static int8_t BandToIndex(int band)
{
    for (uint8_t i = 0U; i < 9U; i++)
    {
        if (s_bandNumbers[i] == band)
        {
            return (int8_t)i;
        }
    }

    return -1;
}

static uint8_t BandFromIndex(uint8_t index)
{
    return (uint8_t)s_bandNumbers[index < 9U ? index : 8U];
}

static int16_t ReadTemperature(void)
{
    if (flag_lm75a == 1U)
    {
        return LM75_temperature();
    }

    return -127;
}

/* Форматирование float без printf-float (экономия флеша). */
static void FormatFloat(char *out, uint16_t size, float value, uint8_t decimals)
{
    Float_transform(value, decimals, &sign_number, &integer_number, &fractional_number);

    if (decimals == 1U)
    {
        snprintf(out, size, "%s%d.%01lu", (sign_number != 0U) ? "-" : "",
                 integer_number, (unsigned long)fractional_number);
    }
    else
    {
        snprintf(out, size, "%s%d.%02lu", (sign_number != 0U) ? "-" : "",
                 integer_number, (unsigned long)fractional_number);
    }
}

/* ------------------------------------------------------------------ ответы */

static void SendError(const char *code)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "ERR %s\n", code);
    Send(buffer);
}

static void HandleDiag(void)
{
    char buffer[160];
    char textRf[12];
    char textRr[12];
    char textPf[12];
    char textPr[12];
    char textP[12];
    char textSwr[12];

    FormatFloat(textRf, sizeof(textRf), rawFor, 2U);
    FormatFloat(textRr, sizeof(textRr), rawRef, 2U);
    FormatFloat(textPf, sizeof(textPf), powerFor, 1U);
    FormatFloat(textPr, sizeof(textPr), powerRef, 1U);
    FormatFloat(textP, sizeof(textP), powerDisp, 1U);
    FormatFloat(textSwr, sizeof(textSwr), swrDisp, 2U);

    snprintf(buffer, sizeof(buffer),
             "DIAG A1=%u A2=%u A3=%u RF=%s RR=%s PF=%s PR=%s P=%s SWR=%s PTT=%u BAND=%u\n",
             (unsigned)adc_values[0],
             (unsigned)adc_out[0],
             (unsigned)adc_out[1],
             textRf,
             textRr,
             textPf,
             textPr,
             textP,
             textSwr,
             (unsigned)(flag_ptt_on ? 1U : 0U),
             (unsigned)BandFromIndex((uint8_t)LPF_active));

    Send(buffer);
}

static void SendTelemetry(void)
{    char buffer[160];
    char textU[12];
    char textI[12];
    char textP[12];
    char textSwr[12];

    FormatFloat(textU, sizeof(textU), voltDisp, 2U);
    FormatFloat(textI, sizeof(textI), currDisp, 2U);
    FormatFloat(textP, sizeof(textP), powerDisp, 1U);
    FormatFloat(textSwr, sizeof(textSwr), swrDisp, 2U);

    //на передаче — фактическое затухание, на приёме — установленное для активного диапазона
    unsigned attValue = (flag_ptt_on == 1U)
        ? (unsigned)att_data
        : (unsigned)bnd_in_att_data[LPF_active];

    snprintf(buffer, sizeof(buffer),
             "TEL U=%s I=%s T=%d P=%s SWR=%s BAND=%u ATT=%u PTT=%u FAULT=%u INA=%u LM=%u FAN=%u\n",
             textU,
             textI,
             (int)ReadTemperature(),
             textP,
             textSwr,
             (unsigned)BandFromIndex((uint8_t)LPF_active),
             attValue,
             (unsigned)(flag_ptt_on ? 1U : 0U),
             (unsigned)pa_fault,
             (unsigned)flag_ina226,
             (unsigned)flag_lm75a,
             (unsigned)(fan_on ? 1U : 0U));

    Send(buffer);
}

static void SendSettings(void)
{
    char buffer[128];

    snprintf(buffer, sizeof(buffer),
             "SET LED=%u POW=%u LPF=%u TCOOL=%u UMIN=%u UMAX=%u IMAX=%u SWR=%u FANLM=%u\n",
             (unsigned)Settings_PA.SET_LED_DISPLAY,
             (unsigned)Settings_PA.SET_CHANGE_POWER,
             (unsigned)Settings_PA.SET_MAN_CONTROL_LPF,
             (unsigned)Settings_PA.SET_TEMP_COOLER,
             (unsigned)Settings_PA.SET_MIN_U,
             (unsigned)Settings_PA.SET_MAX_U,
             (unsigned)Settings_PA.SET_MAX_I,
             (unsigned)Settings_PA.SET_MAX_SWR,
             (unsigned)Settings_PA.SET_FAN_NO_LM75A);

    Send(buffer);
}

static void SendAttenuation(void)
{
    char buffer[128];
    int offset = snprintf(buffer, sizeof(buffer), "ATT");

    for (uint8_t i = 0U; i < 9U; i++)
    {
        offset += snprintf(buffer + offset, sizeof(buffer) - (size_t)offset,
                           " %u=%u", (unsigned)s_bandNumbers[i], (unsigned)bnd_in_att_data[i]);
    }

    snprintf(buffer + offset, sizeof(buffer) - (size_t)offset, "\n");
    Send(buffer);
}

/* ------------------------------------------------------------------ команды */

static void HandleSet(char **argv, uint8_t argc)
{    uint32_t parameter = 0U;
    uint8_t value = 0U;

    if ((argc == 1U) || ((argc == 2U) && StrEqualCI(argv[1], "?")))
    {
        SendSettings();
        return;
    }

    if (argc != 3U)
    {
        SendError("SYNTAX");
        return;
    }

    //настройки во время передачи не меняем
    if (flag_ptt_on == 1U)
    {
        SendError("BUSY");
        return;
    }

    value = (uint8_t)atoi(argv[2]);

    if (StrEqualCI(argv[1], "LED"))
    {
        if (value > 2U)
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_1;
        Settings_PA.SET_LED_DISPLAY = (SETUP_PA_PARAM)value;
        EEPROM_Write(parameter, value);
        LED_display(Settings_PA.SET_LED_DISPLAY);
    }
    else if (StrEqualCI(argv[1], "POW"))
    {
        if (value > 1U)
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_2;
        Settings_PA.SET_CHANGE_POWER = (SETUP_PA_PARAM)value;
        EEPROM_Write(parameter, value);
        att_data = 0U;
        PE4302_change_att(1U);
    }
    else if (StrEqualCI(argv[1], "LPF"))
    {
        if (value > 1U)
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_3;
        Settings_PA.SET_MAN_CONTROL_LPF = (SETUP_PA_PARAM)value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "TCOOL"))
    {
        if ((value < MIN_T) || (value > MAX_T))
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_4;
        Settings_PA.SET_TEMP_COOLER = value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "UMIN"))
    {
        if ((value < DEF_MIN_U) || (value > DEF_MAX_U))
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_5;
        Settings_PA.SET_MIN_U = value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "UMAX"))
    {
        if ((value < DEF_MIN_U) || (value > (DEF_MAX_U + 5U)))
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_6;
        Settings_PA.SET_MAX_U = value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "IMAX"))
    {
        if ((value < DEF_MIN_I) || (value > DEF_MAX_I))
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_7;
        Settings_PA.SET_MAX_I = value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "SWR"))
    {
        if ((value < MIN_SWR) || (value > MAX_SWR))
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_8;
        Settings_PA.SET_MAX_SWR = value;
        EEPROM_Write(parameter, value);
    }
    else if (StrEqualCI(argv[1], "FANLM"))
    {
        if (value > 1U)
        {
            SendError("RANGE");
            return;
        }

        parameter = PARAM_18;
        Settings_PA.SET_FAN_NO_LM75A = (SETUP_PA_PARAM)value;
        EEPROM_Write(parameter, value);
        FAN_fail_safe();
    }
    else
    {
        SendError("SYNTAX");
        return;
    }

    Send("OK\n");
}

static void HandleAtt(char **argv, uint8_t argc)
{
    int8_t index;
    uint8_t value;

    if ((argc == 1U) || ((argc == 2U) && StrEqualCI(argv[1], "?")))
    {
        SendAttenuation();
        return;
    }

    if (argc != 3U)
    {
        SendError("SYNTAX");
        return;
    }

    index = BandToIndex(atoi(argv[1]));
    value = (uint8_t)atoi(argv[2]);

    if ((index < 0) || (value > 63U))
    {
        SendError("RANGE");
        return;
    }

    /*
     * На передаче разрешена запись только затухания активного диапазона
     * и только если включено ручное изменение мощности (SET POW 1).
     * На приёме доступны все диапазоны.
     */
    if (flag_ptt_on == 1U)
    {
        if ((Settings_PA.SET_CHANGE_POWER != ON) ||
            ((uint8_t)LPF_active != (uint8_t)index))
        {
            SendError("BUSY");
            return;
        }
    }

    bnd_in_att_data[index] = value;
    band_att_data[index] = value;
    EEPROM_Write(s_attParam[(uint8_t)index], value);

    if ((uint8_t)LPF_active == (uint8_t)index)
    {
        att_data = value;
        PE4302_change_att(1U);
    }

    Send("OK\n");
}

static void HandleBand(char **argv, uint8_t argc)
{
    if ((argc == 1U) || ((argc == 2U) && StrEqualCI(argv[1], "?")))
    {
        char buffer[24];
        snprintf(buffer, sizeof(buffer), "BAND %u\n", (unsigned)BandFromIndex((uint8_t)LPF_active));
        Send(buffer);
        return;
    }

    if (argc != 2U)
    {
        SendError("SYNTAX");
        return;
    }

    int8_t index = BandToIndex(atoi(argv[1]));
    if (index < 0)
    {
        SendError("RANGE");
        return;
    }

    if (flag_ptt_on == 1U)
    {
        SendError("BUSY");
        return;
    }

    LPF_active = (BAND_INDEX)index;
    LPF_change_band();
    Send("OK\n");
}

static void HandleFan(char **argv, uint8_t argc)
{
    if ((argc == 1U) || ((argc == 2U) && StrEqualCI(argv[1], "?")))
    {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "FAN %u\n", (unsigned)(fan_on ? 1U : 0U));
        Send(buffer);
        return;
    }

    if (argc != 2U)
    {
        SendError("SYNTAX");
        return;
    }

    int value = atoi(argv[1]);

    if ((value < 0) || (value > 1))
    {
        SendError("RANGE");
        return;
    }

    FAN_test_set((uint8_t)value);
    Send("OK\n");
}

static void HandleState(void)
{
    char buffer[96];

    snprintf(buffer, sizeof(buffer),
             "STATE PG=%u SEL=%u KEY=%u BUT=%u LAST=%u PTT=%u FAULT=%u\n",
             (unsigned)PROG_state,
             (unsigned)sel_param,
             (unsigned)adc_values[0],
             (unsigned)BUT_press,
             (unsigned)lastButton,
             (unsigned)(flag_ptt_on ? 1U : 0U),
             (unsigned)pa_fault);

    Send(buffer);
}

static void HandlePtt(char **argv, uint8_t argc)
{
    if ((argc == 1U) || ((argc == 2U) && StrEqualCI(argv[1], "?")))
    {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "PTT %u\n", (unsigned)(flag_ptt_on ? 1U : 0U));
        Send(buffer);
        return;
    }

    if (argc != 2U)
    {
        SendError("SYNTAX");
        return;
    }

    if (pa_fault != PA_FAULT_NONE)
    {
        SendError("BUSY");
        return;
    }

    flag_ptt_on = (uint8_t)(atoi(argv[1]) != 0);
    flag_ptt_change = 1U;
    PTT_process();
    Send("OK\n");
}

/* Принудительно отключить USB (прижать D+ к земле), чтобы хост увидел
 * переподключение, и перезагрузить МК. Без этого при постоянной подтяжке
 * D+ к 3,3 В хост не замечает сброс и COM-порт остаётся нерабочим. */
static void USB_DisconnectAndReset(void)
{
    //даём ответу OK уйти в USB
    HAL_Delay(50U);

    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    //держим D+ низким, чтобы хост точно зафиксировал отключение
    HAL_Delay(100U);

    //после сброса пин отпустится, внешняя подтяжка снова поднимет D+
    NVIC_SystemReset();
}

static void ProcessLine(char *line)
{
    char *argv[4];
    uint8_t argc = 0U;
    char *cursor = line;

    while ((*cursor != '\0') && (argc < 4U))
    {
        while (*cursor == ' ')
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            break;
        }

        argv[argc++] = cursor;

        while ((*cursor != '\0') && (*cursor != ' '))
        {
            cursor++;
        }

        if (*cursor == ' ')
        {
            *cursor = '\0';
            cursor++;
        }
    }

    if (argc == 0U)
    {
        return;
    }

    /* допускаем команды и с '?' и без него: VER? == VER, SET? == SET */
    for (uint8_t i = 0U; i < argc; i++)
    {
        size_t length = strlen(argv[i]);

        if ((length > 0U) && (argv[i][length - 1U] == '?'))
        {
            argv[i][length - 1U] = '\0';
        }
    }

    if (StrEqualCI(argv[0], "ID"))
    {
        Send("ID HFPA-R9OFG\n");
    }
    else if (StrEqualCI(argv[0], "PING"))
    {
        Send("PONG\n");
    }
    else if (StrEqualCI(argv[0], "VER"))
    {
        Send("VER " PA_FW_VERSION_STRING "\n");
    }
    else if (StrEqualCI(argv[0], "TEL"))
    {
        SendTelemetry();
    }
    else if (StrEqualCI(argv[0], "DIAG"))
    {
        HandleDiag();
    }
    else if (StrEqualCI(argv[0], "STATE"))
    {
        HandleState();
    }
    else if (StrEqualCI(argv[0], "WORK"))
    {
        force_work = 1U;
        Send("OK\n");
    }

    else if (StrEqualCI(argv[0], "SET"))
    {
        HandleSet(argv, argc);
    }
    else if (StrEqualCI(argv[0], "ATT"))
    {
        HandleAtt(argv, argc);
    }
    else if (StrEqualCI(argv[0], "BAND"))
    {
        HandleBand(argv, argc);
    }
    else if (StrEqualCI(argv[0], "PTT"))
    {
        HandlePtt(argv, argc);
    }
    else if (StrEqualCI(argv[0], "FAN"))
    {
        HandleFan(argv, argc);
    }
    else if (StrEqualCI(argv[0], "RESET"))
    {
        SETUP_pa_default();
        Send("OK\n");
        USB_DisconnectAndReset();
    }
    else if (StrEqualCI(argv[0], "REBOOT"))
    {
        Send("OK\n");
        USB_DisconnectAndReset();
    }
    else
    {
        SendError("UNKNOWN");
    }
}

/* ------------------------------------------------------------------ публичные функции */

void PA_Protocol_Push(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++)
    {
        uint16_t next = (uint16_t)((s_rxHead + 1U) % PROTO_RING_SIZE);

        if (next != s_rxTail)
        {
            s_rxRing[s_rxHead] = data[i];
            s_rxHead = next;
        }
    }
}

void PA_Protocol_Poll(void)
{
    while (s_rxTail != s_rxHead)
    {
        char symbol = (char)s_rxRing[s_rxTail];
        s_rxTail = (uint16_t)((s_rxTail + 1U) % PROTO_RING_SIZE);

        if ((symbol == '\n') || (symbol == '\r'))
        {
            if (s_lineLength > 0U)
            {
                s_line[s_lineLength] = '\0';
                ProcessLine(s_line);
                s_lineLength = 0U;
            }
        }
        else if (s_lineLength < (PROTO_LINE_SIZE - 1U))
        {
            s_line[s_lineLength++] = symbol;
        }
    }


}
