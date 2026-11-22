cmake_minimum_required(VERSION 3.22)

project(HF_PA_OU_2xRD16HHF1_2xRD100HHF1 C ASM)

# ---------------------------------------------------------------------------
# Параметры проекта
# ---------------------------------------------------------------------------

set(MCU_FLAGS   -mcpu=cortex-m3 -mthumb -mfloat-abi=soft)
set(LD_SCRIPT   "${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld")
set(FW_NAME     "hf_pa_controller_stm32f103c8t6_r9ofg_2.0")

# Тестовая сборка: только USB CDC (диагностика)
option(PA_USB_ONLY_TEST "Собрать тестовую прошивку только с USB CDC" OFF)
if(PA_USB_ONLY_TEST)
    set(FW_NAME "usb_cdc_test")
endif()

# ---------------------------------------------------------------------------
# Исходники
# ---------------------------------------------------------------------------

file(GLOB CORE_SOURCES    "${CMAKE_SOURCE_DIR}/Core/Src/*.c")
file(GLOB HAL_SOURCES     "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/*.c")
file(GLOB USB_SOURCES     "${CMAKE_SOURCE_DIR}/Drivers/STM32_USB_Device_Library/Core/Src/*.c"
                          "${CMAKE_SOURCE_DIR}/Drivers/STM32_USB_Device_Library/Class/CDC/Src/*.c")
set(STARTUP_SOURCE        "${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32f103c8tx.s")

add_executable(${PROJECT_NAME}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${USB_SOURCES}
    ${STARTUP_SOURCE}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${FW_NAME}"
    SUFFIX ".elf"
    LINK_DEPENDS "${LD_SCRIPT}"
)

# ---------------------------------------------------------------------------
# Заголовочные файлы и defines
# ---------------------------------------------------------------------------

target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/Core/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include"
    "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32_USB_Device_Library/Core/Inc"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32_USB_Device_Library/Class/CDC/Inc"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F103xB
    $<$<BOOL:${PA_USB_ONLY_TEST}>:USB_ONLY_TEST>
)

# ---------------------------------------------------------------------------
# Флаги компиляции
#
# Исходники и строки в UTF-8; шрифт дисплея GMG12864 построен по таблице
# CP1251, поэтому LCD_Output_String() конвертирует UTF-8 в CP1251 на лету.
# Дополнительных флагов кодировки от тулчейна не требуется.
# ---------------------------------------------------------------------------

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    $<$<CONFIG:Debug>:-Og;-g3;-DDEBUG>
    $<$<CONFIG:Release>:-Os;-g3>
)

# ---------------------------------------------------------------------------
# Линковка
# ---------------------------------------------------------------------------

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${LD_SCRIPT}
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments
    -Wl,-Map=${CMAKE_BINARY_DIR}/${FW_NAME}.map,--cref
    -specs=nano.specs
    -specs=nosys.specs
    -static
)

# ---------------------------------------------------------------------------
# Пост-обработка: hex/bin с именем прошивки
# ---------------------------------------------------------------------------

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> "${CMAKE_BINARY_DIR}/${FW_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> "${CMAKE_BINARY_DIR}/${FW_NAME}.bin"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Формирование ${FW_NAME}.hex / .bin"
)

# ---------------------------------------------------------------------------
# Прошивка через OpenOCD (ST-Link)
# ---------------------------------------------------------------------------

find_program(OPENOCD_EXECUTABLE openocd)

if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE}
                -f interface/stlink.cfg
                -f target/stm32f1x.cfg
                -c "program ${CMAKE_BINARY_DIR}/${FW_NAME}.hex verify reset exit"
        DEPENDS ${PROJECT_NAME}
        COMMENT "Прошивка ${FW_NAME}.hex через ST-Link"
        VERBATIM
    )
else()
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "OpenOCD не найден. Установите: winget install xpack-dev-tools.openocd-xpack"
        COMMAND ${CMAKE_COMMAND} -E false
    )
endif()
