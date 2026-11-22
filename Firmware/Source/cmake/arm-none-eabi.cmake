# Toolchain-файл CMake для arm-none-eabi-gcc (STM32F103C8Tx).
#
# Компилятор ищется так:
#   1. переменная окружения ARM_TOOLCHAIN_DIR (путь до каталога тулчейна);
#   2. arm-none-eabi-gcc в PATH;
#   3. автоопределение D:/Tools/xpack-arm-none-eabi-gcc-*/bin.
#
# Ручная настройка: -DARM_TOOLCHAIN_DIR=C:/path/to/arm-none-eabi

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Пробные сборки не линкуем (для target без ОС это обычно мешает)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ARM_TOOLCHAIN_DIR AND EXISTS "${ARM_TOOLCHAIN_DIR}/bin")
  set(_ARM_BIN "${ARM_TOOLCHAIN_DIR}/bin")
elseif(DEFINED ENV{ARM_TOOLCHAIN_DIR} AND EXISTS "$ENV{ARM_TOOLCHAIN_DIR}/bin")
  set(_ARM_BIN "$ENV{ARM_TOOLCHAIN_DIR}/bin")
endif()

if(NOT _ARM_BIN)
  find_program(_ARM_GCC arm-none-eabi-gcc)
  if(_ARM_GCC)
    get_filename_component(_ARM_BIN "${_ARM_GCC}" DIRECTORY)
  endif()
endif()

if(NOT _ARM_BIN)
  file(GLOB _ARM_CANDIDATES
       "D:/Tools/xpack-arm-none-eabi-gcc-*/bin"
       "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
       "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin")
  if(_ARM_CANDIDATES)
    list(SORT _ARM_CANDIDATES)
    list(GET _ARM_CANDIDATES -1 _ARM_BIN)
  endif()
endif()

if(NOT _ARM_BIN)
  message(FATAL_ERROR
    "Не найден arm-none-eabi-gcc. Установите Arm GNU Toolchain и укажите путь "
    "через -DARM_TOOLCHAIN_DIR=... или переменную окружения ARM_TOOLCHAIN_DIR.")
endif()

set(_EXE "")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
  set(_EXE ".exe")
endif()

set(CMAKE_C_COMPILER   "${_ARM_BIN}/arm-none-eabi-gcc${_EXE}")
set(CMAKE_ASM_COMPILER "${_ARM_BIN}/arm-none-eabi-gcc${_EXE}")
set(CMAKE_CXX_COMPILER "${_ARM_BIN}/arm-none-eabi-g++${_EXE}")
set(CMAKE_OBJCOPY      "${_ARM_BIN}/arm-none-eabi-objcopy${_EXE}")
set(CMAKE_SIZE         "${_ARM_BIN}/arm-none-eabi-size${_EXE}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
