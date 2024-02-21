cmake_minimum_required(VERSION 3.16)

set(TARGET_CPU "cortex-m4")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ${TARGET_CPU})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER        arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER      arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER      arm-none-eabi-gcc)
set(CMAKE_SIZE_UTIL         arm-none-eabi-size)

set(COMMON_FLAGS    "-mthumb -mcpu=${TARGET_CPU}")
set(C_CXX_FLAGS     "-specs=nano.specs -specs=nosys.specs -ffunction-sections -fdata-sections -ffreestanding")
set(CXX_FLAGS       "-fno-exceptions -fno-rtti -fno-threadsafe-statics -fcoroutines")

set(CMAKE_C_FLAGS_INIT          "${COMMON_FLAGS} ${C_CXX_FLAGS}"              CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT        "${COMMON_FLAGS} ${C_CXX_FLAGS} ${CXX_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_INIT        "${COMMON_FLAGS} -x assembler-with-cpp"       CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--gc-sections"                           CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_DEBUG     "-O0 -g"            CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g"            CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE   "-O3 -DNDEBUG"      CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG"      CACHE STRING "" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_EXPORT_COMPILE_COMMANDS             ON)
set(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES    ON)
set(CMAKE_C_USE_RESPONSE_FILE_FOR_LIBRARIES   ON)
set(CMAKE_C_USE_RESPONSE_FILE_FOR_OBJECTS     ON)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES  ON)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_LIBRARIES ON)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS   ON)
set(CMAKE_NINJA_FORCE_RESPONSE_FILE           ON)