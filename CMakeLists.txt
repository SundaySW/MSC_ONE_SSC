cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchain.cmake)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "CMAKE_BUILD_TYPE not set, defaulting to DEBUG")
    set(CMAKE_BUILD_TYPE DEBUG)
else()
    message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")

project(MSC_ONE_SSC     C CXX ASM)
set(DEVICE_FAMILY       STM32G4xx)
set(DEVICE_NAME         STM32G431xx)
set(DEVICE_FULL_NAME    STM32G431CBTX)

include(cmake/utils.cmake)

include_directories(
        App
        Core
        Drivers
        Libs
)

add_subdirectory(Drivers)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/App/*.*
        ${CMAKE_CURRENT_SOURCE_DIR}/Libs/*.*
        )

add_executable(${PROJECT_NAME} Core/Src/main.c)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Core
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/App
        ${CMAKE_CURRENT_SOURCE_DIR}/Libs
        )

#Excluding directory - no compilation
set (EXCLUDE_DIR "tmp")
foreach (TMP_PATH ${SOURCES})
    string (FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
    if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
        list (REMOVE_ITEM SOURCES ${TMP_PATH})
    endif ()
endforeach(TMP_PATH)


target_sources(${PROJECT_NAME}
        PUBLIC
        ${SOURCES}
        )

set_target_properties(${PROJECT_NAME}
        PROPERTIES
        SUFFIX ".elf"
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
        CXX_STANDARD 20
        CXX_EXTENSIONS ON
        CMAKE_CXX_STANDARD_REQUIRED ON
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_NAME}-Drivers
        )

target_link_options(${PROJECT_NAME}
        PUBLIC
        -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map,--cref
        )

utils_target_set_linker_script(${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_FULL_NAME}_FLASH.ld
        )

utils_target_generate_hex(${PROJECT_NAME})
utils_target_generate_bin(${PROJECT_NAME})
utils_target_print_size(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME}
        PRIVATE
        -Wno-volatile
#        -Wall
#        -Wextra
#        -Wpedantic
#        -Werror
#        -Wduplicated-cond
#        -Wduplicated-branches
#        -Wlogical-op
#        -Wnull-dereference
#        -Wshadow
#        -Wconversion
#        -Wsign-conversion
        )

#if(RTT_LOG)
#    add_subdirectory(libs/rtt)
#    target_link_libraries(${PROJECT_NAME}
#            PRIVATE
#            rtt
#)
#endif()

#set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
#set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
#
#add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
#        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
#        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
#        COMMENT "Building ${HEX_FILE}
#Building ${BIN_FILE}")