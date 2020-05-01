##
## Authors:  Diego Bienz
##
project(Tiny-Lacuna)
cmake_minimum_required(VERSION 3.6)
enable_language(ASM)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------

list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/rtc-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpio-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/uart-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/delay-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/spi-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sx1262-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/i2c-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/gps-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/se050-board.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/pin_mux.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis/clock_config.c"

    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/gcc/startup_MK22F51212.S"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/system_MK22F51212.c"

    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/utilities.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_rtc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_clock.c" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_dmamux.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_edma.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_smc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_dspi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers/fsl_i2c_edma.c"
)
set_property(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/gcc/startup_MK22F51212.S" PROPERTY LANGUAGE C)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_compile_definitions(${PROJECT_NAME} PUBLIC -DCPU_MK22FN512VLH12)

# Add define if debbuger support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_DEBUGGER}>:USE_DEBUGGER>)

# Add define if radio debug pins support is enabled
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<BOOL:${USE_RADIO_DEBUG}>:USE_RADIO_DEBUG>)

# For debug builds set the symbol DEBUG
set(CMAKE_C_FLAGS_DEBUG -DDEBUG)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/cmsis
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/arm
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/cmsis_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/gcc
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/utilities/debug_console
    ${CMAKE_CURRENT_SOURCE_DIR}/../mcu/mk22/MK22F51212/utilities/str
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
