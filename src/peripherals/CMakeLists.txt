##
##   ______                              _
##  / _____)             _              | |
## ( (____  _____ ____ _| |_ _____  ____| |__
##  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
##  _____) ) ____| | | || |_| ____( (___| | | |
## (______/|_____)_|_|_| \__)_____)\____)_| |_|
## (C)2013-2017 Semtech
##  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
## Authors:  Johannes Bruder (STACKFORCE), Miguel Luis (Semtech)
##
project(peripherals)
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------

list(APPEND ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/gpio-ioe.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/mag3110.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/mma8451.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/mpl3115.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/pam7q.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sx1509.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sx9500.c"
)

if(SECURE_ELEMENT STREQUAL soft-se)
    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/soft-se/aes.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/soft-se/cmac.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/soft-se/soft-se.c"
    )

elseif(SECURE_ELEMENT STREQUAL se050)

    file(GLOB ${PROJECT_NAME}_HOSTLIB_INFRA "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/infra/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_NXSCP "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/nxScp/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_SCP "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/scp/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_SMCOM "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/smCom/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_T1OI2C "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/smCom/T1oI2C/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_MBEDTLS "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/mbedtls/src/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_KSDK "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/platform/ksdk/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_SE05X "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/se05x/src/*.c")
    file(GLOB ${PROJECT_NAME}_HOSTLIB_SE05X03 "${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/se05x_03_xx_xx/*.c")
    file(GLOB ${PROJECT_NAME}_SSS_EX_SRC "${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/ex/src/*.c")
    file(GLOB ${PROJECT_NAME}_SSS_PLUGIN_MBEDTLS "${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/plugin/mbedtls/*.c")
    file(GLOB ${PROJECT_NAME}_SSS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/src/*.c")
    file(GLOB ${PROJECT_NAME}_SSS_MBEDTLS "${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/src/mbedtls/*.c")
    file(GLOB ${PROJECT_NAME}_SSS_SE05X "${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/src/se05x/*.c")
    file(GLOB ${PROJECT_NAME}_MBEDTLS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/se050/mbedtls/library/*.c")
    file(GLOB ${PROJECT_NAME}_MBEDTLS_KSDK "${CMAKE_CURRENT_SOURCE_DIR}/se050/mbedtls/port/ksdk/*.c")

    list(APPEND ${PROJECT_NAME}_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/se050/se050.c"
        "${${PROJECT_NAME}_HOSTLIB_INFRA}"
        "${${PROJECT_NAME}_HOSTLIB_NXSCP}"
        "${${PROJECT_NAME}_HOSTLIB_SCP}"
        "${${PROJECT_NAME}_HOSTLIB_SMCOM}"
        "${${PROJECT_NAME}_HOSTLIB_T1OI2C}"
        "${${PROJECT_NAME}_HOSTLIB_MBEDTLS}"
        "${${PROJECT_NAME}_HOSTLIB_KSDK}"
        "${${PROJECT_NAME}_HOSTLIB_SE05X}"
        "${${PROJECT_NAME}_HOSTLIB_SE05X03}"
        "${${PROJECT_NAME}_SSS_EX_SRC}"
        "${${PROJECT_NAME}_SSS_PLUGIN_MBEDTLS}"
        "${${PROJECT_NAME}_SSS_SRC}"
        "${${PROJECT_NAME}_SSS_MBEDTLS}"
        "${${PROJECT_NAME}_SSS_SE05X}"
        "${${PROJECT_NAME}_MBEDTLS_SRC}"
        "${${PROJECT_NAME}_MBEDTLS_KSDK}"
    )
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/soft-se
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/infra
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/smCom
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/libCommon/smCom/T1oI2C
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/mbedtls/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/platform/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/hostlib/hostLib/se05x_03_xx_xx
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/ex/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/plugin/mbedtls
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/sss/port/ksdk
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/mbedtls/include
    ${CMAKE_CURRENT_SOURCE_DIR}/se050/mbedtls/port/ksdk
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
