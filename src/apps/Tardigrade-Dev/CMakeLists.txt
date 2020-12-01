##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
## Authors:  Diego Bienz
##
project(Tardigrade-Dev)
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------

# Allow switching of sub projects
set(SUB_PROJECT_LIST energy-mgmt)
set(SUB_PROJECT energy-mgmt CACHE STRING "Default sub project is energy-mgmt")
set_property(CACHE SUB_PROJECT PROPERTY STRINGS ${SUB_PROJECT_LIST})

#---------------------------------------------------------------------------------------
# Application
#---------------------------------------------------------------------------------------
file(GLOB ${PROJECT_NAME}_SOURCES "${CMAKE_CURRENT_LIST_DIR}/${SUB_PROJECT}/*.c")

add_executable(${PROJECT_NAME}-${SUB_PROJECT}
                            ${${PROJECT_NAME}_SOURCES}
                            $<TARGET_OBJECTS:mac>
                            $<TARGET_OBJECTS:system>
                            $<TARGET_OBJECTS:radio>
                            $<TARGET_OBJECTS:peripherals>
                            $<TARGET_OBJECTS:${BOARD}>
)

target_compile_definitions(${PROJECT_NAME}-${SUB_PROJECT}  PUBLIC
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mac,INTERFACE_COMPILE_DEFINITIONS>>
)

target_include_directories(${PROJECT_NAME}-${SUB_PROJECT} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_PROJECT}}
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mac,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:system,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:radio,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:peripherals,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:${BOARD},INTERFACE_INCLUDE_DIRECTORIES>>
)

set_property(TARGET ${PROJECT_NAME}-${SUB_PROJECT} PROPERTY C_STANDARD 11)

target_link_libraries(${PROJECT_NAME}-${SUB_PROJECT} m)

#---------------------------------------------------------------------------------------
# Debugging and Binutils
#---------------------------------------------------------------------------------------

include(gdb-helper)
include(binutils-arm-none-eabi)

# Generate debugger configurations
generate_run_gdb_stlink(${PROJECT_NAME}-${SUB_PROJECT})
generate_run_gdb_openocd(${PROJECT_NAME}-${SUB_PROJECT})
generate_vscode_launch_openocd(${PROJECT_NAME}-${SUB_PROJECT})

# Print section sizes of target
print_section_sizes(${PROJECT_NAME}-${SUB_PROJECT})

# Create output in hex and binary format
create_bin_output(${PROJECT_NAME}-${SUB_PROJECT})
create_hex_output(${PROJECT_NAME}-${SUB_PROJECT})
