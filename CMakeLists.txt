cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME) 
set(CMAKE_PROJECT_NAME ${PROJECT_DIR})

# Include toolchain file
include(".cmake/riscv-none-embed.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 彩色诊断输出 - 完整配置
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
    add_link_options(-fdiagnostics-color=always)
elseif(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
endif()
# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Common compile definitions
# add_compile_definitions(APM32F407xx)
# add_compile_definitions(APM32F40X)
# add_compile_definitions(USE_DAL_DRIVER)
# add_compile_definitions(HSE_VALUE=12000000U)

# Add STM32CubeMX generated sources
add_subdirectory(.cmake/app)

# ==================== Main Application ====================
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Source/app/main.c
    ${CMAKE_SOURCE_DIR}/Core/Source/app/usb2uart.c
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    app
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T ${CMAKE_SOURCE_DIR}/Core/Source/app/ch32v305_flash.ld
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)
set(DIS_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.dis)
set(LST_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.lst)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
    COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > ${DIS_FILE}
    COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > ${LST_FILE}
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Building application completed"
)

# ==================== Boot Firmware ====================
add_executable(${CMAKE_PROJECT_NAME}_boot
    ${CMAKE_SOURCE_DIR}/Core/Source/boot/main.c
    ${CMAKE_SOURCE_DIR}/Core/Source/boot/dfu_with_st_tool_template.c
)
set_target_properties(${CMAKE_PROJECT_NAME}_boot PROPERTIES SUFFIX ".elf")

target_link_options(${CMAKE_PROJECT_NAME}_boot PRIVATE
    -T ${CMAKE_SOURCE_DIR}/Core/Source/boot/ch32v305_flash.ld
)

target_compile_definitions(${CMAKE_PROJECT_NAME}_boot PRIVATE BOOT_FW)

target_link_libraries(${CMAKE_PROJECT_NAME}_boot PRIVATE
    app
)

set(BOOT_HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot.hex)
set(BOOT_BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot.bin)
set(BOOT_DIS_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot.dis)
set(BOOT_LST_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_boot.lst)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME}_boot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}_boot> ${BOOT_HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}_boot> ${BOOT_BIN_FILE}
    COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}_boot> > ${BOOT_DIS_FILE}
    COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${CMAKE_PROJECT_NAME}_boot> > ${BOOT_LST_FILE}
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}_boot>
    COMMENT "Building boot firmware completed"
)