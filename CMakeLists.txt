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

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 彩色诊断输出 - 完整配置
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
    add_link_options(-fdiagnostics-color=always)
elseif(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
endif()

# 设置环境变量
set(ENV{FORCE_COLOR} "1")
# add_compile_definitions(STM32F401xC)

# Add STM32CubeMX generated sources
add_subdirectory(.cmake/app)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # STM32F401xC
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    app
    # Add user defined libraries
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)
set(DIS_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.dis)
set(LST_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.lst)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
        COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > ${DIS_FILE}
        COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${PROJECT_NAME}> > ${LST_FILE}
        # COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${CMAKE_PROJECT_NAME}.elf> > ${LST_FILE}
        COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        COMMENT "Building completed"
)
