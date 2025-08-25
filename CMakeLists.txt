set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.26)

set(CROSS_COMPILE "riscv-wch-elf-")
# specify cross-compilers and tools
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_AR ${CROSS_COMPILE}ar)
set(CMAKE_OBJCOPY ${CROSS_COMPILE}objcopy)
set(CMAKE_OBJDUMP ${CROSS_COMPILE}objdump)
set(SIZE ${CROSS_COMPILE}size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME) 
set(CMAKE_PROJECT_NAME ${PROJECT_DIR})
project(${PROJECT_DIR} C ASM)
# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/SRC/Ld/Link_app.ld)
set(APP_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/SRC/Ld/Link_app.ld)
set(BOOT_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/SRC/Ld/Link_boot.ld)
add_compile_options(-march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore)

include_directories(
        "SRC/Core"
        "SRC/Debug"
        "SRC/Peripheral/inc"
        "User"
        "CherryUSB/class/cdc"
        "CherryUSB/class/msc"
        "CherryUSB/common"
        "CherryUSB/core"
        "CherryUSB/port/ch32"
        "CherryRB"
        "CherryDAP"
        "CherryDAP/DAP/Include"
        "DAPHost"
)

file(GLOB_RECURSE APP_SOURCES
        "SRC/Core/core_riscv.c"
        "SRC/Debug/debug.c"
        "SRC/Peripheral/src/*.c"
        "SRC/Startup/startup_ch32v30x_D8C.S"
        "User/*.c"
	"CherryUSB/class/msc/usbd_msc.c"
        "CherryUSB/class/cdc/usbd_cdc.c"
        "CherryUSB/core/usbd_core.c"
        "CherryUSB/port/ch32/usb_dc_usbhs.c"
        "CherryRB/chry_ringbuffer.c"
        "DAPHost/swd_host.c"
        "CherryDAP/DAP/Source/*.c"
        "CherryDAP/dap_main.c"
)

file(GLOB_RECURSE BOOT_SOURCES
        "SRC/Core/core_riscv.c"
        "SRC/Debug/debug.c"
        "SRC/Peripheral/src/*.c"
        "SRC/Startup/startup_ch32v30x_D8C.S"
        "User/*.c"
        "CherryUSB/class/msc/usbd_msc.c"
        "CherryUSB/core/usbd_core.c"
        "CherryUSB/port/ch32/usb_dc_usbhs.c"
)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    # flto
    add_compile_options(-flto)
    add_compile_options(-O3 -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-O0 -g -gdwarf-2)
endif ()

add_compile_options(
        -fmessage-length=0
        -fsigned-char
        -ffunction-sections
        -fdata-sections
        -fno-common
        -Wunused
        -Wuninitialized
        -Os
        -flto
)

add_link_options(
        -march=rv32imacxw -mabi=ilp32
        -nostartfiles
        -Xlinker
        --gc-sections
        -lm
        -Wl,--print-memory-usage
        --specs=nano.specs --specs=nosys.specs
)

############### APP ###############
add_executable(${PROJECT_NAME}.elf ${APP_SOURCES})
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE PROJ=0)
target_link_options(${PROJECT_NAME}.elf PRIVATE
        -T ${LINKER_SCRIPT}
        -Wl,-Map,${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(LST_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.lst)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${PROJECT_NAME}.elf> > ${LST_FILE}
        COMMAND ${SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
)

############### BOOT ###############
add_executable(${PROJECT_NAME}_boot.elf ${BOOT_SOURCES})
target_compile_definitions(${PROJECT_NAME}_boot.elf PRIVATE PROJ=1)
target_link_options(${PROJECT_NAME}_boot.elf PRIVATE
        -T ${BOOT_LINKER_SCRIPT}
        -Wl,-Map,${PROJECT_BINARY_DIR}/${PROJECT_NAME}_boot.map
)

set(BOOT_HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_boot.hex)
set(BOOT_BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_boot.bin)
set(BOOT_LST_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_boot.lst)

add_custom_command(TARGET ${PROJECT_NAME}_boot.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}_boot.elf> ${BOOT_HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}_boot.elf> ${BOOT_BIN_FILE}
        COMMAND ${CMAKE_OBJDUMP} --all-headers --demangle --disassemble $<TARGET_FILE:${PROJECT_NAME}_boot.elf> > ${BOOT_LST_FILE}
        COMMAND ${SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}_boot.elf>
)
