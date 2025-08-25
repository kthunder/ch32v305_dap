cmake_minimum_required(VERSION 3.22)

project(app)
add_library(app INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(app INTERFACE 
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(app INTERFACE
    ${CMAKE_SOURCE_DIR}/SRC/Core
    ${CMAKE_SOURCE_DIR}/SRC/Debug
    ${CMAKE_SOURCE_DIR}/SRC/Peripheral/inc
    ${CMAKE_SOURCE_DIR}/User
    ${CMAKE_SOURCE_DIR}/CherryUSB/class/cdc
    ${CMAKE_SOURCE_DIR}/CherryUSB/class/msc
    ${CMAKE_SOURCE_DIR}/CherryUSB/common
    ${CMAKE_SOURCE_DIR}/CherryUSB/core
    ${CMAKE_SOURCE_DIR}/CherryUSB/port/ch32
    ${CMAKE_SOURCE_DIR}/CherryRB
    ${CMAKE_SOURCE_DIR}/CherryDAP
    ${CMAKE_SOURCE_DIR}/CherryDAP/DAP/Include
    ${CMAKE_SOURCE_DIR}/DAPHost
)
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/SRC/*.c
    ${CMAKE_SOURCE_DIR}/SRC/Startup/startup_ch32v30x_D8C.S
    ${CMAKE_SOURCE_DIR}/User/*.c
    ${CMAKE_SOURCE_DIR}/CherryUSB/class/cdc/usbd_msc.c
    ${CMAKE_SOURCE_DIR}/CherryUSB/class/cdc/usbd_cdc.c
    ${CMAKE_SOURCE_DIR}/CherryUSB/core/usbd_core.c
    ${CMAKE_SOURCE_DIR}/CherryUSB/port/ch32/usb_dc_usbhs.c
    ${CMAKE_SOURCE_DIR}/CherryRB/chry_ringbuffer.c
    ${CMAKE_SOURCE_DIR}/DAPHost/swd_host.c
    ${CMAKE_SOURCE_DIR}/CherryDAP/DAP/Source/*.c
    ${CMAKE_SOURCE_DIR}/CherryDAP/dap_main.c
)
        # "SRC/Core/core_riscv.c"
        # "SRC/Debug/debug.c"
        # "SRC/Peripheral/src/*.c"
        # "SRC/Startup/startup_ch32v30x_D8C.S"
        # "User/*.c"
        # "CherryUSB/class/msc/usbd_msc.c"
        # "CherryUSB/core/usbd_core.c"
        # "CherryUSB/port/ch32/usb_dc_usbhs.c"
target_sources(app INTERFACE
    ${SOURCES}
)

target_link_directories(app INTERFACE
)

target_link_libraries(app INTERFACE
)

# Validate that app code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()

# ###################### USER ADD LINES ##############################

