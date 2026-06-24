cmake_minimum_required(VERSION 3.22)

project(app)
add_library(app INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(app INTERFACE 
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(app INTERFACE
    ${CMAKE_SOURCE_DIR}/Core/Include
    ${CMAKE_SOURCE_DIR}/Libraries/CH32V305_STD_Driver/Core
    ${CMAKE_SOURCE_DIR}/Libraries/CH32V305_STD_Driver/Debug
    ${CMAKE_SOURCE_DIR}/Libraries/CH32V305_STD_Driver/Peripheral/inc

    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP
    # ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/DAP/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/cdc
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/hid
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/msc
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/dfu
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/common
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/core
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/port/ch32/ch32hs
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryRB
    ${CMAKE_SOURCE_DIR}/Middlewares/port
)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/Core/*.c
    ${CMAKE_SOURCE_DIR}/Libraries/CH32V305_STD_Driver/*.c

    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/DAP/Source/*.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/dap_main.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/cdc/usbd_cdc_acm.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/hid/usbd_hid.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/msc/usbd_msc.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/class/dfu/usbd_dfu.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/core/usbd_core.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryUSB/port/ch32/usb_dc_usbhs.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/CherryRB/chry_ringbuffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/port/*.c
)
list(REMOVE_ITEM SOURCES 
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/dap_main.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/DAP/Source/DAP_vendor.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/DAP/Source/DAP.c
    ${CMAKE_SOURCE_DIR}/Middlewares/CherryDAP/DAP/Source/JTAG_DP.c
    # ${CMAKE_SOURCE_DIR}/Middlewares/CherryUSB/port/ch32/usb_dc_usbhs.c
    ${CMAKE_SOURCE_DIR}/Core/Source/boot/main.c
    ${CMAKE_SOURCE_DIR}/Core/Source/boot/dfu_with_st_tool_template.c
    ${CMAKE_SOURCE_DIR}/Core/Source/app/main.c
    ${CMAKE_SOURCE_DIR}/Core/Source/app/usb2uart.c
)

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

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/Core/Source/main.c")

# ###################### USER ADD LINES ##############################

