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
)
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/SRC/*.c
    ${CMAKE_SOURCE_DIR}/User/*.c
)
list(REMOVE_ITEM SOURCES 
    # ${CMAKE_SOURCE_DIR}/User/msc_ram_template.c
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

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/User/main.c")

# ###################### USER ADD LINES ##############################

