set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          riscv32)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                "riscv-wch-elf-")

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP                   ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
#     message(STATUS "Maximum optimization for speed")
#     # flto
#     # add_compile_options(-flto)
#     add_compile_options(-O3 -g)
# elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
#     message(STATUS "Maximum optimization for speed, debug info included")
#     add_compile_options(-Ofast -g)
# elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
#     message(STATUS "Maximum optimization for size")
#     add_compile_options(-Os)
# else ()
#     message(STATUS "Minimal optimization, debug info included")
#     add_compile_options(-O0 -g -gdwarf-2)
# endif ()

# MCU specific flags
add_compile_options(
    -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore
    -fmessage-length=0
    -fsigned-char
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wunused
    -Wuninitialized
    -Os
    -g
)

add_link_options(
    -march=rv32imacxw -mabi=ilp32
    -nostartfiles
    -Xlinker
    --gc-sections
    -lm
    -Wl,--print-memory-usage
    --specs=nano.specs --specs=nosys.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections
)

# 读取 main.c 文件内容
file(READ "User/main.c" MAIN_C_CONTENT)

# 提取 PROJ 宏定义
string(REGEX MATCH "#define PROJ ([A-Z]+)" PROJ_MATCH "${MAIN_C_CONTENT}")
if(PROJ_MATCH)
    set(PROJ_VALUE ${CMAKE_MATCH_1})
else()
    # 默认值
    set(PROJ_VALUE "APP")
endif()

# 根据 PROJ 值选择链接脚本
if(PROJ_VALUE STREQUAL "APP")
    add_link_options(-T ${CMAKE_SOURCE_DIR}/SRC/Ld/Link.ld)
    add_link_options(-Wl,--defsym=__flash_origin=0x00002000)
    add_link_options(-Wl,--defsym=__flash_length=120K)
elseif(PROJ_VALUE STREQUAL "BOOT")
    add_link_options(-T ${CMAKE_SOURCE_DIR}/SRC/Ld/Link.ld)
    add_link_options(-Wl,--defsym=__flash_origin=0x00000000)
    add_link_options(-Wl,--defsym=__flash_length=8K)
endif()