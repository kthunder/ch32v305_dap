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
    -T ${CMAKE_SOURCE_DIR}/SRC/Ld/Link.ld
)