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

# ---------------------------
# Version info: generated at build time via version.h
# ---------------------------

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# ---------------------------
# Boot mode selection
# ---------------------------
# OFF (default): image runs under the custom secure bootloader from the active
#                slot (linked at 0x08020400, VTOR relocated there). A DFU
#                request hands control back to the bootloader via the RTC
#                backup register + reset.
# ON           : bare-metal image flashed directly to 0x08000000 with NO
#                bootloader. A DFU request jumps to the STM32 system-memory ROM
#                bootloader (0x1FF09800).
option(BARE_METAL "Build a bare-metal image (no custom bootloader; runs at 0x08000000)" OFF)

# ---------------------------
# Factory programming (PTR-2026-1-5 follow-up)
# ---------------------------
# OFF (default, every Release and Debug build): the FPGA/UFM programming
#      service (OW_FPGA_PROG) and the Lattice programming entry points are
#      compiled OUT of the image, and the laser-safety parameters are
#      firmware-owned (safety_envelope.h): a host cannot set them.
# ON  (CMake preset "Factory" / "Factory-BareMetal" only): programming and
#      safety-parameter overrides are compiled in for manufacturing and RMA
#      bench work. Never built for an rc/final tag (clinical-build.yml builds
#      the Release preset only; build-firmware.yml checks the Release ELF for
#      programming symbols). A Factory image announces itself at boot and in
#      its version string ("-factory").
option(OW_FACTORY_PROGRAMMING "Factory build: FPGA programming and safety-parameter overrides compiled in (bench only)" OFF)
if(OW_FACTORY_PROGRAMMING AND CMAKE_BUILD_TYPE STREQUAL "Release")
    message(FATAL_ERROR "OW_FACTORY_PROGRAMMING must not be combined with a Release build type: use the Factory preset (Debug).")
endif()

# Generate version.h at configure time (initial)
find_package(Git QUIET)

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FW_GIT_DESCRIBE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_DESCRIBE_RESULT
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FW_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_SHA_RESULT
    )
    if(NOT GIT_DESCRIBE_RESULT EQUAL 0)
        set(FW_GIT_DESCRIBE "unknown")
    endif()
    if(NOT GIT_SHA_RESULT EQUAL 0)
        set(FW_GIT_SHA "unknown")
    endif()
else()
    set(FW_GIT_DESCRIBE "unknown")
    set(FW_GIT_SHA "unknown")
endif()

if(OW_FACTORY_PROGRAMMING)
    set(FW_GIT_DESCRIBE "${FW_GIT_DESCRIBE}-factory")
endif()

string(TIMESTAMP FW_BUILD_TIME_UTC "%Y-%m-%d %H:%M:%S UTC" UTC)

message(STATUS "FW version   : ${FW_GIT_DESCRIBE}")
message(STATUS "FW git sha   : ${FW_GIT_SHA}")
message(STATUS "FW build time: ${FW_BUILD_TIME_UTC}")

configure_file(
    ${CMAKE_SOURCE_DIR}/version.h.in
    ${CMAKE_BINARY_DIR}/generated/version.h
    @ONLY
)

# ---------------------------
# Expected FPGA configuration digests (fpga_attest.c)
# ---------------------------
# Generated from the released FPGA artefacts committed under fpga-artefacts/
# at the tags recorded in scripts/sbom-manifest.json. The configure step
# FAILS when a tag, an artefact or a digest does not match the manifest.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(NOT Python3_Interpreter_FOUND)
    message(FATAL_ERROR "python3 is required to generate fpga_attest_expected.h (scripts/gen_fpga_digests.py)")
endif()
set(FPGA_ARTEFACT_DIR "${CMAKE_SOURCE_DIR}/fpga-artefacts" CACHE PATH "Released FPGA .jed artefacts, <repo>/<tag>/<file>")
execute_process(
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_SOURCE_DIR}/scripts/gen_fpga_digests.py
            --manifest ${CMAKE_SOURCE_DIR}/scripts/sbom-manifest.json
            --artefacts ${FPGA_ARTEFACT_DIR}
            -o ${CMAKE_BINARY_DIR}/generated/fpga_attest_expected.h
    RESULT_VARIABLE FPGA_DIGEST_RESULT
    OUTPUT_VARIABLE FPGA_DIGEST_OUTPUT
    ERROR_VARIABLE FPGA_DIGEST_ERROR
)
message(STATUS "${FPGA_DIGEST_OUTPUT}")
if(NOT FPGA_DIGEST_RESULT EQUAL 0)
    message(FATAL_ERROR "FPGA digest generation failed:
${FPGA_DIGEST_ERROR}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/scripts/sbom-manifest.json
    ${CMAKE_SOURCE_DIR}/scripts/gen_fpga_digests.py)

# Set the project name
set(CMAKE_PROJECT_NAME motion-console-fw)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Select the linker script based on the BARE_METAL option. The matching
# BARE_METAL_BUILD compile definition is applied to the stm32cubemx interface
# library below (after add_subdirectory) so the STM32 drivers — notably
# system_stm32h7xx.c, which owns the VTOR base address and the ROM-bootloader
# jump — pick it up alongside the application sources.
if(BARE_METAL)
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H743XX_FLASH_BARE.ld")
    message(STATUS "Boot mode    : BARE METAL (0x08000000, DFU -> ROM bootloader)")
else()
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H743XX_FLASH.ld")
    message(STATUS "Boot mode    : custom bootloader slot (0x08020400)")
endif()
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -T "${LINKER_SCRIPT}")
# Re-link when the chosen linker script changes.
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")

# Enable float printing in printf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Propagate the boot-mode define to every translation unit (application sources
# and the STM32 drivers in the STM32_Drivers object library, which both consume
# the stm32cubemx interface library).
if(BARE_METAL)
    target_compile_definitions(stm32cubemx INTERFACE BARE_METAL_BUILD=1)
endif()
if(OW_FACTORY_PROGRAMMING)
    target_compile_definitions(stm32cubemx INTERFACE OW_FACTORY_PROGRAMMING=1)
    message(STATUS "Factory build : FPGA programming and safety overrides COMPILED IN (bench only)")
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    USB_DEVICE/App/usb_events.c
    USB_DEVICE/App/usb_recovery.c
    Core/Src/ad5761r.c
    Core/Src/ads7828.c
    Core/Src/ads7924.c
    Core/Src/fan_driver.c
    Core/Src/jsmn.c
    Core/Src/json_strict.c
    Core/Src/led_driver.c
    Core/Src/logging.c
    Core/Src/lwrb.c
    Core/Src/max31875.c
    Core/Src/pca9535.c
    Core/Src/tca9548.c
    Core/Src/console_i2c_health.c
    Core/Src/mcp42u83.c
    Core/Src/trigger.c
    Core/Src/XO2_dev.c
    Core/Src/XO2_cmds.c
    Core/Src/XO2_api.c
    Core/Src/if_commands.c
    Core/Src/if_fpga_prog.c
    Core/Src/msg_queue.c
    Core/Src/uart_comms.c
    Core/Src/flash_eeprom.c
    Core/Src/motion_config.c
    Core/Src/motion_cfg_keys.c
    Core/Src/safety_envelope.c
    Core/Src/safety_ufm_record.c
    Core/Src/sha256.c
    Core/Src/fpga_attest.c
    Core/Src/fpga_register_map.c
    Core/Src/utils.c
    Core/Src/pdc_buffer.c
    Core/Src/pdc_poll.c
    Core/Src/odometer.c
    Core/Src/console_serial.c
    Core/Src/eeprom_24aa025.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_BINARY_DIR}/generated
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # jsmn strict mode (CFW-15): unquoted values must be numbers, true, false
    # or null, and must not be object keys. Both parsers (trigger JSON and the
    # stored user config) only ever receive JSON produced by json.dumps or
    # JSON.stringify; host_tests build jsmn the same way.
    JSMN_STRICT
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate hex and bin files after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating .hex and .bin files"
)

# Build + DFU-flash target. Invokes scripts/deploy.py which:
#   1. enters DFU via the omotion SDK
#   2. waits for the STM32 ROM bootloader to enumerate
#   3. flashes the .bin via dfu-util with :leave for auto-reset
# Requires: dfu-util on PATH, omotion installed in the active Python env.
find_program(PYTHON_FOR_DEPLOY NAMES python3 python)
if(PYTHON_FOR_DEPLOY)
    add_custom_target(flash
        COMMAND ${PYTHON_FOR_DEPLOY} ${CMAKE_SOURCE_DIR}/scripts/deploy.py
                --config $<CONFIG> --no-build
        DEPENDS ${CMAKE_PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        USES_TERMINAL
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} via DFU"
    )
else()
    message(WARNING "Python not found — `flash` target will not be available")
endif()
