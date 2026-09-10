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
set(CMAKE_PROJECT_NAME ros)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST "Run a bxCAN internal loopback self-test firmware instead of the normal control loop." OFF)
option(MOTOR_BENCH_TEST "Run the one-shot sequential motor bench test." OFF)
option(CAN_MOTOR_BENCH_TEST "Run the CAN-injected motor bench test (internal loopback command stream)." OFF)
option(CALIBRATION_BENCH_TEST "Run the CAN-injected wheel calibration service bench test." OFF)
option(STEERING_BENCH_TEST "Run the zero-wheel-speed +/-5 degree steering bench test." OFF)
option(ACKERMANN_BENCH_TEST "Run the lifted-wheel Ackermann command and encoder bench test." OFF)
option(H4_CHARACTERIZATION_TEST "Run the lifted-wheel H4 hardware characterization." OFF)
option(IWDG_STALL_TEST "Run the opt-in watchdog stall/reset hardware probe." OFF)
option(SAFE_STOP_WATCHDOG_TEST "Run the opt-in SAFE_STOP watchdog feed probe." OFF)
option(AUTOTUNE_SAFE_PROFILE "Build the isolated lifted-wheel AutotuneSafe firmware profile." OFF)
option(LOCAL_SESSION_SMOKE "Build the actuator-free LocalSessionSmoke firmware profile." OFF)
set(H4_OPEN_LOOP_PWM 500 CACHE STRING "Open-loop PWM permille used by H4 characterization.")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable. LocalSessionSmoke is intentionally a separate
# source graph: it cannot link actuator, control, encoder, CAN, or PID code.
if(LOCAL_SESSION_SMOKE)
    set(LOCAL_SESSION_BUILD_ID "0x4C534D31UL" CACHE STRING
        "Firmware build identity for LocalSessionSmoke.")
    execute_process(
        COMMAND git rev-parse --short=8 HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE LOCAL_SESSION_GIT_ID
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE LOCAL_SESSION_GIT_RESULT
    )
    if(LOCAL_SESSION_GIT_RESULT EQUAL 0 AND LOCAL_SESSION_GIT_ID MATCHES "^[0-9a-fA-F]+$")
        set(LOCAL_SESSION_BUILD_ID "0x${LOCAL_SESSION_GIT_ID}UL")
    endif()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        BSP/autotune_safe_session.c
        BSP/autotune_safe_local_smoke.c
    )
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        LOCAL_SESSION_BUILD_ID=${LOCAL_SESSION_BUILD_ID}
    )
else()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        BSP/bsp_bxcan.c
        BSP/bsp_bxcan_loopback.c
        BSP/can_motor_bench.c
        BSP/steering_bench.c
        BSP/ackermann_bench.c
        BSP/h4_characterization.c
        BSP/chassis_control.c
        BSP/physical_estop.c
        BSP/wheel_calibration.c
        BSP/wheel_calibration_storage.c
        BSP/wheel_calibration_service.c
        BSP/safety_manager.c
        BSP/bsp_motor.c
        BSP/servo.c
        BSP/encoder.c
        BSP/PID.c
        BSP/watchdog.c
        BSP/pid_tuning.c
        BSP/autotune_safe.c
        Hardware/MyI2C.c
        Hardware/OLED.c
        Hardware/OLED_Data.c
        Hardware/oled_app.c
    )
endif()

# Font tables intentionally use flat initializers for fixed-width array data.
set_source_files_properties(Hardware/OLED_Data.c PROPERTIES
    COMPILE_OPTIONS "-Wno-missing-braces"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

if(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        BSP_BXCAN_RUN_LOOPBACK_SELF_TEST
    )
endif()

if(MOTOR_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        MOTOR_BENCH_TEST
    )
endif()

if(CAN_MOTOR_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        CAN_MOTOR_BENCH_TEST
        CALIBRATION_BENCH_DEFAULTS
    )
endif()

if(CALIBRATION_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        CALIBRATION_BENCH_TEST
    )
endif()

if(STEERING_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        STEERING_BENCH_TEST
    )
endif()

if(ACKERMANN_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ACKERMANN_BENCH_TEST
        CALIBRATION_BENCH_DEFAULTS
    )
endif()

if(H4_CHARACTERIZATION_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        H4_CHARACTERIZATION_TEST
        CALIBRATION_BENCH_DEFAULTS
        H4_OPEN_LOOP_PWM=${H4_OPEN_LOOP_PWM}
    )
endif()

if(IWDG_STALL_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IWDG_STALL_TEST)
endif()

if(SAFE_STOP_WATCHDOG_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SAFE_STOP_WATCHDOG_TEST)
endif()

if(AUTOTUNE_SAFE_PROFILE)
    # The no-CAN local executor needs a frozen, lifted-wheel calibration
    # baseline to pass Safety preflight. This is profile-only; it is not a
    # runtime host write and ordinary Debug/Release never enable it.
    if(NOT LOCAL_SESSION_SMOKE)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE BSP/autotune_safe_local.c)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
            AUTOTUNE_SAFE_PROFILE
            CALIBRATION_BENCH_DEFAULTS
        )
    endif()
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
