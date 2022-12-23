cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})


# Initialize the SDK
pico_sdk_init()


add_executable(main
        main.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(main pico_stdlib hardware_uart hardware_adc hardware_i2c hardware_irq hardware_pwm hardware_clocks)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# add url via pico_set_program_url
#example_auto_set_url(main)

pico_set_binary_type(main no_flash)
