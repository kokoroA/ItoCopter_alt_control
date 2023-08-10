cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ito_copter C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
include_directories(/Users/kokoro/Documents/drone_project/pico/eigen)

add_executable(ito_copter
pico_copter.cpp
sensor.cpp
ekf.cpp
lsm9ds1_reg.c
pwm.cpp
radio.cpp
control.cpp
vl53l1_platform.c
VL53L1X_api.c
VL53L1X_calibration.c
)

target_link_libraries(ito_copter 
  hardware_i2c
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  pico_stdlib
  pico_multicore)

pico_enable_stdio_usb(ito_copter 1)
pico_enable_stdio_uart(ito_copter 0)

pico_add_extra_outputs(ito_copter)

