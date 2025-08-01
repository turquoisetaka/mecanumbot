cmake_minimum_required(VERSION 3.31)

set(PICO_BOARD pico2)
set(PICO_PLATFORM rp2350)
set(CMAKE_C_STANDARD 11)


include(pico_sdk_import.cmake)

project(MecanumBot)

pico_sdk_init()

add_executable(MecanumBot)

add_library(xinput_host INTERFACE)

target_sources(MecanumBot PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/hid_app.c
    ${CMAKE_CURRENT_LIST_DIR}/msc_app.c
    ${CMAKE_CURRENT_LIST_DIR}/cdc_app.c
    ${CMAKE_CURRENT_LIST_DIR}/tusb_xinput/xinput_host.c
)

target_include_directories(MecanumBot PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/tusb_xinput)


pico_enable_stdio_usb(MecanumBot 0)
pico_enable_stdio_uart(MecanumBot 1)

target_link_libraries(MecanumBot pico_stdlib 
tinyusb_host tinyusb_board 
hardware_timer hardware_pwm
)

pico_add_extra_outputs(MecanumBot)