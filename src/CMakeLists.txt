set(TARGET ${PROJECT_NAME})
add_executable(${TARGET} main.c usb_descriptors.c)

target_include_directories(${TARGET} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    pico_multicore
    tinyusb_device
    tinyusb_board
    hardware_adc
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# disable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 0)