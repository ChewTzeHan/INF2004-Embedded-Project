# This file goes in your src directory (e.g., /your_project/src/CMakeLists.txt)

if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    # Configuration WITH sockets (recommended for FreeRTOS)
    add_executable(picow_freertos_ping
            main.c                      # Main application file
            picow_freertos_ping.c       # TCP server functionality
            motor_encoder_demo.c        # Motor and encoder functionality
            ultrasonic.c                # Ultrasonic sensor functionality
            imu_raw_demo.c              # IMU sensor functionality
            ir_sensor.c                 # ADD THIS - IR sensor functionality
            encoder.c                   # ADD THIS - Digital encoder functionality
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
        target_compile_definitions(picow_freertos_ping PRIVATE
                WIFI_SSID=\"${WIFI_SSID}\"
                WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
                NO_SYS=0                    # Using FreeRTOS with sockets
                LWIP_SOCKET=1               # Enable socket API
                PING_USE_SOCKETS=1          # Use sockets for ping
                PICO_MAX_SHARED_IRQ_HANDLERS=8  # ADD THIS
                )
    target_include_directories(picow_freertos_ping PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(picow_freertos_ping
            pico_cyw43_arch_lwip_sys_freertos  # Use FreeRTOS version
            pico_stdlib
            pico_lwip_nosys
            FreeRTOS-Kernel-Heap4       # FreeRTOS kernel and dynamic heap
            hardware_adc
            hardware_pwm                # for PWM functionality
            hardware_gpio               # for GPIO functionality
            hardware_i2c                # for I2C functionality (IMU)
            pico_time                   # for timing functions
            m                           # Math library for IMU calculations
            )
    pico_enable_stdio_usb(picow_freertos_ping 1)
    pico_add_extra_outputs(picow_freertos_ping)
    
endif()