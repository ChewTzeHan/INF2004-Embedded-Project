# This file goes in your src directory (e.g., /your_project/src/CMakeLists.txt)

if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    # Configuration WITH sockets (recommended for FreeRTOS)
    add_executable(picow_freertos_ping
            main.c                      # Main application file
            mqtt_client.c              # MQTT client functionality
            ${PICO_LWIP_PATH}/src/apps/mqtt/mqtt.c  # Force-include lwIP MQTT app source
            PID_Line_Follow.c           # ADD THIS - PID line following functionality
                Obstacle_Avoidance.c        # ADD THIS - Obstacle avoidance functionality
                barcode.c
                IMU_movement.c
           # picow_freertos_ping.c       # TCP server functionality
            motor_encoder_demo.c        # Motor and encoder functionality
            ultrasonic.c                # Ultrasonic sensor functionality
            imu_raw_demo.c              # IMU sensor functionality
            ir_sensor.c                 # IR sensor functionality
            encoder.c                   # Digital encoder functionality
           # ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
        target_compile_definitions(picow_freertos_ping PRIVATE
                # WIFI_SSID=\"${WIFI_SSID}\"
                # WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
                WIFI_SSID="Galaxy S20 FE 5GD3A6"
                WIFI_PASSWORD="xqag4139"
                # WIFI_SSID="Starhub_0060"
                # WIFI_PASSWORD="fYKERJqzG2"
                NO_SYS=0                    # Using FreeRTOS with sockets
                #LWIP_SOCKET=1               # Enable socket API
                #PING_USE_SOCKETS=1          # Use sockets for ping
                PICO_MAX_SHARED_IRQ_HANDLERS=8

                BROKER_IP="192.168.63.104"        # <-- put YOUR broker IP here
                # BROKER_IP="192.168.0.7"
                BASE_TOPIC="robot/alpha"            # <-- or your preferred base
                MQTT_PORT=1883
                MQTT_CLIENT_ID="pico-car"
                MQTT_KEEPALIVE_S=30

                # Increase memory pools
                MEMP_NUM_SYS_TIMEOUT=20
                MEMP_NUM_NETCONN=8
                MEMP_NUM_TCP_PCB=8
                MEMP_NUM_TCP_PCB_LISTEN=4
                MEMP_NUM_TCP_SEG=32
                
                # Increase packet buffers
                PBUF_POOL_SIZE=16
                MEM_SIZE=16000
                TCP_MSS=1460
                TCP_SND_BUF=8192
                TCP_WND=8192
                
                # FreeRTOS settings
                LWIP_FREERTOS=1
                TCPIP_THREAD_STACKSIZE=2048
                DEFAULT_THREAD_STACKSIZE=1024
                                
                )
    target_include_directories(picow_freertos_ping PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
         #   ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(picow_freertos_ping
            pico_cyw43_arch_lwip_sys_freertos  # Use FreeRTOS version
            pico_stdlib
            #pico_lwip_nosys
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