# This file goes in your project root directory (e.g., /your_project/CMakeLists.txt)

cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

# Include FreeRTOS kernel
include(FreeRTOS_Kernel_import.cmake)

project(picow_freertos_ping C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Set Wi-Fi credentials
set(WIFI_SSID "Your_WiFi_SSID" CACHE STRING "Wi-Fi SSID")
set(WIFI_PASSWORD "Your_WiFi_Password" CACHE STRING "Wi-Fi Password")

# Add executable - this points to your src directory
add_subdirectory(src)