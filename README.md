# INF2004 Embedded Project – Line-Following Robot with Obstacle Avoidance & MQTT

This project implements an autonomous line-following robot on a Raspberry Pi Pico W using FreeRTOS.

The system:

- Follows a line using IR sensors and PID control  
- Detects obstacles using an ultrasonic sensor and performs avoidance using encoders and IMU yaw  
- Scans simple barcodes on the track and converts them into turn commands (e.g. LEFT / RIGHT)  
- Connects to Wi-Fi and publishes telemetry over MQTT, which can be viewed on a simple web dashboard

---

## Folder and File Summary

### Root

- **CMakeLists.txt**  
  Top-level CMake configuration.  
  - Initializes the Pico SDK and FreeRTOS via `pico_sdk_import.cmake` and `FreeRTOS_Kernel_import.cmake`.  
  - Sets the project name (`picow_freertos_ping`).  
  - Defines Wi-Fi SSID and password (placeholders `Your_WiFi_SSID`, `Your_WiFi_Password`).  
  - Adds the `src/` directory with `add_subdirectory(src)`.

- **pico_sdk_import.cmake**  
  Helper script to import and initialize the Raspberry Pi Pico SDK (uses `PICO_SDK_PATH`).

- **FreeRTOS_Kernel_import.cmake**  
  Helper script to import the FreeRTOS Kernel into the build.

- **include/**  
  lwIP (TCP/IP stack) configuration headers:
  - **include/lwipopts.h** – lwIP configuration specific to this project (buffer sizes, timeouts, features).  
  - **include/lwipopts_examples_common.h** – Common lwIP configuration used by Pico W examples.

- **.vscode/**  
  Visual Studio Code project configuration:
  - `cmake-kits.json`, `c_cpp_properties.json`, `extensions.json`, `launch.json`, `settings.json`, `tasks.json`.

- **.gitignore**  
  Git ignore rules for build artifacts and temporary files.

---

### src/ – Application Code

- **src/CMakeLists.txt**  
  CMake configuration for the firmware executable `picow_freertos_ping`.  
  - Adds `main.c` and all required modules from `algorithms/`, `drivers/`, `navigation/`, `network/`, `system/`.  
  - Force-includes the lwIP MQTT app source (`mqtt.c`).  
  - Links the Pico SDK libraries, lwIP, FreeRTOS Kernel, and hardware drivers (ADC, PWM, GPIO, I2C, timing).

- **src/main.c**  
  Main application entry point.  
  - Initializes hardware (motors, encoders, IR sensor, ultrasonic sensor, IMU).  
  - Creates FreeRTOS tasks for:
    - Line following / main control  
    - Obstacle detection and avoidance  
    - Barcode scanning and command execution  
    - IMU-based movement/telemetry  
    - Wi-Fi and MQTT connectivity  
  - Coordinates the robot’s operating modes (line following, avoiding obstacles, reacting to barcodes).

- **src/EMBEDDED CODE CLEANED.zip**  
  Archived copy of cleaned embedded code (not used in the current build).

---

### src/algorithms/ – High-Level Robot Logic

- **src/algorithms/PID_Line_Follow.c**  
  PID-based line-following logic.  
  - Reads IR sensor values and calculates line position error.  
  - Applies PID control to generate a correction term.  
  - Maps the correction to left/right motor speeds.

- **src/algorithms/PID_Line_Follow.h**  
  Declarations for PID line-following initialization and update functions.

- **src/algorithms/Obstacle_Avoidance.c**  
  Obstacle detection and avoidance behaviour.  
  - Uses ultrasonic sensor readings to detect obstacles.  
  - Uses wheel encoders and IMU yaw to navigate around obstacles.  
  - Provides helpers to:
    - Check whether an obstacle is present  
    - Run avoidance-only behaviour  
    - Track current speed and total distance

- **src/algorithms/Obstacle_Avoidance.h**  
  Declarations for obstacle detection/avoidance functions and telemetry helpers.

- **src/algorithms/barcode.c**  
  Barcode scanning and decoding.  
  - Samples the IR digital signal while moving over a barcode.  
  - Converts timing of black/white segments into narrow/wide patterns.  
  - Decodes patterns into characters and maps them to robot commands (LEFT, RIGHT, etc.).  

- **src/algorithms/barcode.h**  
  Declarations for barcode scan/decoding APIs and command mapping.

---

### src/drivers/ – Hardware Drivers

#### Motor / Encoders

- **src/drivers/motor/encoder.c**  
  Wheel encoder driver.  
  - Initializes encoder pins.  
  - Measures pulse durations/periods.  
  - Computes wheel speed in cm/s and distance in cm.  
  - Exposes functions to read pulse counts and reset distance.

- **src/drivers/motor/encoder.h**  
  Declarations for encoder initialization and speed/distance measurement.

- **src/drivers/motor/motor_encoder_demo.c**  
  Motor and encoder control module.  
  - Initializes motors and provides functions to set motor speeds.  
  - Used by higher-level logic (line following, obstacle avoidance) for driving the wheels.

- **src/drivers/motor/motor_encoder_demo.h**  
  Declarations for motor/encoder helper functions.

#### Sensors

- **src/drivers/sensors/ir_sensor.c**  
  IR sensor driver.  
  - Reads analog/digital values from the line sensor.  
  - Provides functions to detect the line and to feed barcode scanning logic.

- **src/drivers/sensors/ir_sensor.h**  
  Pin definitions, thresholds, and function declarations for IR sensor access.

- **src/drivers/sensors/ultrasonic.c**  
  Ultrasonic distance sensor driver (e.g. HC-SR04).  
  - Configures trigger/echo pins.  
  - Sends trigger pulses and measures echo to compute distance in centimeters.  
  - Provides a fast boolean check for “obstacle within range”.

- **src/drivers/sensors/ultrasonic.h**  
  Declarations for ultrasonic initialization and distance measurement functions.

- **src/drivers/sensors/imu_raw_demo.c**  
  IMU driver and demo utilities.  
  - Initializes the IMU (accelerometer/magnetometer).  
  - Reads raw IMU data (acceleration, magnetic field).  
  - Computes basic heading/yaw and prints data for debugging when logging is enabled.

- **src/drivers/sensors/imu_raw_demo.h**  
  Declarations for IMU initialization and raw-data reading/demo functions.

#### Time

- **src/drivers/time/platform_time_pico.c**  
  Simple time abstraction for the Pico platform.  
  - Provides timing utilities based on Pico/FreeRTOS (e.g. milliseconds).

---

### src/navigation/ – IMU-Based Movement

- **src/navigation/IMU_movement.c**  
  IMU-based movement and yaw control.  
  - Sets up IMU and PID structures for yaw regulation.  
  - Reads IMU data and computes yaw error.  
  - Uses PID to drive motors towards a yaw setpoint.  
  - Tracks speed and distance over time.

- **src/navigation/IMU_movement.h**  
  Declarations for IMU movement initialization, PID helpers, and the IMU movement FreeRTOS task.

---

### src/network/ – Wi-Fi & MQTT

- **src/network/mqtt_client.c**  
  Wi-Fi and MQTT client logic.  
  - Connects to Wi-Fi using the Pico W’s CYW43 interface with credentials defined in the build.  
  - Connects to an MQTT broker (broker IP and topic configured in the source).  
  - Publishes telemetry such as speed, distance, yaw, ultrasonic distance, and robot state.  
  - Provides functions to check MQTT connection status and publish messages.

- **src/network/mqtt_client.h**  
  Declarations for MQTT setup and publish helpers.

- **src/network/picow_freertos_ping.c**  
  Networking example adapted from the Pico W FreeRTOS ping demo.  
  - Demonstrates TCP/network integration with lwIP and FreeRTOS.

- **src/network/picow_freertos_ping.h**  
  Declarations for the ping/example networking API.

---

### src/system/ – System Configuration & Logging

- **src/system/FreeRTOSConfig.h**  
  FreeRTOS configuration.  
  - Task priorities, stack sizes, tick rate, heap implementation, and other kernel settings tailored for this project.

- **src/system/log.h**  
  Logging macros used throughout the project.  
  - `LOG_INFO(...)` – debug logging macro, mapped to `printf` when enabled and compiled out when disabled.  
  - `snLOG_INFO(...)` – wrapper around `snprintf(...)` for building strings.

---

### src/dashboard/ – Web Dashboard

- **src/dashboard/index.html**  
  HTML/JavaScript dashboard page.  
  - Connects to an MQTT broker via WebSockets and subscribes to robot telemetry topics.  
  - Displays live telemetry such as sensor values, state, and other data published by the firmware.

- **src/dashboard/readme.txt**  
  Notes on MQTT topics and sample broker/WebSocket usage for the dashboard.

---

## How to Compile

### Requirements

- Raspberry Pi Pico SDK installed and `PICO_SDK_PATH` set  
- FreeRTOS kernel (imported by `FreeRTOS_Kernel_import.cmake`)  
- ARM GCC toolchain supported by the Pico SDK  
- CMake (version 3.13 or later)

Wi-Fi SSID and password defaults are defined in the root `CMakeLists.txt` as `WIFI_SSID` and `WIFI_PASSWORD`.  
MQTT parameters (broker IP, base topic, client ID, port) are defined in `src/network/mqtt_client.c`.

### Build Steps (command line)

From the project root (where `CMakeLists.txt` is located):

```bash
mkdir -p build
cd build

cmake ..
cmake --build . --config Release
```

If the environment is set up correctly, the build will produce a UF2 file for the firmware (e.g. `picow_freertos_ping.uf2`) in the build directory.

### Flashing to Pico W

1. Hold the **BOOTSEL** button on the Raspberry Pi Pico W.  
2. Connect the board to the PC via USB.  
3. A mass-storage device (e.g. `RPI-RP2`) will appear.  
4. Copy the generated `.uf2` file from the `build/` directory to the `RPI-RP2` drive.  
5. The board will reboot and start running the firmware.

---

## How to Run and Test

### 1. Robot Behaviour

1. Power the Pico W running the firmware on the robot.  
2. Place the robot on a track with a clear line and, optionally, barcodes.  
3. As it runs:
   - IR-based PID control keeps the robot following the line.  
   - The ultrasonic sensor detects obstacles; obstacle avoidance routines run to navigate around them and rejoin the line.  
   - The barcode module scans barcodes on the track and converts them into turn commands at junctions.

### 2. MQTT Telemetry

1. Ensure an MQTT broker is running and reachable at the IP and port configured in `src/network/mqtt_client.c`.  
2. After the Pico W connects to Wi-Fi and the broker, it will publish telemetry under the configured base topic.  
3. You can subscribe to the relevant topics with an MQTT client (e.g. `mosquitto_sub`) to view sensor and state data.

Example (topic values depend on the configuration in `mqtt_client.c`):

```bash
mosquitto_sub -h <BROKER_IP> -p <MQTT_PORT> -t "<BASE_TOPIC>/#" -v
```

### 3. Web Dashboard

1. Make sure the MQTT broker is accessible via WebSockets if required by your setup.  
2. Open `src/dashboard/index.html` in a web browser.  
3. Configure the connection parameters in the page to match the MQTT/WebSocket setup.  
4. Once connected, the dashboard will display live telemetry from the robot based on the topics it subscribes to.

---
