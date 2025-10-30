Microsoft Windows [Version 10.0.22631.6060]
(c) Microsoft Corporation. All rights reserved.

C:\Windows\System32>sc query mosquitto

SERVICE_NAME: mosquitto
        TYPE               : 10  WIN32_OWN_PROCESS
        STATE              : 4  RUNNING
                                (STOPPABLE, NOT_PAUSABLE, ACCEPTS_SHUTDOWN)
        WIN32_EXIT_CODE    : 0  (0x0)
        SERVICE_EXIT_CODE  : 0  (0x0)
        CHECKPOINT         : 0x0
        WAIT_HINT          : 0x0

C:\Windows\System32>net stop mosquitto
The Mosquitto Broker service is stopping.
The Mosquitto Broker service was stopped successfully.


C:\Windows\System32>dir C:\mosquitto\mosquitto.conf
 Volume in drive C is OS
 Volume Serial Number is 3297-98D4

 Directory of C:\mosquitto

29/10/2025  12:07 pm               411 mosquitto.conf
               1 File(s)            411 bytes
               0 Dir(s)  80,866,078,720 bytes free

C:\Windows\System32>type C:\mosquitto\mosquitto.conf
# --- basic persistence & logging ---
persistence true
persistence_location C:\mosquitto\data\
log_dest file C:\mosquitto\log\mosquitto.log
log_type all

# --- listeners ---
# Pico firmware connects over classic MQTT/TCP
listener 1883
protocol mqtt

# Browser dashboard connects over WebSockets
listener 9001
protocol websockets

# TEMP for local testing; switch off later
allow_anonymous true

C:\Windows\System32>mkdir C:\mosquitto\log
A subdirectory or file C:\mosquitto\log already exists.

C:\Windows\System32>mkdir C:\mosquitto\data
A subdirectory or file C:\mosquitto\data already exists.

C:\Windows\System32>"C:\Program Files\mosquitto\mosquitto.exe" -v -c "C:\mosquitto\mosquitto.conf"
1761712153: mosquitto version 2.0.22 starting
1761712153: Config loaded from C:\mosquitto\mosquitto.conf.
1761712153: Opening ipv6 listen socket on port 1883.
1761712153: Error: Only one usage of each socket address (protocol/network address/port) is normally permitted.


C:\Windows\System32>net stop mosquitto
The Mosquitto Broker service is not started.

More help is available by typing NET HELPMSG 3521.


C:\Windows\System32>taskkill /F /IM mosquitto.exe
SUCCESS: The process "mosquitto.exe" with PID 21060 has been terminated.

C:\Windows\System32>netstat -ano | find ":1883"

C:\Windows\System32>netstat -ano | find ":9001"
  TCP    127.0.0.1:55606        127.0.0.1:9001         SYN_SENT        12596
  TCP    [::1]:9001             [::1]:49332            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:50429            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:51055            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:51436            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:51942            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:52206            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:52322            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:52326            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:52809            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:54364            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:54689            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:55135            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:55689            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:56205            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:56383            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:57026            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:57191            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:58408            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:58650            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:58800            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:59108            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:59170            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:59315            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:59645            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:60010            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:60095            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:60618            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:60670            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:61655            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:61703            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:61764            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:62212            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:62611            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:62643            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:63705            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:63870            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:64259            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:64567            TIME_WAIT       0
  TCP    [::1]:9001             [::1]:65197            TIME_WAIT       0
  TCP    [::1]:62680            [::1]:9001             SYN_SENT        12596

C:\Windows\System32>"C:\Program Files\mosquitto\mosquitto.exe" -v -c "C:\mosquitto\mosquitto.conf"
1761712536: mosquitto version 2.0.22 starting
1761712536: Config loaded from C:\mosquitto\mosquitto.conf.
1761712536: Opening ipv6 listen socket on port 1883.
1761712536: Opening ipv4 listen socket on port 1883.
1761712536: Opening websockets listen socket on port 9001.
1761712536: mosquitto version 2.0.22 running
1761712536: New client connected from ::1:58206 as web-dash-b9808763f2986 (p2, c1, k60).
1761712536: No will message specified.
1761712536: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712536: Received SUBSCRIBE from web-dash-b9808763f2986
1761712536:     robot/alpha/telemetry (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712536:     robot/alpha/speed (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/speed
1761712536:     robot/alpha/distance (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/distance
1761712536:     robot/alpha/imu/yaw (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712536:     robot/alpha/imu/pitch (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712536:     robot/alpha/imu/roll (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712536:     robot/alpha/ir/raw (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712536:     robot/alpha/line/pos (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712536:     robot/alpha/ultra/cm (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712536:     robot/alpha/state (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/state
1761712536:     robot/alpha/barcode (QoS 1)
1761712536: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712536: Sending SUBACK to web-dash-b9808763f2986
1761712536: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712536: New client connected from ::1:52267 as web-dash-b9808763f2986 (p2, c1, k60).
1761712536: No will message specified.
1761712536: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712536: Client web-dash-b9808763f2986 closed its connection.
1761712537: Received SUBSCRIBE from web-dash-b9808763f2986
1761712537:     robot/alpha/telemetry (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712537:     robot/alpha/speed (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/speed
1761712537:     robot/alpha/distance (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/distance
1761712537:     robot/alpha/imu/yaw (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712537:     robot/alpha/imu/pitch (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712537:     robot/alpha/imu/roll (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712537:     robot/alpha/ir/raw (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712537:     robot/alpha/line/pos (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712537:     robot/alpha/ultra/cm (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712537:     robot/alpha/state (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/state
1761712537:     robot/alpha/barcode (QoS 1)
1761712537: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712537: Sending SUBACK to web-dash-b9808763f2986
1761712539: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712539: New client connected from ::1:53903 as web-dash-b9808763f2986 (p2, c1, k60).
1761712539: No will message specified.
1761712539: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712539: Client web-dash-b9808763f2986 closed its connection.
1761712540: Received SUBSCRIBE from web-dash-b9808763f2986
1761712540:     robot/alpha/telemetry (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712540:     robot/alpha/speed (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/speed
1761712540:     robot/alpha/distance (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/distance
1761712540:     robot/alpha/imu/yaw (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712540:     robot/alpha/imu/pitch (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712540:     robot/alpha/imu/roll (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712540:     robot/alpha/ir/raw (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712540:     robot/alpha/line/pos (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712540:     robot/alpha/ultra/cm (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712540:     robot/alpha/state (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/state
1761712540:     robot/alpha/barcode (QoS 1)
1761712540: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712540: Sending SUBACK to web-dash-b9808763f2986
1761712542: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712542: New client connected from ::1:62564 as web-dash-b9808763f2986 (p2, c1, k60).
1761712542: No will message specified.
1761712542: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712542: Client web-dash-b9808763f2986 closed its connection.
1761712543: Received SUBSCRIBE from web-dash-b9808763f2986
1761712543:     robot/alpha/telemetry (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712543:     robot/alpha/speed (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/speed
1761712543:     robot/alpha/distance (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/distance
1761712543:     robot/alpha/imu/yaw (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712543:     robot/alpha/imu/pitch (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712543:     robot/alpha/imu/roll (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712543:     robot/alpha/ir/raw (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712543:     robot/alpha/line/pos (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712543:     robot/alpha/ultra/cm (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712543:     robot/alpha/state (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/state
1761712543:     robot/alpha/barcode (QoS 1)
1761712543: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712543: Sending SUBACK to web-dash-b9808763f2986
1761712545: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712545: New client connected from ::1:57911 as web-dash-b9808763f2986 (p2, c1, k60).
1761712545: No will message specified.
1761712545: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712545: Client web-dash-b9808763f2986 closed its connection.
1761712546: Received SUBSCRIBE from web-dash-b9808763f2986
1761712546:     robot/alpha/telemetry (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712546:     robot/alpha/speed (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/speed
1761712546:     robot/alpha/distance (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/distance
1761712546:     robot/alpha/imu/yaw (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712546:     robot/alpha/imu/pitch (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712546:     robot/alpha/imu/roll (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712546:     robot/alpha/ir/raw (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712546:     robot/alpha/line/pos (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712546:     robot/alpha/ultra/cm (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712546:     robot/alpha/state (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/state
1761712546:     robot/alpha/barcode (QoS 1)
1761712546: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712546: Sending SUBACK to web-dash-b9808763f2986
1761712548: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712548: New client connected from ::1:57790 as web-dash-b9808763f2986 (p2, c1, k60).
1761712548: No will message specified.
1761712548: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712548: Client web-dash-b9808763f2986 closed its connection.
1761712549: Received SUBSCRIBE from web-dash-b9808763f2986
1761712549:     robot/alpha/telemetry (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712549:     robot/alpha/speed (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/speed
1761712549:     robot/alpha/distance (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/distance
1761712549:     robot/alpha/imu/yaw (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712549:     robot/alpha/imu/pitch (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712549:     robot/alpha/imu/roll (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712549:     robot/alpha/ir/raw (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712549:     robot/alpha/line/pos (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712549:     robot/alpha/ultra/cm (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712549:     robot/alpha/state (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/state
1761712549:     robot/alpha/barcode (QoS 1)
1761712549: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712549: Sending SUBACK to web-dash-b9808763f2986
1761712553: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712553: New client connected from ::1:54916 as web-dash-b9808763f2986 (p2, c1, k60).
1761712553: No will message specified.
1761712553: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712553: Client web-dash-b9808763f2986 closed its connection.
1761712553: Received SUBSCRIBE from web-dash-b9808763f2986
1761712553:     robot/alpha/telemetry (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712553:     robot/alpha/speed (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/speed
1761712553:     robot/alpha/distance (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/distance
1761712553:     robot/alpha/imu/yaw (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712553:     robot/alpha/imu/pitch (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712553:     robot/alpha/imu/roll (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712553:     robot/alpha/ir/raw (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712553:     robot/alpha/line/pos (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712553:     robot/alpha/ultra/cm (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712553:     robot/alpha/state (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/state
1761712553:     robot/alpha/barcode (QoS 1)
1761712553: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712553: Sending SUBACK to web-dash-b9808763f2986
1761712558: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712558: New client connected from ::1:65247 as web-dash-b9808763f2986 (p2, c1, k60).
1761712558: No will message specified.
1761712558: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712558: Client web-dash-b9808763f2986 closed its connection.
1761712558: Received SUBSCRIBE from web-dash-b9808763f2986
1761712558:     robot/alpha/telemetry (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712558:     robot/alpha/speed (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/speed
1761712558:     robot/alpha/distance (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/distance
1761712558:     robot/alpha/imu/yaw (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712558:     robot/alpha/imu/pitch (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712558:     robot/alpha/imu/roll (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712558:     robot/alpha/ir/raw (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712558:     robot/alpha/line/pos (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712558:     robot/alpha/ultra/cm (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712558:     robot/alpha/state (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/state
1761712558:     robot/alpha/barcode (QoS 1)
1761712558: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712558: Sending SUBACK to web-dash-b9808763f2986
1761712563: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712563: New client connected from ::1:63249 as web-dash-b9808763f2986 (p2, c1, k60).
1761712563: No will message specified.
1761712563: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712563: Client web-dash-b9808763f2986 closed its connection.
1761712563: Received SUBSCRIBE from web-dash-b9808763f2986
1761712563:     robot/alpha/telemetry (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712563:     robot/alpha/speed (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/speed
1761712563:     robot/alpha/distance (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/distance
1761712563:     robot/alpha/imu/yaw (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712563:     robot/alpha/imu/pitch (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712563:     robot/alpha/imu/roll (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712563:     robot/alpha/ir/raw (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712563:     robot/alpha/line/pos (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712563:     robot/alpha/ultra/cm (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712563:     robot/alpha/state (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/state
1761712563:     robot/alpha/barcode (QoS 1)
1761712563: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712563: Sending SUBACK to web-dash-b9808763f2986
1761712567: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712567: New client connected from ::1:57904 as web-dash-b9808763f2986 (p2, c1, k60).
1761712567: No will message specified.
1761712567: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712567: Client web-dash-b9808763f2986 closed its connection.
1761712567: Received SUBSCRIBE from web-dash-b9808763f2986
1761712567:     robot/alpha/telemetry (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712567:     robot/alpha/speed (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/speed
1761712567:     robot/alpha/distance (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/distance
1761712567:     robot/alpha/imu/yaw (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712567:     robot/alpha/imu/pitch (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712567:     robot/alpha/imu/roll (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712567:     robot/alpha/ir/raw (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712567:     robot/alpha/line/pos (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712567:     robot/alpha/ultra/cm (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712567:     robot/alpha/state (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/state
1761712567:     robot/alpha/barcode (QoS 1)
1761712567: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712567: Sending SUBACK to web-dash-b9808763f2986
1761712569: Client web-dash-b9808763f2986 already connected, closing old connection.
1761712569: New client connected from ::1:59980 as web-dash-b9808763f2986 (p2, c1, k60).
1761712569: No will message specified.
1761712569: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712569: Client web-dash-b9808763f2986 closed its connection.
1761712569: Received SUBSCRIBE from web-dash-b9808763f2986
1761712569:     robot/alpha/telemetry (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712569:     robot/alpha/speed (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/speed
1761712569:     robot/alpha/distance (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/distance
1761712569:     robot/alpha/imu/yaw (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712569:     robot/alpha/imu/pitch (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712569:     robot/alpha/imu/roll (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712569:     robot/alpha/ir/raw (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712569:     robot/alpha/line/pos (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712569:     robot/alpha/ultra/cm (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712569:     robot/alpha/state (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/state
1761712569:     robot/alpha/barcode (QoS 1)
1761712569: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712569: Sending SUBACK to web-dash-b9808763f2986
1761712570: Client web-dash-b9808763f2986 closed its connection.
1761712581: New client connected from ::1:58127 as web-dash-b9808763f2986 (p2, c1, k60).
1761712581: No will message specified.
1761712581: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/telemetry (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/speed (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/speed
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/distance (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/distance
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/imu/yaw (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/imu/pitch (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/imu/roll (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/ir/raw (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/line/pos (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/ultra/cm (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/state (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/state
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712581: Received SUBSCRIBE from web-dash-b9808763f2986
1761712581:     robot/alpha/barcode (QoS 1)
1761712581: web-dash-b9808763f2986 1 robot/alpha/barcode
1761712581: Sending SUBACK to web-dash-b9808763f2986
1761712641: Received PINGREQ from web-dash-b9808763f2986
1761712641: Sending PINGRESP to web-dash-b9808763f2986
1761712650: New connection from ::1:61007 on port 1883.
1761712650: New client connected from ::1:61007 as auto-E35F261E-C7E5-C131-6E96-567F78757C9C (p2, c1, k60).
1761712650: No will message specified.
1761712650: Sending CONNACK to auto-E35F261E-C7E5-C131-6E96-567F78757C9C (0, 0)
1761712650: Received PUBLISH from auto-E35F261E-C7E5-C131-6E96-567F78757C9C (d0, q0, r0, m0, 'robot/alpha/telemetry', ... (91 bytes))
1761712650: Sending PUBLISH to web-dash-b9808763f2986 (d0, q0, r0, m0, 'robot/alpha/telemetry', ... (91 bytes))
1761712650: Received DISCONNECT from auto-E35F261E-C7E5-C131-6E96-567F78757C9C
1761712650: Client auto-E35F261E-C7E5-C131-6E96-567F78757C9C disconnected.
1761712701: Received PINGREQ from web-dash-b9808763f2986
1761712701: Sending PINGRESP to web-dash-b9808763f2986
1761712761: Received PINGREQ from web-dash-b9808763f2986
1761712761: Sending PINGRESP to web-dash-b9808763f2986
1761712821: Received PINGREQ from web-dash-b9808763f2986
1761712821: Sending PINGRESP to web-dash-b9808763f2986
1761712881: Received PINGREQ from web-dash-b9808763f2986
1761712881: Sending PINGRESP to web-dash-b9808763f2986
1761712941: Received PINGREQ from web-dash-b9808763f2986
1761712941: Sending PINGRESP to web-dash-b9808763f2986
1761713001: Received PINGREQ from web-dash-b9808763f2986
1761713001: Sending PINGRESP to web-dash-b9808763f2986
1761713061: Received PINGREQ from web-dash-b9808763f2986
1761713061: Sending PINGRESP to web-dash-b9808763f2986
1761713121: Received PINGREQ from web-dash-b9808763f2986
1761713121: Sending PINGRESP to web-dash-b9808763f2986
1761713181: Received PINGREQ from web-dash-b9808763f2986
1761713181: Sending PINGRESP to web-dash-b9808763f2986
1761713241: Received PINGREQ from web-dash-b9808763f2986
1761713241: Sending PINGRESP to web-dash-b9808763f2986
1761713301: Received PINGREQ from web-dash-b9808763f2986
1761713301: Sending PINGRESP to web-dash-b9808763f2986
1761713361: Received PINGREQ from web-dash-b9808763f2986
1761713361: Sending PINGRESP to web-dash-b9808763f2986
1761713421: Received PINGREQ from web-dash-b9808763f2986
1761713421: Sending PINGRESP to web-dash-b9808763f2986
1761713481: Received PINGREQ from web-dash-b9808763f2986
1761713481: Sending PINGRESP to web-dash-b9808763f2986
1761713541: Received PINGREQ from web-dash-b9808763f2986
1761713541: Sending PINGRESP to web-dash-b9808763f2986
1761713601: Received PINGREQ from web-dash-b9808763f2986
1761713601: Sending PINGRESP to web-dash-b9808763f2986
1761713662: Received PINGREQ from web-dash-b9808763f2986
1761713662: Sending PINGRESP to web-dash-b9808763f2986
1761713722: Received PINGREQ from web-dash-b9808763f2986
1761713722: Sending PINGRESP to web-dash-b9808763f2986
1761713782: Received PINGREQ from web-dash-b9808763f2986
1761713782: Sending PINGRESP to web-dash-b9808763f2986
1761713842: Received PINGREQ from web-dash-b9808763f2986
1761713842: Sending PINGRESP to web-dash-b9808763f2986
1761713866: Client web-dash-b9808763f2986 closed its connection.
1761713888: New connection from ::1:49674 on port 1883.
1761713888: New client connected from ::1:49674 as auto-20E4D262-2803-23BE-BCEE-57AAD4AF4283 (p2, c1, k60).
1761713888: No will message specified.
1761713888: Sending CONNACK to auto-20E4D262-2803-23BE-BCEE-57AAD4AF4283 (0, 0)
1761713888: Received PUBLISH from auto-20E4D262-2803-23BE-BCEE-57AAD4AF4283 (d0, q0, r0, m0, 'robot/alpha/telemetry', ... (91 bytes))
1761713888: Received DISCONNECT from auto-20E4D262-2803-23BE-BCEE-57AAD4AF4283
1761713888: Client auto-20E4D262-2803-23BE-BCEE-57AAD4AF4283 disconnected.
1761713891: New client connected from ::1:50405 as web-dash-b9808763f2986 (p2, c1, k60).
1761713891: No will message specified.
1761713891: Sending CONNACK to web-dash-b9808763f2986 (0, 0)
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/telemetry (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/telemetry
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/speed (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/speed
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/distance (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/distance
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/imu/yaw (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/imu/yaw
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/imu/pitch (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/imu/pitch
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/imu/roll (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/imu/roll
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/ir/raw (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/ir/raw
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/line/pos (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/line/pos
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/ultra/cm (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/ultra/cm
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/state (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/state
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713891: Received SUBSCRIBE from web-dash-b9808763f2986
1761713891:     robot/alpha/barcode (QoS 1)
1761713891: web-dash-b9808763f2986 1 robot/alpha/barcode
1761713891: Sending SUBACK to web-dash-b9808763f2986
1761713894: New connection from ::1:51388 on port 1883.
1761713894: New client connected from ::1:51388 as auto-EDC6B702-A1D6-6D0E-0260-2853D9354611 (p2, c1, k60).
1761713894: No will message specified.
1761713894: Sending CONNACK to auto-EDC6B702-A1D6-6D0E-0260-2853D9354611 (0, 0)
1761713894: Received PUBLISH from auto-EDC6B702-A1D6-6D0E-0260-2853D9354611 (d0, q0, r0, m0, 'robot/alpha/telemetry', ... (91 bytes))
1761713894: Sending PUBLISH to web-dash-b9808763f2986 (d0, q0, r0, m0, 'robot/alpha/telemetry', ... (91 bytes))
1761713894: Received DISCONNECT from auto-EDC6B702-A1D6-6D0E-0260-2853D9354611
1761713894: Client auto-EDC6B702-A1D6-6D0E-0260-2853D9354611 disconnected.
1761713951: Received PINGREQ from web-dash-b9808763f2986
1761713951: Sending PINGRESP to web-dash-b9808763f2986
1761714011: Received PINGREQ from web-dash-b9808763f2986
1761714011: Sending PINGRESP to web-dash-b9808763f2986

-----------------
CMD:
"C:\Program Files\mosquitto\mosquitto_pub.exe" -h localhost -p 1883 -t robot/alpha/telemetry -m "{\"speed\":15,\"distance\":120,\"imu\":{\"yaw\":45},\"ir_raw\":2048,\"ultra_cm\":28.5,\"state\":\"moving\"}"
ipconfig





website:
ws://localhost:9001
ws://<Wireless LAN adapter Wi-Fi: IPv4 Address. >:9001