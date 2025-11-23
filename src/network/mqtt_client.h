#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool wifi_and_mqtt_start(void);     // connect Wi-Fi + MQTT
void mqtt_publish_telemetry(float speed, float distance_cm, float yaw_deg, float ultra_cm, const char* state);
void mqtt_loop_poll(void);          // call periodically (non-RTOS)
bool mqtt_is_connected(void);
bool wifi_and_mqtt_start_nonblocking(void); // connect Wi-Fi + MQTT (non-blocking)

#ifdef __cplusplus
}
#endif
