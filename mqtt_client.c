// #include <stdio.h>
// #include <string.h>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "lwip/apps/mqtt.h"
// #include "lwip/dns.h"
// #include "lwip/ip_addr.h"

// #include <inttypes.h> 

// // ======= Wi-Fi credentials =======
// #define WIFI_SSID      "Fata nos in unum"
// #define WIFI_PASSWORD  "rosemary"
// // =================================

// // ======= EDIT THESE TWO TO MATCH YOUR SETUP =======
// #define BROKER_IP   "192.168.134.191"      // your laptop's IP (broker)
// #define BASE_TOPIC  "robot/alpha"          // e.g., robot/alpha
// // ==================================================

// static mqtt_client_t *g_client = NULL;
// static bool g_mqtt_connected = false;

// /* ---------------- MQTT callbacks ---------------- */
// static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
//     // payload chunk(s). For simplicity we assume small messages.
//     static char buf[256];
//     size_t n = (len < sizeof(buf)-1) ? len : sizeof(buf)-1;
//     memcpy(buf, data, n); buf[n] = '\0';
//     printf("[MQTT] Incoming data: %s\n", buf);

//     // Expect commands like: {"type":"barcode","value":"LEFT"}
//     // can parse here and route to the motion / barcode handler.
// }

// static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
//     printf("[MQTT] Incoming publish on topic: %s (len=%"PRIu32")\n", topic, tot_len);
// }

// static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
//     if (status == MQTT_CONNECT_ACCEPTED) {
//         g_mqtt_connected = true;
//         printf("[MQTT] conn status=%d\n", status);
//         printf("[MQTT] Connected.\n");

//         // subscribe to {base}/cmd
//         char topic[128];
//         snprintf(topic, sizeof(topic), "%s/cmd", BASE_TOPIC);
//         mqtt_sub_unsub(client, topic, 1, NULL, NULL, 1);
//         printf("[MQTT] Subscribed to %s\n", topic);

//         // register incoming handlers
//         mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
//     } else {
//         g_mqtt_connected = false;
//         printf("[MQTT] Disconnected (status=%d)\n", status);
//     }
// }

// /* -------------- Public API ---------------------- */
// bool wifi_and_mqtt_start(void) {
//     // Init Wi-Fi stack (threadsafe background is recommended)
//     if (cyw43_arch_init()) {
//         printf("cyw43 init failed\n");
//         return false;
//     }
//     cyw43_arch_enable_sta_mode();

//     printf("Connecting Wi-Fi…\n");
//     if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
//             CYW43_AUTH_WPA2_AES_PSK, 30000)) {
//         printf("Wi-Fi connect failed\n");
//         return false;
//     }
//     printf("Wi-Fi connected. IP broker: %s\n", BROKER_IP);

//     // Resolve/convert broker IP
//     ip_addr_t ip;
//     if (!ipaddr_aton(BROKER_IP, &ip)) {
//         printf("Invalid BROKER_IP\n");
//         return false;
//     }

//     g_client = mqtt_client_new();
//     if (!g_client) {
//         printf("mqtt_client_new failed\n");
//         return false;
//     }

//     struct mqtt_connect_client_info_t ci;
//     memset(&ci, 0, sizeof(ci));
//     ci.client_id  = "pico-car";
//     ci.keep_alive = 30;
//     ci.client_user = NULL;   // set if using auth
//     ci.client_pass = NULL;

//     printf("Connecting MQTT…\n");
//     cyw43_arch_lwip_begin();
//     err_t err = mqtt_client_connect(g_client, &ip, 1883, mqtt_connection_cb, NULL, &ci);
//     cyw43_arch_lwip_end();
//     if (err != ERR_OK) {
//         printf("mqtt_client_connect err=%d\n", err);
//         return false;
//     }
//     return true;
// }

// bool mqtt_is_connected(void) { return g_mqtt_connected; }

// void mqtt_publish_telemetry(float speed, float distance_cm, float yaw_deg, float ultra_cm, const char* state) {
//     if (!g_client || !g_mqtt_connected) return;

//     char topic[128];
//     char payload[256];

//     // JSON matches your dashboard
//     snprintf(payload, sizeof(payload),
//              "{\"speed\":%.2f,\"distance\":%.2f,\"imu\":{\"yaw\":%.2f},\"ultra_cm\":%.2f,\"state\":\"%s\"}",
//              speed, distance_cm, yaw_deg, ultra_cm, state ? state : "idle");

//     snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);

//     cyw43_arch_lwip_begin();
//     err_t err = mqtt_publish(g_client, topic, payload, strlen(payload),
//                              1 /*QoS1*/, 0 /*retain*/, NULL, NULL);

//     cyw43_arch_lwip_end();
    
//     if (err != ERR_OK) {
//         printf("[MQTT] publish err=%d\n", err);
//     }
// }

// // If not using FreeRTOS, call this periodically to let lwIP timers run
// void mqtt_loop_poll(void) {
//     // With pico_cyw43_arch_lwip_threadsafe_background, no explicit poll is needed.
//     // Still, a small sleep yields CPU.
//     sleep_ms(1);
// }

// mqtt_client.c — Pico W + FreeRTOS (sys_freertos) safe MQTT helper
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/err.h"

#include "mqtt_client.h"

// ---------- User/Build configuration (can be -D… in CMake) ----------
#ifndef WIFI_SSID
#define WIFI_SSID       "YOUR_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "YOUR_PASSWORD"
#endif
#ifndef BROKER_IP
// Plain TCP MQTT broker on your laptop / Raspberry Pi etc.
#define BROKER_IP       "192.168.1.10"
#endif
#ifndef BASE_TOPIC
#define BASE_TOPIC      "robot/alpha"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT       1883
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID  "pico-car"
#endif
#ifndef MQTT_KEEPALIVE_S
#define MQTT_KEEPALIVE_S 60
#endif
// --------------------------------------------------------------------

static mqtt_client_t *g_client = NULL;
static volatile bool g_mqtt_connected = false;

// Forward
static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

static void log_ip(const char *label, const ip_addr_t *ip) {
    printf("%s %s\n", label, ipaddr_ntoa(ip));
}

bool mqtt_is_connected(void) {
    return g_mqtt_connected;
}

/* =============================
 * MQTT incoming handlers
 * ============================= */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    // This callback runs in the lwIP TCPIP thread
    printf("[MQTT] <- PUBLISH topic=\"%s\" len=%" PRIu32 "\n", topic ? topic : "(null)", tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // This callback runs in the lwIP TCPIP thread
    static char buf[384];
    size_t n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = '\0';
    printf("[MQTT] <- DATA: %s%s\n", buf, (flags & MQTT_DATA_FLAG_LAST) ? " [LAST]" : "");
    // Parse commands here if you want (e.g. {"type":"barcode","value":"LEFT"})
}

/* =============================
 * MQTT connection callback
 * ============================= */
// static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
//     printf("[MQTT] conn cb status=%d\n", status);
//     // This callback runs in the lwIP TCPIP thread (already inside the lock)
//     if (status == MQTT_CONNECT_ACCEPTED) {
//          printf("[MQTT] CONNECTED (keepalive=%d, clientId=%s)\n", MQTT_KEEPALIVE_S, MQTT_CLIENT_ID);
//         // g_mqtt_connected = true;
//         // printf("[MQTT] CONNECTED (status=%d)\n", status);

//         // Subscribe to {BASE_TOPIC}/cmd (QoS 1)
//         // char topic[128];
//         // snprintf(topic, sizeof(topic), "%s/cmd", BASE_TOPIC);
//         // err_t e = mqtt_sub_unsub(client, topic, 1 /*qos*/, NULL, NULL, 1 /*subscribe*/);
//         // if (e == ERR_OK) {
//         //     printf("[MQTT] SUBSCRIBED %s\n", topic);
//         // } else {
//         //     printf("[MQTT] subscribe err=%d\n", e);
//         // }
//         const char *diag = "{\"hello\":\"pico-online\"}";
//     char dtopic[128]; snprintf(dtopic, sizeof(dtopic), "%s/diag", BASE_TOPIC);
//     cyw43_arch_lwip_begin();
//     err_t de = mqtt_publish(client, dtopic, diag, strlen(diag), 0, 1, NULL, NULL);
//     cyw43_arch_lwip_end();
//     printf("[MQTT] diag publish -> %s (retained) err=%d\n", dtopic, (int)de);

//         // Register incoming handlers
//         mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

//     } else {
//         g_mqtt_connected = false;
//         printf("[MQTT] DISCONNECTED (status=%d)\n", status);
//     }
// }

static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    printf("[MQTT] conn cb status=%d\n", status);
    if (status == MQTT_CONNECT_ACCEPTED) {
        g_mqtt_connected = true;
        printf("[MQTT] CONNECTED (keepalive=%d, clientId=%s)\n", MQTT_KEEPALIVE_S, MQTT_CLIENT_ID);

        // Subscribe to {BASE_TOPIC}/cmd (QoS 1)
        char topic[128];
        snprintf(topic, sizeof(topic), "%s/cmd", BASE_TOPIC);
        err_t e = mqtt_sub_unsub(client, topic, 1, NULL, NULL, 1);
        printf("[MQTT] subscribe %s => err=%d\n", topic, (int)e);

        // Optional: retained diag so web can see the device is online
        const char *diag = "{\"hello\":\"pico-online\"}";
        char dtopic[128]; snprintf(dtopic, sizeof(dtopic), "%s/diag", BASE_TOPIC);
        cyw43_arch_lwip_begin();
        err_t de = mqtt_publish(client, dtopic, diag, (u16_t)strlen(diag), 0, 1, NULL, NULL);
        cyw43_arch_lwip_end();
        printf("[MQTT] diag publish -> %s (retained) err=%d\n", dtopic, (int)de);

        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    } else {
        g_mqtt_connected = false;
        printf("[MQTT] DISCONNECTED (status=%d)\n", status);
    }
}


/* =============================
 * Public: connect Wi-Fi + MQTT
 * ============================= */
bool wifi_and_mqtt_start(void) {
    // Init CYW43 and lwIP/FreeRTOS integration
    printf("ATTEMPTING WIFI AND MQTT START\n");
    if (cyw43_arch_init()) {
        printf("[NET] cyw43_arch_init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("[CFG] BROKER_IP macro: %s\n", BROKER_IP);


    printf("[NET] Connecting Wi-Fi SSID=\"%s\"…\n", WIFI_SSID);
    int r = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000000);
    if (r) {
        printf("[NET] Wi-Fi connect failed (err=%d)\n", r);
        return false;
    }
    printf("[NET] Wi-Fi connected.\n");

    // Resolve broker IP string
    ip_addr_t broker_ip;
    if (!ipaddr_aton(BROKER_IP, &broker_ip)) {
        printf("[MQTT] Invalid BROKER_IP string: %s\n", BROKER_IP);
        return false;
    }
    log_ip("[MQTT] Broker:", &broker_ip);

    // Create MQTT client
    cyw43_arch_lwip_begin();
    g_client = mqtt_client_new();
    cyw43_arch_lwip_end();
    if (!g_client) {
        printf("[MQTT] mqtt_client_new failed\n");
        return false;
    }

    // Prepare client info
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id  = MQTT_CLIENT_ID;
    ci.keep_alive = MQTT_KEEPALIVE_S;
    // ci.client_user = "user"; ci.client_pass = "pass"; // if your broker needs auth

    printf("[MQTT] Connecting to %s:%d as \"%s\"…\n", BROKER_IP, MQTT_PORT, MQTT_CLIENT_ID);

    // Connect (must be under lwIP lock from FreeRTOS side)
    err_t err;
    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(g_client, &broker_ip, MQTT_PORT, mqtt_conn_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[MQTT] mqtt_client_connect err=%d\n", err);
        return false;
    }
    return true;
}

/* =============================
 * Public: publish telemetry JSON
 * Safe for FreeRTOS tasks.
 * ============================= */
// void mqtt_publish_telemetry(float speed, float distance_cm, float yaw_deg,
//                             float ultra_cm, const char *state) {
//     if (!g_client || !g_mqtt_connected) { printf("[MQTT] skip publish (not connected)\n"); return; }

//     char topic[128];
//     char payload[256];

//     char topic[128]; snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);
// printf("[MQTT] -> %s : %s\n", topic, payload);
// cyw43_arch_lwip_begin();
// err_t e = mqtt_publish(g_client, topic, payload, (u16_t)strlen(payload), 1, 0, NULL, NULL);
// cyw43_arch_lwip_end();
// if (e != ERR_OK) printf("[MQTT] publish err=%d\n", (int)e);

//     // Build JSON to match your web dashboard
//     snprintf(payload, sizeof(payload),
//              "{\"speed\":%.2f,\"distance\":%.2f,\"imu\":{\"yaw\":%.2f},"
//              "\"ultra_cm\":%.2f,\"state\":\"%s\"}",
//              speed, distance_cm, yaw_deg, ultra_cm, state ? state : "idle");

//     snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);
//         // Print debug info
//     printf("[MQTT DEBUG] Topic: %s\n", topic);
//     printf("[MQTT DEBUG] Payload: %s\n", payload);

//     // // Publish telemetry
//     // snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);
//     // err_t err = mqtt_publish(g_client, topic, payload, strlen(payload),
//     //                         1 /* QoS1 */, 0 /* retain */, NULL, NULL);
//     // if (err != ERR_OK) {
//     //     printf("[MQTT] publish err=%d\n", err);
//     // } else {
//     //     printf("[MQTT] publish OK\n");
//     // }

//    // snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);

//     // Publish with QoS 1, non-retained — under lwIP lock
//     cyw43_arch_lwip_begin();
//     // err_t e = mqtt_publish(g_client,
//     //                        topic,
//     //                        payload,
//     //                        (u16_t)strlen(payload),
//     //                        1 /* QoS1 */,
//     //                        0 /* retain */,
//     //                        NULL, NULL);
//     err_t e = mqtt_publish(g_client, topic, payload, (u16_t)strlen(payload),
//                            1, 0, NULL, NULL);
//     cyw43_arch_lwip_end();

//     if (e != ERR_OK) {
//         printf("[MQTT] publish err=%d\n", e);
//     }
// }

void mqtt_publish_telemetry(float speed, float distance_cm, float yaw_deg,
                            float ultra_cm, const char *state) {
    if (!g_client || !g_mqtt_connected) { 
        printf("[MQTT] skip publish (not connected)\n"); 
        return; 
    }

    char topic[128];
    char payload[256];

    // Build JSON first
    snprintf(payload, sizeof(payload),
             "{\"speed\":%.2f,\"distance\":%.2f,\"imu\":{\"yaw\":%.2f},"
             "\"ultra_cm\":%.2f,\"state\":\"%s\"}",
             speed, distance_cm, yaw_deg, ultra_cm, state ? state : "idle");

    snprintf(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);

    // Loud debug
    printf("[MQTT] -> topic=\"%s\" bytes=%u\n", topic, (unsigned)strlen(payload));
    printf("[MQTT] payload: %s\n", payload);

    // Single publish under lwIP lock
    cyw43_arch_lwip_begin();
    err_t e = mqtt_publish(g_client, topic, payload, (u16_t)strlen(payload),
                           1 /*QoS1*/, 0 /*retain*/, NULL, NULL);
    cyw43_arch_lwip_end();

    if (e != ERR_OK) {
        printf("[MQTT] publish err=%d\n", (int)e);
    } else {
        printf("[MQTT] publish OK\n");
    }
}


/* =============================
 * Optional: non-RTOS polling
 * (not needed for sys_freertos,
 *  but kept to match header)
 * ============================= */
void mqtt_loop_poll(void) {
    // With pico_cyw43_arch_lwip_sys_freertos, lwIP runs on its own thread.
    // Nothing to do here; brief sleep to yield CPU if someone calls it anyway.
    sleep_ms(1);
}
