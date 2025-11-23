#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/err.h"

#include "log.h"
#include "mqtt_client.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

// ---------- User/Build configuration (can be -D... in CMake) ----------
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
    LOG_INFO("%s %s\n", label, ipaddr_ntoa(ip));
}

bool mqtt_is_connected(void) {
    return g_mqtt_connected;
}

/* =============================
 * MQTT incoming handlers
 * ============================= */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    // This callback runs in the lwIP TCPIP thread
    LOG_INFO("[MQTT] <- PUBLISH topic=\"%s\" len=%" PRIu32 "\n", topic ? topic : "(null)", tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // This callback runs in the lwIP TCPIP thread
    static char buf[384];
    size_t n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = '\0';
    LOG_INFO("[MQTT] <- DATA: %s%s\n", buf, (flags & MQTT_DATA_FLAG_LAST) ? " [LAST]" : "");
    // Parse commands here if you want (e.g. {"type":"barcode","value":"LEFT"})
}

static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    LOG_INFO("[MQTT] conn cb status=%d\n", status);
    if (status == MQTT_CONNECT_ACCEPTED) {
        g_mqtt_connected = true;
        LOG_INFO("[MQTT] CONNECTED (keepalive=%d, clientId=%s)\n", MQTT_KEEPALIVE_S, MQTT_CLIENT_ID);

        // Subscribe to {BASE_TOPIC}/cmd (QoS 1)
        char topic[128];
        snLOG_INFO(topic, sizeof(topic), "%s/cmd", BASE_TOPIC);
        err_t e = mqtt_sub_unsub(client, topic, 1, NULL, NULL, 1);
        LOG_INFO("[MQTT] subscribe %s => err=%d\n", topic, (int)e);

        // Optional: retained diag so web can see the device is online
        const char *diag = "{\"hello\":\"pico-online\"}";
        char dtopic[128]; snLOG_INFO(dtopic, sizeof(dtopic), "%s/diag", BASE_TOPIC);
        cyw43_arch_lwip_begin();
        err_t de = mqtt_publish(client, dtopic, diag, (u16_t)strlen(diag), 0, 1, NULL, NULL);
        cyw43_arch_lwip_end();
        LOG_INFO("[MQTT] diag publish -> %s (retained) err=%d\n", dtopic, (int)de);

        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    } else {
        g_mqtt_connected = false;
        LOG_INFO("[MQTT] DISCONNECTED (status=%d)\n", status);
    }
}


/* =============================
 * Public: connect Wi-Fi + MQTT
 * ============================= */
bool wifi_and_mqtt_start(void) {
    // Init CYW43 and lwIP/FreeRTOS integration
    LOG_INFO("ATTEMPTING WIFI AND MQTT START\n");
    if (cyw43_arch_init()) {
        LOG_INFO("[NET] cyw43_arch_init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    LOG_INFO("[CFG] BROKER_IP macro: %s\n", BROKER_IP);


    LOG_INFO("[NET] Connecting Wi-Fi SSID=\"%s\"...\n", WIFI_SSID);
    int r = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000000);
    if (r) {
        LOG_INFO("[NET] Wi-Fi connect failed (err=%d)\n", r);
        return false;
    }
    LOG_INFO("[NET] Wi-Fi connected.\n");

    // Resolve broker IP string
    ip_addr_t broker_ip;
    if (!ipaddr_aton(BROKER_IP, &broker_ip)) {
        LOG_INFO("[MQTT] Invalid BROKER_IP string: %s\n", BROKER_IP);
        return false;
    }
    log_ip("[MQTT] Broker:", &broker_ip);

    // Create MQTT client
    cyw43_arch_lwip_begin();
    g_client = mqtt_client_new();
    cyw43_arch_lwip_end();
    if (!g_client) {
        LOG_INFO("[MQTT] mqtt_client_new failed\n");
        return false;
    }

    // Prepare client info
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id  = MQTT_CLIENT_ID;
    ci.keep_alive = MQTT_KEEPALIVE_S;
    // ci.client_user = "user"; ci.client_pass = "pass"; // if your broker needs auth

    LOG_INFO("[MQTT] Connecting to %s:%d as \"%s\"...\n", BROKER_IP, MQTT_PORT, MQTT_CLIENT_ID);

    // Connect (must be under lwIP lock from FreeRTOS side)
    err_t err;
    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(g_client, &broker_ip, MQTT_PORT, mqtt_conn_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        LOG_INFO("[MQTT] mqtt_client_connect err=%d\n", err);
        return false;
    }
    return true;
}

void mqtt_publish_telemetry(float speed, float distance_cm, float yaw_deg,
                            float ultra_cm, const char *state) {
    if (!g_client || !g_mqtt_connected) { 
        LOG_INFO("[MQTT] skip publish (not connected)\n"); 
        return; 
    }

    char topic[128];
    char payload[768];

    // Build JSON first
    snLOG_INFO(payload, sizeof(payload),
             "{\"speed\":%.2f,\"distance\":%.2f,\"imu\":{\"yaw\":%.2f},"
             "\"ultra_cm\":%.2f,\"state\":\"%s\"}",
             speed, distance_cm, yaw_deg, ultra_cm, state ? state : "idle");

    snLOG_INFO(topic, sizeof(topic), "%s/telemetry", BASE_TOPIC);

    // Loud debug
    LOG_INFO("[MQTT] -> topic=\"%s\" bytes=%u\n", topic, (unsigned)strlen(payload));
    LOG_INFO("[MQTT] payload: %s\n", payload);

    // Single publish under lwIP lock
    cyw43_arch_lwip_begin();
    err_t e = mqtt_publish(g_client, topic, payload, (u16_t)strlen(payload),
                           1 /*QoS1*/, 0 /*retain*/, NULL, NULL);
    cyw43_arch_lwip_end();

    if (e != ERR_OK) {
        LOG_INFO("[MQTT] publish err=%d\n", (int)e);
    } else {
        LOG_INFO("[MQTT] publish OK\n");
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
// Add to mqtt_client.c
static void wifi_connect_task(void *pvParameters) {
    LOG_INFO("[NET] WiFi connection task started\n");
    
    if (cyw43_arch_init()) {
        LOG_INFO("[NET] cyw43_arch_init failed\n");
        vTaskDelete(NULL);
        return;
    }
    cyw43_arch_enable_sta_mode();

    LOG_INFO("[NET] Connecting Wi-Fi...\n");
    int r = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000); // 15 second timeout
    
    if (r) {
        LOG_INFO("[NET] Wi-Fi connect failed (err=%d)\n", r);
        cyw43_arch_deinit();
    } else {
        LOG_INFO("[NET] Wi-Fi connected successfully\n");
        // Optionally start MQTT connection here
    }
    
    vTaskDelete(NULL);
}

// Modified wifi_and_mqtt_start that doesn't block
bool wifi_and_mqtt_start_nonblocking(void) {
    // Start WiFi in background task
    if (xTaskCreate(
        wifi_connect_task,
        "WiFi_Conn",
        2048,
        NULL,
        tskIDLE_PRIORITY,  // Low priority - don't block robot
        NULL
    ) != pdPASS) {
        LOG_INFO("[NET] Failed to create WiFi task\n");
        return false;
    }
    
    return true; // Return immediately - WiFi will connect in background
}