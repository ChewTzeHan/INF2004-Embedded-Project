/**
 * @file picow_freertos_ping.h
 * @brief WiFi TCP Server functions for Raspberry Pi Pico W
 *
 * Header file containing declarations for TCP server functionality
 * and temperature reading for the Pico W WiFi data exchange demo.
 */

#ifndef PICOW_FREERTOS_PING_H
#define PICOW_FREERTOS_PING_H

#include "FreeRTOS.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "message_buffer.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

// ==============================
// CONSTANT DEFINITIONS
// ==============================

#define mbaTASK_MESSAGE_BUFFER_SIZE (60)
#define TCP_PORT                    4242
#define BUFFER_SIZE                 256

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// ==============================
// EXTERNAL VARIABLES
// ==============================

extern MessageBufferHandle_t xControlMessageBuffer;
extern struct tcp_pcb*       tcp_server_pcb;
extern uint8_t               receive_buffer[BUFFER_SIZE];

// ==============================
// FUNCTION DECLARATIONS
// ==============================

/**
 * @brief Reads the onboard temperature sensor
 * @return float Temperature in degrees Celsius
 */
float read_onboard_temperature(void);

/**
 * @brief TCP Error callback function
 * @param arg User-defined argument
 * @param err Error code
 */
static void tcp_server_error(void* arg, err_t err);

/**
 * @brief TCP Receive callback function
 * @param arg User-defined argument
 * @param tpcb TCP control block
 * @param p Packet buffer
 * @param err Error status
 * @return err_t Error code
 */
static err_t tcp_server_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);

/**
 * @brief TCP Accept callback function
 * @param arg User-defined argument
 * @param newpcb TCP control block for new connection
 * @param err Error status
 * @return err_t Error code
 */
static err_t tcp_server_accept(void* arg, struct tcp_pcb* newpcb, err_t err);

/**
 * @brief Initialize the TCP server
 * @return bool true if successful, false otherwise
 */
static bool tcp_server_init(void);

/**
 * @brief Main task function
 * @param params Task parameters
 */
void main_task(__unused void* params);

#endif // PICOW_FREERTOS_PING_H
