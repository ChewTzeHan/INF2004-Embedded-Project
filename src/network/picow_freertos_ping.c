/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Modified for WiFi Data Exchange Demo
 * 
 * Description: This program creates a WiFi TCP server on Raspberry Pi Pico W
 * that listens for incoming connections and exchanges data with clients.
 * It demonstrates bidirectional data exchange over WiFi.
 */

#include "picow_freertos_ping.h"
#include "log.h"

// ==============================
// GLOBAL VARIABLES
// ==============================

MessageBufferHandle_t xControlMessageBuffer = NULL;
struct tcp_pcb *tcp_server_pcb = NULL;
uint8_t receive_buffer[BUFFER_SIZE];

// ==============================
// FUNCTION DEFINITIONS
// ==============================

/**
 * @brief Reads the onboard temperature sensor
 * 
 * The RP2040 has an internal temperature sensor connected to ADC channel 4.
 * This function converts the ADC reading to temperature in Celsius.
 * 
 * @return float Temperature in degrees Celsius
 */
float read_onboard_temperature() {
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    return tempC;
}

/**
 * @brief TCP Error callback function
 * 
 * Called when a TCP error occurs on a connection.
 * 
 * @param arg User-defined argument (not used)
 * @param err Error code indicating what went wrong
 */
static void tcp_server_error(void *arg, err_t err) {
    LOG_INFO("TCP client error: %d\n", err);
}

/**
 * @brief TCP Receive callback function
 * 
 * Called when data is received from a TCP client.
 * Processes the received data and sends back a response.
 * 
 * @param arg User-defined argument (not used)
 * @param tpcb TCP control block for the connection
 * @param p Packet buffer containing received data
 * @param err Error status
 * @return err_t Error code (ERR_OK on success)
 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    // Check if connection was closed (p is NULL when client disconnects)
    if (p == NULL) {
        LOG_INFO("TCP connection closed\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Check for receive errors
    if (err != ERR_OK) {
        LOG_INFO("TCP receive error: %d\n", err);
        pbuf_free(p);
        return err;
    }

    // Acknowledge received data to the TCP stack (flow control)
    tcp_recved(tpcb, p->tot_len);

    // Process received data if we have any
    if (p->tot_len > 0) {
        LOG_INFO("Received %d bytes from client\n", p->tot_len);
        
        // Copy data from packet buffer to our receive buffer
        uint16_t copy_len = pbuf_copy_partial(p, receive_buffer, p->tot_len, 0);
        receive_buffer[copy_len] = '\0';
        
        LOG_INFO("Received data: %s\n", receive_buffer);
        
        // Send acknowledgment back to client
        char ack_msg[64];
        snLOG_INFO(ack_msg, sizeof(ack_msg), "ACK: Received '%s'", receive_buffer);
        err_t write_err = tcp_write(tpcb, ack_msg, strlen(ack_msg), TCP_WRITE_FLAG_COPY);
        if (write_err == ERR_OK) {
            tcp_output(tpcb);
        }
        
        // Send temperature data back to client
        float temperature = read_onboard_temperature();
        char temp_msg[64];
        snLOG_INFO(temp_msg, sizeof(temp_msg), "Pico Temp: %.2f degC", temperature);
        write_err = tcp_write(tpcb, temp_msg, strlen(temp_msg), TCP_WRITE_FLAG_COPY);
        if (write_err == ERR_OK) {
            tcp_output(tpcb);
        }
        
        LOG_INFO("Sent acknowledgment and temperature to client\n");
    }

    pbuf_free(p);
    return ERR_OK;
}

/**
 * @brief TCP Accept callback function
 * 
 * Called when a new client connects to the server.
 * Sets up the callbacks for the new connection.
 * 
 * @param arg User-defined argument (not used)
 * @param newpcb TCP control block for the new connection
 * @param err Error status
 * @return err_t Error code (ERR_OK on success)
 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    // Check for errors or invalid PCB
    if (err != ERR_OK || newpcb == NULL) {
        LOG_INFO("TCP accept error: %d\n", err);
        return ERR_VAL;
    }
    
    // Print connection information
    LOG_INFO("New TCP connection accepted from %s:%d\n", 
           ip4addr_ntoa(&newpcb->remote_ip), newpcb->remote_port);
    
    // Set up callbacks for the new connection
    tcp_arg(newpcb, NULL);
    tcp_err(newpcb, tcp_server_error);
    tcp_recv(newpcb, tcp_server_recv);
    
    return ERR_OK;
}

/**
 * @brief Initialize the TCP server
 * 
 * Creates and configures a TCP server that listens for incoming connections.
 * 
 * @return bool true if initialization successful, false otherwise
 */
static bool tcp_server_init(void) {
    err_t err;
    
    // Create new TCP Protocol Control Block (PCB)
    tcp_server_pcb = tcp_new();
    if (!tcp_server_pcb) {
        LOG_INFO("Failed to create TCP PCB\n");
        return false;
    }
    
    // Bind the PCB to any local IP address and the specified port
    err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, TCP_PORT);
    if (err != ERR_OK) {
        LOG_INFO("Failed to bind TCP PCB: %d\n", err);
        tcp_close(tcp_server_pcb);
        return false;
    }
    
    // Start listening for incoming connections
    tcp_server_pcb = tcp_listen(tcp_server_pcb);
    if (!tcp_server_pcb) {
        LOG_INFO("Failed to listen on TCP PCB\n");
        return false;
    }
    
    // Set the accept callback function for new connections
    tcp_accept(tcp_server_pcb, tcp_server_accept);
    
    // Server is now ready
    LOG_INFO("TCP server started on port %d\n", TCP_PORT);
    LOG_INFO("Server IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    
    return true;
}