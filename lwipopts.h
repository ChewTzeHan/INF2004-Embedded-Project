// #ifndef _LWIPOPTS_H
// #define _LWIPOPTS_H

// // Generally you would define your own explicit list of lwIP options
// // (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
// //
// // This example uses a common include to avoid repetition
// #include "lwipopts_examples_common.h"

// #if !NO_SYS
// #define TCPIP_THREAD_STACKSIZE 1024
// #define DEFAULT_THREAD_STACKSIZE 1024
// #define DEFAULT_RAW_RECVMBOX_SIZE 8
// #define TCPIP_MBOX_SIZE 8
// #define LWIP_TIMEVAL_PRIVATE 0

// // not necessary, can be done either way
// #define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// // ping_thread sets socket receive timeout, so enable this feature
// #define LWIP_SO_RCVTIMEO 1
// #endif

// #endif /* _LWIPOPTS_H */

#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// We are building the FreeRTOS/sys variant
#define NO_SYS 0
#define LWIP_SOCKET  0   // raw API
#define LWIP_NETCONN 0

// Pull common defaults AFTER forcing NO_SYS
#include "lwipopts_examples_common.h"

// DO NOT define LWIP_SOCKET here (CMake provides it as 1 for ping).
// Leave NETCONN alone too unless you really need it.

// Thread/stack/mailboxes
#define TCPIP_THREAD_STACKSIZE   4096
#define DEFAULT_THREAD_STACKSIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE          8
#define LWIP_TIMEVAL_PRIVATE     0
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1
#define LWIP_SO_RCVTIMEO         1

// Enable lwIP MQTT app
#define LWIP_MQTT                1
#define MQTT_OUTPUT_RINGBUF_SIZE 2048
#define MQTT_REQ_MAX_IN_FLIGHT   4

#endif /* _LWIPOPTS_H */
