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

// #ifndef _LWIPOPTS_H
// #define _LWIPOPTS_H

// // We are building the FreeRTOS/sys variant
// #define NO_SYS 0
// #define LWIP_SOCKET  0   // raw API
// #define LWIP_NETCONN 0

// // Pull common defaults AFTER forcing NO_SYS
// #include "lwipopts_examples_common.h"

// // DO NOT define LWIP_SOCKET here (CMake provides it as 1 for ping).
// // Leave NETCONN alone too unless you really need it.

// // Thread/stack/mailboxes
// #define TCPIP_THREAD_STACKSIZE   4096
// #define DEFAULT_THREAD_STACKSIZE 4096
// #define DEFAULT_RAW_RECVMBOX_SIZE 8
// #define TCPIP_MBOX_SIZE          8
// #define LWIP_TIMEVAL_PRIVATE     0
// #define LWIP_TCPIP_CORE_LOCKING_INPUT 1
// #define LWIP_SO_RCVTIMEO         1

// // Enable lwIP MQTT app
// #define LWIP_MQTT                1
// #define MQTT_OUTPUT_RINGBUF_SIZE 2048
// #define MQTT_REQ_MAX_IN_FLIGHT   4

// #endif /* _LWIPOPTS_H */


#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* ================================
 * Build: FreeRTOS/sys variant
 * ================================ */
#define NO_SYS            0          /* we use the sys (FreeRTOS) port */
#define LWIP_SOCKET       0          /* raw API only */
#define LWIP_NETCONN      0

/* Pull common defaults AFTER forcing NO_SYS.
 * (This file from pico-examples sets a bunch of sane defaults.)
 */
#include "lwipopts_examples_common.h"

/* ================================
 * Threads / mailboxes / timing
 * ================================ */
#undef  TCPIP_THREAD_STACKSIZE
#define TCPIP_THREAD_STACKSIZE      4096

#undef  DEFAULT_THREAD_STACKSIZE
#define DEFAULT_THREAD_STACKSIZE    4096

#undef  DEFAULT_RAW_RECVMBOX_SIZE
#define DEFAULT_RAW_RECVMBOX_SIZE   16

#undef  TCPIP_MBOX_SIZE
#define TCPIP_MBOX_SIZE             16

#define LWIP_TIMEVAL_PRIVATE        0
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1
#define LWIP_SO_RCVTIMEO            1

/* More timer nodes for DHCP, DNS, TCP, MQTT, plus your extra TCP server */
#ifndef MEMP_NUM_SYS_TIMEOUT
#define MEMP_NUM_SYS_TIMEOUT        32     /* <- key fix; was ~5 */
#endif

/* ================================
 * Memory / pools / pbufs
 * ================================ */
/* Grow the lwIP heap to avoid "pool empty" panics under load */
#undef  MEM_SIZE
#define MEM_SIZE                    24000  /* 24 KB for lwIP heap */

#undef  PBUF_POOL_SIZE
#define PBUF_POOL_SIZE              32

/* (Keep alignment from common header) */
#ifndef MEM_ALIGNMENT
#define MEM_ALIGNMENT               4
#endif

/* ================================
 * Core Protocols
 * ================================ */
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1

/* ================================
 * MQTT app (raw API)
 * ================================ */
#define LWIP_MQTT                   1

/* Limit concurrent in-flight requests; plenty for telemetry */
#undef  MQTT_REQ_MAX_IN_FLIGHT
#define MQTT_REQ_MAX_IN_FLIGHT      2

/* Outgoing ringbuffer (bytes) */
#undef  MQTT_OUTPUT_RINGBUF_SIZE
#define MQTT_OUTPUT_RINGBUF_SIZE    2048

/* ================================
 * Optional debugging
 * ================================ */
#ifndef NDEBUG
#define LWIP_DEBUG                  1
/* Turn on just MQTT debug to start (others OFF to keep noise low) */
#define MQTT_DEBUG                  LWIP_DBG_ON
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#endif

#endif /* _LWIPOPTS_H */
