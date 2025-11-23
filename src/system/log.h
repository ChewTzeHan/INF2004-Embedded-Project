#ifndef LOG_H
#define LOG_H

#include <stdio.h>

// 0 = no debug output (submission), 1 = show debug prints
#define ENABLE_DEBUG_LOG 0

#if ENABLE_DEBUG_LOG
    #define LOG_INFO(...)  printf(__VA_ARGS__)
#else
    #define LOG_INFO(...)
#endif

#define snLOG_INFO(...)   snprintf(__VA_ARGS__)

#endif // LOG_H