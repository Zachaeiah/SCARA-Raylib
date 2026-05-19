#ifndef PROFILER_H
#define PROFILER_H

#include "raylib.h"

/**
 * @brief 
 * 
 */
typedef struct ProfilerTimer
{
    const char* name;
    double start;
    double elapsed_ms;
} ProfilerTimer;

/**
 * @brief 
 * 
 * @param timer 
 */
static inline void PROFILER_Begin(ProfilerTimer* timer)
{
    timer->start = GetTime();
    timer->elapsed_ms = 0;
}

/**
 * @brief 
 * 
 * @param timer 
 */
static inline void PROFILER_End(ProfilerTimer* timer)
{
    double end = GetTime();
    timer->elapsed_ms = (end - timer->start) * 1000.0;
}

#endif