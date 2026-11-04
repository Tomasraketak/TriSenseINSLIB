// Minimal Arduino stand-in for the host test. Not part of the library.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
extern uint64_t g_sim_us;
inline uint32_t micros() { return (uint32_t)g_sim_us; }
inline uint32_t millis() { return (uint32_t)(g_sim_us / 1000); }
inline void delay(uint32_t ms) { g_sim_us += (uint64_t)ms * 1000; }
