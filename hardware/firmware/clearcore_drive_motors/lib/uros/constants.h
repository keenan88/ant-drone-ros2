#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "ClearCore.h"

#define FL_MOTOR ConnectorM0
#define FR_MOTOR ConnectorM1
#define RL_MOTOR ConnectorM2
#define RR_MOTOR ConnectorM3
#define E_STOP_INPUT CLEARCORE_PIN_DI6


constexpr uint16_t k_ms_per_s = 1000;
constexpr uint32_t k_ns_per_ms = 1000 * 1000;
constexpr uint32_t k_ns_per_s = 1000 * 1000 * 1000;



#endif