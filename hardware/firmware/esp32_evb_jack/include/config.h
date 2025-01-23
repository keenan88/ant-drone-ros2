#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <inttypes.h>

#define SERIAL_DEBUG
#define FIRMWARE_VERSION "0.1.0"
#define HARDWARE_ID "ce4da0b1-30c2-45d6-9597-1d5ec8a065ad"
constexpr uint16_t kMillisecondsInASecond = 1000;
constexpr uint32_t kNanosecondsInAMillisecond = 1000000;
constexpr uint32_t kNanosecondsInASecond = 1000000000;
constexpr int8_t kDomainId = 10;
constexpr char kNodeName[] = "uros_client";
constexpr char kNamespace[] = "";

constexpr uint16_t kStateUpdateMs = 100;   
constexpr uint8_t kStringLength = 140;
constexpr uint8_t kNumberOfHandles = 2;  // # subscriptions + # timers

#endif