#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <inttypes.h>

#define FIRMWARE_VERSION "0.1.0"
#define HARDWARE_ID "1ceec360-32b9-431c-9166-1252809e85b2"

constexpr int8_t kDomainId = 8;
constexpr char kNodeName[] = "uros_clearcore_client";
constexpr char kNamespace[] = "drive_motor";

constexpr uint8_t kStringLength = 140;

#endif