#include <Arduino.h>

#include "pins.h"
#include "uros.h"

void setup() {
  InitializeMicroRosTransport();

  // pinMode(kHeartBeatLedPin, OUTPUT);
}

void loop() { ManageAgentLifecycle(); }