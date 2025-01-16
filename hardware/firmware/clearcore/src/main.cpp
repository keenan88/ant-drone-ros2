#include <Arduino.h>

#include "pins.h"
#include "uros.h"
#include "motor_interface.h"

void setup() {

  initialize_motors();

  InitializeMicroRosTransport();

  // pinMode(kHeartBeatLedPin, OUTPUT);
}

void loop() { ManageAgentLifecycle(); }