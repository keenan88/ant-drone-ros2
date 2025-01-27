#include <Arduino.h>

#include "uros.h"
#include "motor_interface.h"
#include "constants.h"

void setup() {
  initialize_motors();

  InitializeMicroRosTransport();
}

void loop() { 
  if(digitalReadClearCore(E_STOP_INPUT))
  {
    stop_motors();
  }
  ManageAgentLifecycle(); 
}