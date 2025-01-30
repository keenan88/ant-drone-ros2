#include <Arduino.h>

#include "uros.h"
#include "motor_interface.h"
#include "constants.h"

#include "ClearCore.h"

void setup() {
  initialize_motors();

  pinModeClearCore(CLEARCORE_PIN_LED, OUTPUT);

  InitializeMicroRosTransport();

  
}

void loop() { 
  if(digitalReadClearCore(E_STOP_INPUT))
  {
    disable_motors();
  }
  ManageAgentLifecycle(); 
}