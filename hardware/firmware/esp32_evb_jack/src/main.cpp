#include <Arduino.h>

#include "micro_ros.h"


void setup()
{
  InitializeMicroRosTransport();

}

void loop() { 

  ManageAgentLifecycle();
}
