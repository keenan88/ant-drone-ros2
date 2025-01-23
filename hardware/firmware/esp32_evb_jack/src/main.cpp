#include <Arduino.h>

#include "micro_ros.h"
#include "jack.hpp"



void setup()
{
  init_jack();
  InitializeMicroRosTransport();

}

void loop() { 

  ManageAgentLifecycle();
}