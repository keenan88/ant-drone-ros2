#include <Arduino.h>

#include "micro_ros.h"
#include "jack.hpp"
#include <Encoder.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);


  init_jack();

  // InitializeMicroRosTransport();

}

void loop() { 
  // go_to_height_ticks(100);
  // go_to_height_mm(100);
  go_to_height_mm(10);
  // Serial.println("Stationary");
  // delay(1000);
  // Serial.print("Current reading: ");
  // Serial.println(analogRead(CURRENT_SENSE));



  // Serial.println("MOVING DOWN");
  // digitalWrite(UP_PIN, LOW); 
  // digitalWrite(DOWN_PIN, HIGH); // High impedance state enables pull-up resistor. If mag sensor is engaged, line will be drained and no movement occurs.
  // delay(1000);
  // Serial.print("Current reading: ");
  // Serial.println(analogRead(CURRENT_SENSE));
  // delay(12000);
  // Serial.print("End of down movement: ");
  // Serial.println(encoder__ -> read());



  // Serial.println("MOVING UP");
  // digitalWrite(DOWN_PIN, LOW); // LOW impedance state drains line
  // digitalWrite(UP_PIN, HIGH); 
  // delay(1000);
  // Serial.print("Current reading: ");
  // Serial.println(analogRead(CURRENT_SENSE));
  // delay(10000);
  // Serial.print("End of stationary movement: ");
  // Serial.println(encoder__ -> read());
}