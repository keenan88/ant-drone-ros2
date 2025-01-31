#include <Arduino.h>

#include "micro_ros.h"
#include "jack.hpp"
#include <Encoder.h>



#define A GPIO_NUM_39
#define B GPIO_NUM_35

#define L_EN_PIN GPIO_NUM_18
#define L_PWM_PIN GPIO_NUM_12
#define R_EN_PIN GPIO_NUM_14
#define R_PWM_PIN GPIO_NUM_16

Encoder* encoder__ = nullptr;

void setup()
{
  // init_jack();

  encoder__ = new Encoder(A, B);

  pinMode(L_EN_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  Serial.begin(115200);
  // InitializeMicroRosTransport();

}

void loop() { 

  // ManageAgentLifecycle();

  Serial.println(encoder__ -> read());

  digitalWrite(L_EN_PIN, HIGH);
  digitalWrite(R_EN_PIN, HIGH);

  analogWrite(L_PWM_PIN, 127);
  analogWrite(R_PWM_PIN, 0);

  delay(1000);

  digitalWrite(L_EN_PIN, HIGH);
  digitalWrite(R_EN_PIN, HIGH);

  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 127);

  delay(1000);
}