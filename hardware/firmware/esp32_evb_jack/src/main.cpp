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

  // Up
  digitalWrite(L_EN_PIN, HIGH);
  digitalWrite(R_EN_PIN, HIGH);  
  analogWrite(L_PWM_PIN, 100);
  analogWrite(R_PWM_PIN, 0);

  delay(5000);

  // Stationary
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW);

  delay(5000);

  // Down
  digitalWrite(L_EN_PIN, HIGH);
  digitalWrite(R_EN_PIN, HIGH);
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 100);

  delay(5000);


  // Stop movement
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(R_EN_PIN, LOW);

  delay(5000);
}