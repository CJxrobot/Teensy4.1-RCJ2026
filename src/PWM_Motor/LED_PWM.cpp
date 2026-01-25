#include <Arduino.h>
#define PWM_Pin1 2

void setup() {
  pinMode(PWM_Pin1, OUTPUT);  // sets the pin as output
}

void loop() {
  analogWrite(PWM_Pin1, 255); // PWM values from 0 to 255
}