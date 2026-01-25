#include <Arduino.h>

// Pin definitions
const int buttonPin = 27;
const int ledPin = 13;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Polling test: press button on pin 27 to turn LED on");

  pinMode(buttonPin, INPUT_PULLUP); // Button with pull-up
  pinMode(ledPin, OUTPUT);          // LED pin
}

void loop() {
  // Polling: check button state continuously
  if (digitalRead(buttonPin) == LOW) {
    digitalWrite(ledPin, HIGH);   // LED on
    Serial.println("Button pressed! LED ON");
    delay(500);                   // keep LED on for 0.5s
    digitalWrite(ledPin, LOW);    // LED off
  }
}
