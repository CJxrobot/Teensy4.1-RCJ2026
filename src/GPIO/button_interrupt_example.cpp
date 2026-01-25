#include <Arduino.h>

const int buttonPin = 27;
const int ledPin = 13;

// Interrupt flag
volatile bool buttonPressed = false;

void ISR_button() {
  buttonPressed = true;
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Button with pull-up resistor
  pinMode(ledPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin), ISR_button, FALLING);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Press button on pin 27 to turn LED on");
}

void loop() {
  if (buttonPressed) {
    buttonPressed = false;  // Clear flag
    digitalWrite(ledPin, HIGH);
    Serial.println("Button pressed! LED ON");
    delay(500);             // Keep LED on for 500ms
    digitalWrite(ledPin, LOW);
  }
}
