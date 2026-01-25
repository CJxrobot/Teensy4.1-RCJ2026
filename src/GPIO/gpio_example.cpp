#include <Arduino.h>

// Pins
const int pinA = 26;
const int pinB = 27;
const int pinC = 33;

// Variables to count interrupts
volatile int countA = 0;
volatile int countB = 0;
volatile int countC = 0;

// Interrupt service routines
void ISR_pinA() {
  countA++;
}

void ISR_pinB() {
  countB++;
}

void ISR_pinC() {
  countC++;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Teensy 4.1 GPIO Interrupt Test ===");

  // Configure pins as input with pullup (common for buttons)
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinC, INPUT_PULLUP);

  // Attach interrupts on FALLING edge (button press)
  attachInterrupt(digitalPinToInterrupt(pinA), ISR_pinA, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinB), ISR_pinB, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinC), ISR_pinC, FALLING);
}

void loop() {
  // Print counts every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Pin26 count: "); Serial.print(countA);
    Serial.print("  | Pin27 count: "); Serial.print(countB);
    Serial.print("  | Pin33 count: "); Serial.println(countC);
  }
}
