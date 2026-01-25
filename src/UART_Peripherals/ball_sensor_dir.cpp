#include <Arduino.h>
uint8_t buffer[3];
int buffer_index = 0;

void setup() {
  Serial.begin(9600);   // Debug
  Serial3.begin(115200);  // Connect to ESP32 Serial0 TX
  Serial.println("start");   // Debug
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
}

void loop() {
  if (Serial3.available()) {
    uint8_t b = Serial3.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;

    if (buffer_index == 3) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[2] == 0xEE) {
        uint8_t temp = buffer[1];
        uint8_t dir = (temp & 0x0F);
        uint8_t dist = (temp & 0xF0) >> 4;
        Serial.print("Ball Direction: ");
        Serial.print(dir);
        Serial.print("°, Distance: ");
        Serial.println(dist);
      }
    }
  }
}
