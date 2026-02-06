#include <Arduino.h>

/*
   Teensy 4.1 Multi-UART Demo
   - Uses Serial2..Serial8
   - Reads exactly 4 bytes at a time
   - Echoes back the same 4 bytes
   - USB Serial Monitor shows what each port receives
*/

void setup() {
  Serial.begin(9600);      // USB serial for debug
  pinMode(13, OUTPUT);       
  digitalWrite(13, HIGH);    // LED ON
  delay(2000);

  Serial.println("=== Teensy 4.1 Multi-UART Test (read 4 bytes) ===");

  // Start all UARTs at 9600 baud
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
}

void loop() {
  char buf[5];  // 4 chars + null terminator
  // Serial4
  if (Serial4.available() >= 4) {
    Serial.print("Serial4 got: "); Serial.println(Serial4.read());
    Serial4.print("Echo: "); Serial4.println(buf);
  }

}
