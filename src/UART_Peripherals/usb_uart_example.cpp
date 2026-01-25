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
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  Serial8.begin(115200);
}

void loop() {
  char buf[5];  // 4 chars + null terminator

  // Serial2
  /*if (Serial2.available() >= 4) {
    Serial2.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial2 got: "); Serial.println(buf);
    Serial2.print("Echo: "); Serial2.println(buf);
  }*/

  // Serial3
  if (Serial3.available() >= 4) {
    Serial3.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial3 got: "); Serial.println(buf);
    Serial3.print("Echo: "); Serial3.println(buf);
  }

  // Serial4
  if (Serial4.available() >= 4) {
    Serial4.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial4 got: "); Serial.println(buf);
    Serial4.print("Echo: "); Serial4.println(buf);
  }

  // Serial5
  if (Serial5.available() >= 4) {
    Serial5.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial5 got: "); Serial.println(buf);
    Serial5.print("Echo: "); Serial5.println(buf);
  }

  // Serial6
  if (Serial6.available() >= 4) {
    Serial6.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial6 got: "); Serial.println(buf);
    Serial6.print("Echo: "); Serial6.println(buf);
  }

  // Serial7
  if (Serial7.available() >= 4) {
    Serial7.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial7 got: "); Serial.println(buf);
    Serial7.print("Echo: "); Serial7.println(buf);
  }

  // Serial8
  if (Serial8.available() >= 4) {
    Serial8.readBytes(buf, 4);
    buf[4] = '\0';
    Serial.print("Serial8 got: "); Serial.println(buf);
    Serial8.print("Echo: "); Serial8.println(buf);
  }
}
