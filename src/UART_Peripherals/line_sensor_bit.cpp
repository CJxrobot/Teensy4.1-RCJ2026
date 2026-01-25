#include <Robot.h>
uint8_t buffer[7];
//AA AA data1 data2 data3 chs end
int buffer_index = 0;

void setup() {
  Serial.begin(9600);   // Debug
  Serial4.begin(115200);  // Connect to ESP32 Serial0 TX
  Serial.println("start");   // Debug
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
}

void loop() {
  if (Serial4.available()) {
    uint8_t b = Serial4.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;

    if (buffer_index == 7) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[0] == 0xAA && buffer[6] == 0xEE) {
        uint32_t state = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16);
        uint8_t checksum = (buffer[2] + buffer[3] + buffer[4]) & 0xFF;
        if (checksum == buffer[5]) {
            Serial.print("sensors: ");
            Serial.println(state, BIN);
        }
      }
    }
  }
}
