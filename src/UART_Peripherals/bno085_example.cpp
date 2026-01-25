#include <Arduino.h>

// Data structure for decoded BNO085 packet
struct BNO085Data {
  uint8_t index;
  float yaw;   // degrees
  float pitch; // degrees
  float roll;  // degrees
  float ax;    // m/s²
  float ay;    // m/s²
  float az;    // m/s²
  bool valid;  // checksum/header status
};

BNO085Data decodeBNO085(Stream &serial) {
  BNO085Data data = {0,0,0,0,0,0,0,false};
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];

  while (serial.available() >= PACKET_SIZE) {
    buffer[0] = serial.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = serial.read();
    if (buffer[1] != 0xAA) continue;

    for (int i = 2; i < PACKET_SIZE; i++) {
      buffer[i] = serial.read();
    }

    uint8_t index = buffer[2];

    // Little-endian decode
    int16_t yaw_raw   = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);
    int16_t roll_raw  = (int16_t)((buffer[8] << 8) | buffer[7]);

    int16_t ax_raw = (int16_t)((buffer[10] << 8) | buffer[9]);
    int16_t ay_raw = (int16_t)((buffer[12] << 8) | buffer[11]);
    int16_t az_raw = (int16_t)((buffer[14] << 8) | buffer[13]);

    uint8_t checksum = buffer[18];

    // Checksum (sum of bytes [2..17])
    uint16_t sum = 0;
    for (int i = 2; i <= 17; i++) {
      sum += buffer[i];
    }
    sum &= 0xFF;

    if (sum == checksum) {
      data.index = index;
      data.yaw   = yaw_raw   * 0.01f;
      data.pitch = pitch_raw * 0.01f;
      data.roll  = roll_raw  * 0.01f;
      data.ax = ax_raw * 0.00980665f;
      data.ay = ay_raw * 0.00980665f;
      data.az = az_raw * 0.00980665f;
      data.valid = true;
    }
    break;
  }
  return data;
}


// ------------------ Arduino setup & loop ------------------

void setup() {
  Serial.begin(9600);   // USB serial for debug
  Serial2.begin(115200);  // BNO085 connected here (change if needed)
  delay(1000);
  Serial.println("BNO085 IMU Decoder Ready");
}
unsigned long lastRatePrint = 0;
uint32_t packetCount = 0;

void loop() {
    if (Serial2.available() >= 19) {
        BNO085Data imu = decodeBNO085(Serial2);
        if (imu.valid) {
            packetCount++;  // count valid packets
/*
            Serial.print("Index: "); Serial.println(imu.index);
            Serial.print("Yaw: ");   Serial.print(imu.yaw);   Serial.println(" deg");
            Serial.print("Pitch: "); Serial.print(imu.pitch); Serial.println(" deg");
            Serial.print("Roll: ");  Serial.print(imu.roll);  Serial.println(" deg");
            Serial.print("AX: ");    Serial.print(imu.ax);    Serial.println(" m/s²");
            Serial.print("AY: ");    Serial.print(imu.ay);    Serial.println(" m/s²");
            Serial.print("AZ: ");    Serial.print(imu.az);    Serial.println(" m/s²");
            Serial.println("---------------------------");
*/
        }
    }

    // Every 1 second, print packets per second (sampling rate)
    if (millis() - lastRatePrint >= 1000) {
        Serial.print("Sampling rate: ");
        Serial.print(packetCount);
        Serial.println(" Hz");

        packetCount = 0;
        lastRatePrint = millis();
    }
}
