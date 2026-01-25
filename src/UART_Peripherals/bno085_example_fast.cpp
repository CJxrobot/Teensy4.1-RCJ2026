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

    static enum { WAIT_HEADER1, WAIT_HEADER2, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_HEADER1;
    static uint8_t buffer[19];
    static uint8_t pos = 0;
    static uint8_t checksum_sum = 0;

    while(serial.available()) {
        uint8_t b = serial.read();

        switch(state) {
            case WAIT_HEADER1:
                if(b == 0xAA) state = WAIT_HEADER2;
                break;

            case WAIT_HEADER2:
                if(b == 0xAA) {
                    state = WAIT_PAYLOAD;
                    pos = 2;
                    buffer[0] = 0xAA;
                    buffer[1] = 0xAA;
                    checksum_sum = 0;
                } else state = WAIT_HEADER1;
                break;

            case WAIT_PAYLOAD:
                buffer[pos++] = b;
                if(pos < 18) checksum_sum += b; // accumulate checksum for bytes [2..17]
                if(pos == 18) state = WAIT_CHECKSUM;
                break;

            case WAIT_CHECKSUM:
                buffer[18] = b;
                if(checksum_sum == buffer[18]) {
                    uint8_t index = buffer[2];
                    // Little-endian decode
                    int16_t yaw_raw   = *(int16_t*)&buffer[3];
                    //int16_t pitch_raw = *(int16_t*)&buffer[5];
                    //int16_t roll_raw  = *(int16_t*)&buffer[7];

                    int16_t ax_raw = *(int16_t*)&buffer[9];
                    int16_t ay_raw = *(int16_t*)&buffer[11];
                    //int16_t az_raw = *(int16_t*)&buffer[13];

                    data.index = index;
                    data.yaw   = yaw_raw * 0.01f;
                    //data.pitch = pitch_raw * 0.01f;
                    //data.roll  = roll_raw * 0.01f;
                    data.ax    = ax_raw * 0.00980665f;
                    data.ay    = ay_raw * 0.00980665f;
                    //data.az  = az_raw * 0.00980665f;
                    data.valid = true;
                }
                state = WAIT_HEADER1; // reset for next packet
                return data.valid ? data : data; // return packet once
        }
    }

    return data; // no packet available
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
