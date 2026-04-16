#include "sub_core.h"

// 1. Define the actual memory for these variables here
LineData line; 
GyroData gyroData;       
RobotStatus robot;
MainCoreCommand mainCommand;
uint16_t avg_ls[32];

void sub_core_init() {
    Serial8.begin(115200); // For communication with MainCore
    Serial2.begin(115200); // For gyro sensor
    Serial.begin(115200);  // For debugging

    // Multiplexer Control
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(M1, INPUT_PULLDOWN);
    pinMode(M2, INPUT_PULLDOWN);

    // Motor Initialization
    // Motor 1
    pinMode(pwmPin1, OUTPUT);
    pinMode(DIRA_1, OUTPUT);
    pinMode(DIRB_1, OUTPUT);
    // Motor 2
    pinMode(pwmPin2, OUTPUT);
    pinMode(DIRA_2, OUTPUT);
    pinMode(DIRB_2, OUTPUT);
    // Motor 3
    pinMode(pwmPin3, OUTPUT);
    pinMode(DIRA_3, OUTPUT);
    pinMode(DIRB_3, OUTPUT);
    // Motor 4
    pinMode(pwmPin4, OUTPUT);
    pinMode(DIRA_4, OUTPUT);
    pinMode(DIRB_4, OUTPUT);

    EEPROM.begin();
    EEPROM.get(0, avg_ls);
}

int readMux(int ch, int sig) {
    digitalWrite(s0, (ch >> 0) & 1);
    digitalWrite(s1, (ch >> 1) & 1);
    digitalWrite(s2, (ch >> 2) & 1);
    digitalWrite(s3, (ch >> 3) & 1);
    delayMicroseconds(10);
    if(sig == 1){
        return analogRead(M1);
    }
    if(sig == 2){
        return analogRead(M2);
    }
}

void update_line_sensor(){
    for (int i = 0; i < 32; i++) {
        int val = readMux(i % 16, (i < 16) ? M1 : M2);
        if (val > avg_ls[i]) {
            line.state |= (1UL << i);
        }
    }
}

void update_gyro_sensor(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if(esti_checksum != buffer[18]){
      //Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);

    //Serial.print("yaw_raw: ");
    //Serial.println(yaw_raw);

    // Convert to degrees if within range
    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.exist = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

void line_calibrate() {
    uint16_t max_ls[32], min_ls[32];
    for (int i = 0; i < 32; i++) { 
        max_ls[i] = 0; 
        min_ls[i] = 4095; 
    }

    while (1) {
        if(Serial8.available()){
            uint8_t cmd = Serial8.read();
            if(cmd == LS_CAL_END){ // End calibration command
                break;
            }
        }
        for (int i = 0; i < 32; i++) {
            int r = readMux(i % 16, (i < 16) ? M1 : M2);
            if (r > max_ls[i]) max_ls[i] = r; 
            if (r < min_ls[i]) min_ls[i] = r;
        }
    }

    for (int i = 0; i < 32; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
    EEPROM.put(0, avg_ls);
    Serial8.write(LS_CAL_ACK); // Send end calibration acknowledgment
}

/* --- Actuators Part --- */

void SetMotorSpeed(uint8_t port, float speed) {
    // Constrain speed to prevent PWM overflow
    speed = constrain(speed, -255, 255); 
    //int pwmVal = abs((int)speed);
    int pwmVal = abs(speed) * 255 / 100;

    uint8_t p_pwm, p_a, p_b;
    switch(port) {
        case 4: p_pwm = pwmPin1; p_a = DIRA_1; p_b = DIRB_1; break;
        case 3: p_pwm = pwmPin2; p_a = DIRA_2; p_b = DIRB_2; break;
        case 2: p_pwm = pwmPin3; p_a = DIRA_3; p_b = DIRB_3; break;
        case 1: p_pwm = pwmPin4; p_a = DIRA_4; p_b = DIRB_4; break;
        default: return;
    }

    analogWrite(p_pwm, pwmVal);
    digitalWrite(p_a, (speed > 0) ? HIGH : LOW);
    digitalWrite(p_b, (speed < 0) ? HIGH : LOW);
}

void RobotIKControl(float vx, float vy, float omega) {
    // Applying the Inverse Kinematics Matrix
    float p1 = -0.643f * vx + 0.766f * vy + omega;
    float p2 = -0.643f * vx - 0.766f * vy + omega;
    float p3 =  0.707f * vx - 0.707f * vy + omega;
    float p4 =  0.707f * vx + 0.707f * vy + omega;

    SetMotorSpeed(1, p1);
    SetMotorSpeed(2, p2);
    SetMotorSpeed(3, p3);
    SetMotorSpeed(4, p4);
}

void Vector_Motion(float Vx, float Vy, float rot_V) {  
    robot.robot_heading += rot_V; // Update target heading based on input
    if(robot.robot_heading > 135){
      robot.robot_heading =  135;  
    }
    else if(robot.robot_heading < 45){
      robot.robot_heading = 45;  
    }
    Serial.printf("robot.robot_heading%f\n",robot.robot_heading);
    
    float e = robot.robot_heading - (90.0f - gyroData.heading);

    // Normalize error (-180 to 180)
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;
    RobotIKControl(Vx, Vy, omega);
}

void FC_Vector_Motion(float WVx, float WVy, float target_heading) {
    // 1. Convert to Robot Frame
    float rad = (target_heading - 90.0f) * (M_PI / 180.0f);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    float robot_vx = WVx * cos_h + WVy * sin_h;
    float robot_vy = -WVx * sin_h + WVy * cos_h;

    // 2. Heading Correction
    float current_gyro_heading = 90.0f - gyroData.heading;
    float e = target_heading - current_gyro_heading;
    
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    float omega = (fabs(e) > robot.heading_threshold) ? (e * robot.P_factor) : 0;

    RobotIKControl(robot_vx, robot_vy, omega);
}

void sendGyroAndLineToMainCore() {
    uint8_t data[5];
    data[0] = 0xBB; // Header
    data[1] = (uint8_t)(gyroData.heading); // Gyro Heading (0-255)
    data[2] = (uint8_t)(line.state & 0xFF); // Line Sensor State (lower 8 bits)
    data[3] = (uint8_t)((line.state >> 8) & 0xFF); // Line Sensor State (upper 8 bits)
    data[4] = 0xEE; // Footer
    Serial8.write(data, sizeof(data));
}

void readMotorCommand() {
    if(Serial8.available()) {
        // 1. Peek: "Is the first byte the header?"
        if(Serial8.peek() != PROTOCOL_HEADER) {
            
            // 2. If NO: "This byte is trash. Throw it away."
            Serial8.read(); 
            
            // 3. Continue: "Check the next byte immediately."
            continue; 
        }
        // 4. If YES: "The header is at the front! Safe to read all 6 bytes now."
        uint8_t buf[6];
        for(int i = 0; i < 6; i++) {
            buf[i] = Serial8.read();
        }
        if(buf[5] != PROTOCOL_END) {
            // Footer check failed, discard and wait for next command
            return;
        }
        int8_t vx = (int8_t)buf[1];
        int8_t vy = (int8_t)buf[2];
        int8_t rot_v = (int8_t)buf[3] * 0.01;
        int8_t target_heading = (int8_t)buf[4] * 10;

        // Convert back to float and degrees
        mainCommand.vx = (float)vx;
        mainCommand.vy = (float)vy;
        mainCommand.rot_v = (float)rot_v;
        mainCommand.heading = (uint16_t)target_heading * 10; // Assuming it was divided by 10 before sending
    }
}