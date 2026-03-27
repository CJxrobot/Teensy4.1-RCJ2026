#include "sub_core.h"

LineData line;
RobotMovement move;
uint16_t avg_ls[32];

void sub_core_init() {
    // Multiplexer Control
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(M1, INPUT_PULLDOWN);
    pinMode(M2, INPUT_PULLDOWN);

    // Motor Initialization
    uint8_t motorPins[] = {pwmPin1, DIRA_1, DIRB_1, pwmPin2, DIRA_2, DIRB_2, 
                           pwmPin3, DIRA_3, DIRB_3, pwmPin4, DIRA_4, DIRB_4};
    for(uint8_t p : motorPins) pinMode(p, OUTPUT);

    // Buttons
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);

    EEPROM.begin(64); 
    EEPROM.get(0, avg_ls);
}

int readMux(int ch, int sig) {
    digitalWrite(s0, (ch >> 0) & 1);
    digitalWrite(s1, (ch >> 1) & 1);
    digitalWrite(s2, (ch >> 2) & 1);
    digitalWrite(s3, (ch >> 3) & 1);
    delayMicroseconds(20);
    return analogRead(sig);
}

void updateLine() {
    float sx = 0, sy = 0;
    int count = 0;
    line.state = 0;

    for (int i = 0; i < 32; i++) {
        int val = readMux(i % 16, (i < 16) ? M1 : M2);
        if (val > avg_ls[i]) {
            line.state |= (1UL << i);
            float rad = i * 0.19635f; 
            sx += cos(rad);
            sy += sin(rad);
            count++;
        }
    }

    line.active = (count > 0);
    if (line.active) {
        move.vx = sx / count;
        move.vy = sy / count;
        move.deg = atan2(sy, sx) * 57.29578f;
        if (move.deg < 0) move.deg += 360.0f;
    } else {
        move.vx = 0; move.vy = 0;
    }
}

void readBNO085Yaw(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.valid = false; // Reset flag before read attempt

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
      gyroData.valid = true;
    }
    
    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }
    break; // Process one packet per call
  }
}

void calibrate() {
    uint16_t max_ls[32], min_ls[32];
    for (int i = 0; i < 32; i++) { max_ls[i] = 0; min_ls[i] = 4095; }

    while (!(Serial8.available() && Serial8.read() == 'E')) {
        for (int i = 0; i < 32; i++) {
            int r = readMux(i % 16, (i < 16) ? M1 : M2);
            if (r > max_ls[i]) max_ls[i] = r; 
            if (r < min_ls[i]) min_ls[i] = r;
        }
    }

    for (int i = 0; i < 32; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
    EEPROM.put(0, avg_ls);
    #if defined(ESP32)
    EEPROM.commit();
    #endif
    Serial8.print('D');
}

/* --- Actuators Part --- */

void SetMotorSpeed(uint8_t port, float speed) {
    // Constrain speed to prevent PWM overflow
    speed = constrain(speed, -255, 255); 
    int pwmVal = abs((int)speed);

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

void Vector_Motion(float Vx, float Vy) {  
    float current_gyro_heading = 90.0f - gyroData.heading;
    float e = robot.robot_heading - current_gyro_heading;

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