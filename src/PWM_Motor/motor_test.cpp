#include <Arduino.h>
/*
const int pwmPin = 23;     // PWM 控制腳
const int dirPin1 = 36;    // 方向控制腳1
const int dirPin2 = 37;    // 方向控制腳2
*/
const int pwmPin = 2;     // PWM 控制腳
const int dirPin1 = 3;    // 方向控制腳1
const int dirPin2 = 4;    // 方向控制腳2

const int maxSpeed = 100;  // 最大速度
const int stepDelay = 100;  // 加速/減速延遲(ms)

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // 一開始停住
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);

   pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(dirPin4, OUTPUT);

  // 一開始停住
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1, 0);
}


void loop() {
  MotorTest();
  // 正轉加速
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
<<<<<<< HEAD:src/PWM/motor_test.cpp
  digitalWrite(dirPin3, HIGH);
  digitalWrite(dirPin4, LOW);
  for(int speed = 0; speed <= maxSpeed; speed++) {
=======
  for(int speed = 10; speed <= maxSpeed; speed++) {
    Serial.print("+");
    Serial.println(speed);
>>>>>>> alpha:src/PWM_Motor/motor_test.cpp
    analogWrite(pwmPin, speed);
    analogWrite(pwmPin1, speed);
    delay(stepDelay);
  }

  // 完全停住 1 秒
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1,0);
  delay(1000);
  

  // 反轉加速
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, HIGH);
  for(int speed = 10; speed <= maxSpeed; speed++) {
    Serial.print("-");
    Serial.println(speed);
    analogWrite(pwmPin, speed);
    analogWrite(pwmPin1, speed);
    delay(stepDelay);
  }

  // 完全停住 1 秒
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, 0);
  digitalWrite(dirPin3, LOW);
  digitalWrite(dirPin4, LOW);
  analogWrite(pwmPin1, 0);
  delay(1000);
  
}
*/