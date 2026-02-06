#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity

//Line Sensor
#define EMERGENCY_THRESHOLD 50

//BALL SEARCHING THRESHOLD
#define BALL_Threshold 5
#define TOTAL_BALL_SENSORS 10

//ROBOT MAX SPEED
#define MAX_V 40

//ROBOT DEFENSE PARAMETERS
#define MAX_VX 60
#define MAX_VY 60
#define Def_offset 2.5
#define Back_safe 35//cm
#define Side_safe 45//cm
#define Back_limit 15
#define Side_limit 45

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f

// --- PIN DEFINITIONS ---
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26
// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Motor 1 Pins
#define pwmPin1 10    // PWM 控制腳
#define DIRA_1 11   // 方向控制腳1
#define DIRB_1 12

// Motor 2 Pins
#define pwmPin2 2    // PWM 控制腳
#define DIRA_2 3   // 方向控制腳1
#define DIRB_2 4

// Motor 3 Pins
#define pwmPin3 23    // PWM 控制腳
#define DIRA_3 36   // 方向控制腳1
#define DIRB_3 37

// Motor 4 Pins
#define pwmPin4 5   // PWM 控制腳
#define DIRA_4 6    // 方向控制腳1
#define DIRB_4 9



//Kicker
#define Charge_Pin 33 //FET1
#define Kicker_Pin 32 //FET2


//Interrupt
volatile bool backtouch = false;
volatile bool lefttouch = false;
volatile bool righttouch = false;

// --- GLOBAL OBJECTS & STRUCTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct GyroData{float heading = 0.0; float pitch = 0.0; bool valid = false;} gyroData;
struct LineData{uint32_t state = 0x3FFFF; bool exist = false;} lineData;
struct BallData{bool exist = false; int16_t deg = -1; int16_t dist = -1;} Ball;
struct GoalData{bool exist = false; uint16_t x = 65535;uint16_t y = 65535;uint16_t w = 65535;uint16_t h = 65535; int16_t deg = -1; int16_t dist = -1; }Goal;

float ballDegreelist[16]={22.5,45,67.5,87.5,92.5,112.5,135,157.5,202.5,225,247.5,265,275,292.5,315,337.5};
float linesensorDegreelist[18]={10,30,50,70,90,110,130,150,170,190,210,230,250,270,290,310,330,350};
int8_t linesensor_ver_cor[18]={1,2,3,4,5,4,3,2,1,-1,-2,-3,-4,-5,-4,-3,-2,-1};

// --- ROBOT CONTROL STRUCT (New: For P-control state) ---
struct RobotControl{
    float robot_heading = 90.0;        // Target heading
    float P_factor = 0.7;             // Proportional gain
    float heading_threshold = 10.0;     // Deadband (degrees)
    int8_t vx = 0;
    int8_t vy = 0;
    int8_t def_pos = 0;
    bool picked_up = false;
} robot;


// --- FUNCTION PROTOTYPES ---
// Including prototypes for the new functions and existing ones
void Robot_Init();
void readBNO085Yaw();
void ballsensor();
void linesensor();
void positionEst();
void showStart();
void showLine();
void showRunScreen();
void showMessage(const char* message, int textSize = 2, int x = 0, int y = 20);
void showSensors(float gyro, int ball, int light);
void SetMotorSpeed(uint8_t port, int8_t speed);
void MotorStop();
void RobotIKControl(int8_t vx, int8_t vy, float omega);
void Vector_Motion(float Vx, float Vy);
void Degree_Motion(float moving_degree, int8_t speed);
void kicker_control(bool);
bool white_line_processing();
void backlstouch();
void leftlstouch();
void rightlstouch();

// ******************************************************
// --- FUNCTION IMPLEMENTATIONS (Existing & New) ---
// ******************************************************

void Robot_Init(){
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  Serial8.begin(115200);
  
  pinMode(pwmPin1,OUTPUT);
  pinMode(DIRA_1,OUTPUT);
  pinMode(DIRB_1,OUTPUT);

  pinMode(pwmPin2,OUTPUT);
  pinMode(DIRA_2,OUTPUT);
  pinMode(DIRB_2,OUTPUT);

  pinMode(pwmPin3,OUTPUT);
  pinMode(DIRA_3,OUTPUT);
  pinMode(DIRB_3,OUTPUT);

  pinMode(pwmPin4,OUTPUT);
  pinMode(DIRA_4,OUTPUT);
  pinMode(DIRB_4,OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_ESC, INPUT_PULLUP);


  pinMode(Kicker_Pin, OUTPUT);
  pinMode(Charge_Pin, OUTPUT);

  digitalWrite(Kicker_Pin, LOW);
  digitalWrite(Charge_Pin, LOW);
  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  kicker_control(0);

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

void readGoalCam(){
  static uint8_t buffer[10];
  uint8_t index = 0;
  while(Serial5.available()){
    uint8_t b = Serial5.read();
    if(index == 0 && b != 0xCC){
      continue;  // 等待開頭 0xCC
    }
    buffer[index++] = b;
    if(index == 10){  // 收滿 10 bytes
      if(buffer[0] == 0xCC && buffer[9] == 0xEE){
        Goal.x = buffer[1] | (buffer[2] << 8);
        Goal.y = buffer[3] | (buffer[4] << 8);
        Goal.w = buffer[5] | (buffer[6] << 8);
        Goal.h = buffer[7] | (buffer[8] << 8);
        Goal.exist = true;  
        if(Goal.x == 65535 && Goal.y == 65535 && Goal.w == 65535 && Goal.h == 65535){
          Goal.exist = false;  
        }            
      }
      index = 0;  // reset buffer
    }
  }
}

void readBallCam(){
    static uint8_t buffer[6] = {0};
    static uint8_t idx = 0;
    while(Serial4.available()){
        uint8_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){ //裝包 共6組
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
               Ball.deg = buffer[1] | (buffer[2] << 8);
               Ball.dist = buffer[3] | (buffer[4] << 8);
            
               if(Ball.deg != 65535 && Ball.dist != 65535)
                Ball.exist = true;
               else{
                Ball.exist = false;
               }  //無球
            }
            else{
                Ball.exist = false;
            }  //無數據
            idx = 0;  // reset buffer
        }  
    }
}

void linesensor(){
  uint8_t buffer[7];
  Serial7.write(0xdd);
  while(!Serial7.available());
  Serial7.readBytes(buffer,7);
  lineData.exist = false;
  if(buffer[0] != 0xaa) return;
  if(buffer[0] == 0xAA && buffer[6] == 0xEE){
    uint8_t checksum = (buffer[1] + buffer[2] + buffer[3] + buffer[4]) & 0xFF;
    if(checksum == buffer[5]){
      
      lineData.state = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);     
      if(lineData.state != 0b111111111111111111){
        //Vector_Motion(0,0);  // Stop robot if line detected
        lineData.exist = true;
      }
    }
  }
  else{
    lineData.exist = false;  // checksum error
  }
}


void showMessage(const char* message, int textSize = 2, int x = 0, int y = 20){
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setCursor(x, y);
  display.println(message);
  display.display();
}

void showSensors(float gyro, int ball, int light){
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Gyro:");  display.println(gyro);
  display.setCursor(0, 15); display.print("Ball:");  display.println(ball);
  display.setCursor(0, 30); display.print("Light:"); display.println(light);
  display.display();
}

/*Actuators Part*/
void SetMotorSpeed(uint8_t port, int8_t speed){
  Serial.printf("M%d : %d", port, speed);
  speed = constrain(speed,-1.5 * MAX_V, 1.5 * MAX_V);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 4:
      analogWrite(pwmPin1, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_1,HIGH);
        digitalWrite(DIRB_1,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,HIGH);
      } else{
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,LOW);
      }
      break;
    case 3:
      analogWrite(pwmPin2, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_2,HIGH);
        digitalWrite(DIRB_2,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,HIGH);
      } else{
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,LOW);
      }
      break;
    case 2:
      analogWrite(pwmPin3, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_3,HIGH);
        digitalWrite(DIRB_3,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,HIGH);
      } else{
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,LOW);
      }
      break;
    case 1:
      analogWrite(pwmPin4, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_4,HIGH);
        digitalWrite(DIRB_4,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,HIGH);
      } else{
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,LOW);
      }
      break;
  }
}

void MotorStop(){
  digitalWrite(DIRA_1,LOW);
  digitalWrite(DIRB_1,LOW);
  digitalWrite(DIRA_2,LOW);
  digitalWrite(DIRB_2,LOW);
  digitalWrite(DIRA_3,LOW);
  digitalWrite(DIRB_3,LOW);
  digitalWrite(DIRA_4,LOW);
  digitalWrite(DIRB_4,LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}

void RobotIKControl(int8_t vx, int8_t vy, float omega){
  // Note: Cast omega to int8_t for consistent data types in the IK control matrix

    int8_t p1 = (int8_t)(-0.643 * vx + 0.766 * vy + omega);
    int8_t p2 = (int8_t)(-0.643 * vx - 0.766 * vy + omega);
    int8_t p3 = (int8_t)( 0.707 * vx - 0.707 * vy + omega);
    int8_t p4 = (int8_t)( 0.707 * vx + 0.707 * vy + omega);
    SetMotorSpeed(1, p1);
    SetMotorSpeed(2, p2);
    SetMotorSpeed(3, p3);
    SetMotorSpeed(4, p4);
}

void Vector_Motion(float Vx, float Vy){  
  float omega = 0.0;
  float current_gyro_heading = gyroData.heading;
  float sensor_heading = 90.0 - current_gyro_heading;
  float e = robot.robot_heading - sensor_heading;
  if(fabs(e) > robot.heading_threshold){
      omega = e * robot.P_factor * 0.5;
  }
  RobotIKControl((int8_t)Vx, (int8_t)Vy, omega);
}

void FC_Vector_Motion(int WVx, int WVy, float target_heading) {
    // 1. Convert gyro to Radians (math functions use radians)
    float rad = (target_heading-90)* (M_PI / 180.0);
    float cos_h = cos(rad);
    float sin_h = sin(rad);

    // 2. Rotate World Vectors to Robot Frame
    int8_t robot_vx = (int8_t)(WVx * cos_h + WVy * sin_h);
    int8_t robot_vy = (int8_t)(-WVx * sin_h + WVy * cos_h);
    //Serial.printf("robot %d, %d\n", robot_vx, robot_vy);
    // 3. Calculate Heading Correction (Omega)
    float omega = 0;
    float current_gyro_heading = 90 - gyroData.heading;
    // Normalize error to find the shortest path to target_heading
    float e = target_heading - current_gyro_heading;
    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    if (fabs(e) > robot.heading_threshold) {
        omega = e * robot.P_factor ;
    }
    //Serial.printf("omege%d\n", omega);
    // 4. Send to IK Control
    RobotIKControl(robot_vx, robot_vy, (int8_t)omega);
}

void Degree_Motion(float moving_degree, int8_t speed){
  if(moving_degree > 360.0 || moving_degree < 0.0){
      MotorStop();
  }
  float moving_degree_rad = moving_degree * DtoR_const;
  float Vx = cos(moving_degree_rad) * speed;
  float Vy = sin(moving_degree_rad) * speed;
  Vector_Motion(Vx, Vy);
}

void kicker_control(bool kick = false){
  static uint64_t charge_start = 0;
  static uint64_t last_charge_done = 0;
  static bool charging_state = false;

  const uint32_t CHARGE_DURATION = 5000;   // ms needed to charge
  const uint32_t CHARGE_TIMEOUT  = 8000;  // ms before recharging automatically

  uint64_t now = millis();

  // Auto-recharge if too long since last charge
  if(charging_state && (now - last_charge_done > CHARGE_TIMEOUT)){
    charging_state = false;
  }

  // Start charging if not charged and not already charging
  if(!charging_state && charge_start == 0){
    charge_start = now;
    //Serial.println("Charge");
    digitalWrite(Charge_Pin, HIGH);
    digitalWrite(Kicker_Pin, LOW);
  }

  // Stop charging when duration is met
  if(charge_start != 0 && (now - charge_start >= CHARGE_DURATION)){
    digitalWrite(Charge_Pin, LOW);
    digitalWrite(Kicker_Pin, LOW);
    //Serial.println("Charge End");
    charging_state = true;
    charge_start = 0;
    last_charge_done = now;
  }

  // Perform kick if charged
  if(kick && charging_state){
    digitalWrite(Kicker_Pin, HIGH);
    delay(10);
    digitalWrite(Kicker_Pin, LOW);
    //delay(10);
    // After kick, reset to recharge again
    Serial.println("kick");
    charging_state = false;
  }
<<<<<<< HEAD
}  
=======
}

// INTERRUPT
void backlstouch(){ backtouch = true; }
void leftlstouch(){ lefttouch = true; }
void rightlstouch(){ righttouch = true; }
>>>>>>> 8cfc299c8b87958b145983642c222a52eed4bb8f
