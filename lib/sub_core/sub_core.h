#ifndef SUB_CORE_H
#define SUB_CORE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <Dual_Core_Config.h>

// --- Multiplexer Pins ---
#define s0 A2
#define s1 A3
#define s2 A4
#define s3 A5
#define M1 A0
#define M2 A1

// Motor 4 Pins
#define pwmPin1 2    // PWM 控制腳
#define DIRA_1 3   // 方向控制腳1
#define DIRB_1 4

// Motor 3 Pins
#define pwmPin2 10    // PWM 控制腳
#define DIRA_2 11   // 方向控制腳1
#define DIRB_2 12

// Motor 2 Pins
#define pwmPin3 5    // PWM 控制腳
#define DIRA_3 6   // 方向控制腳1
#define DIRB_3 9

// Motor 1 Pins
#define pwmPin4 23  // PWM 控制腳
#define DIRA_4 37    // 方向控制腳1
#define DIRB_4 36 



// --- Data Structures ---

struct MainCoreCommand {
    float vx = 0.0f;
    float vy = 0.0f;
    float rot_v = 0.0f;
    uint16_t heading = 0; // Target heading in degrees
};

struct LineData {
    uint32_t state = 0;
    bool active = false;
};

struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    bool exist = false;
};

// Robot global configuration and tuning parameters
struct RobotStatus {
    float robot_heading = 90.0f;      // Target heading
    float P_factor = 0.7f;            // Proportional gain for rotation
    float heading_threshold = 10.0f;  // Deadband (degrees)
    int8_t def_pos = 0;               // Default position state
    bool picked_up = false;           // Lift detection flag
};



// --- Global Variable Declarations (Externs) ---
extern LineData line;           
extern MainCoreCommand mainCommand; // Added for MainCoreCommand usage  
extern GyroData gyroData;       
extern RobotStatus robot;        // Added to match Vector_Motion usage
extern uint16_t avg_ls[32];     

// --- Core Function Prototypes ---
void sub_core_init();
int  readMux(int ch, int sig);
void update_line_sensor();
void update_gyro_sensor();
void line_calibrate();

// --- Actuators & IK Prototypes ---
void SetMotorSpeed(uint8_t port, float speed);
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
void readMotor();
void readMotorandSendSensors();
#endif