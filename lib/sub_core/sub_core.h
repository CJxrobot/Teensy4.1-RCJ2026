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

// --- Motor 1 Pins ---
#define pwmPin1 10
#define DIRA_1  11
#define DIRB_1  12

// --- Motor 2 Pins ---
#define pwmPin2 2
#define DIRA_2  3
#define DIRB_2  4

// --- Motor 3 Pins ---
#define pwmPin3 23
#define DIRA_3  36
#define DIRB_3  37

// --- Motor 4 Pins ---
#define pwmPin4 5
#define DIRA_4  6
#define DIRB_4  9


// --- Data Structures ---

struct MainCoreCommand {
    float vx = 0.0f;
    float vy = 0.0f;
    float deg = 0.0f;
    enum command_type {ACTUATE, CALIBRATE} type;
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
void calibrate();

// --- Actuators & IK Prototypes ---
void SetMotorSpeed(uint8_t port, float speed);
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, int rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);

// Dual Core Communication Prototypes
void sendToCore2();
uint32_t readfrom_MainCore();

#endif