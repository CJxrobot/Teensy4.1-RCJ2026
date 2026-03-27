#ifndef SUB_CORE_H
#define SUB_CORE_H

#include <Arduino.h>
#include <EEPROM.h>

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

// --- Button Pins ---
#define BTN_UP    32
#define BTN_DOWN  33
#define BTN_ENTER 34
#define BTN_ESC   35

// --- Configuration Constants ---
#define MAX_V 100 // Adjust this based on your speed requirements

// --- Data Structures ---
struct LineData {
    uint32_t state;
    bool active;
};

struct RobotMovement {
    float vx, vy, deg;
};

// --- Global Variables (Externs) ---
extern LineData line;
extern RobotMovement move;
extern uint16_t avg_ls[32];

// --- Core Function Prototypes ---
void sub_core_init();
int  readMux(int ch, int sig);
void updateLine();
void calibrate();

// --- Actuators & IK Prototypes ---
void SetMotorSpeed(uint8_t port, float speed);
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);

#endif