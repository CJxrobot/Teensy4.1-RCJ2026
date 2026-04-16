#ifndef MAIN_CORE_H
#define MAIN_CORE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Dual_Core_Config.h"

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Button Pins ---
#define BTN_UP 31
#define BTN_DOWN 30
#define BTN_ENTER 27
#define BTN_ESC 26

// --- Ultrasonic ---
#define front_us A15
#define left_us A14
#define back_us A16
#define right_us A17
#define alpha 0.75  


// --- 1. Blueprints (Struct Definitions) ---
// We define these so every file knows the "shape" of the data.
struct CamData {
    uint16_t ball_x = 65535; uint16_t ball_y = 65535;
    uint16_t ball_w = 65535; uint16_t ball_h = 65535;
    bool ball_valid = false;
    uint16_t goal_x = 65535; uint16_t goal_y = 65535;
    uint16_t goal_w = 65535; uint16_t goal_h = 65535;
    bool goal_valid = false;
};

struct BallData {
    uint16_t dist = 255; uint16_t angle = 255;
    uint16_t possession = 255; bool valid = false;
    float Vx; float Vy;
};

struct USSensor {
    uint16_t dist_b = 0; uint16_t dist_l = 0;
    uint16_t dist_r = 0; uint16_t dist_f = 0;
};

struct LineData {
    bool exist; uint32_t state;
};

// --- 2. Communication Packets (Packed) ---
typedef struct __attribute__((packed)) {
    uint8_t header; // 0xAA
    float vx;       // Translation X
    float vy;       // Translation Y
    float vrot;     // Rotation
} MovePacket;

typedef struct __attribute__((packed)) {
    uint8_t header;     // 0xBB
    uint32_t lineState; // 32-bit sensor data
    float heading;      // Gyro heading
} FeedbackPacket;

// --- 3. External Variables ---
// These tell the compiler "The actual memory for these is in main.cpp"
extern CamData camData;
extern BallData ballData;
extern USSensor usData;
extern LineData lineData;
extern Adafruit_SSD1306 display;


// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void readBallCam();
void readFrontCam();
void readussensor();
void kicker_control(bool kick);
bool UI_Interface();
void localization();

#endif