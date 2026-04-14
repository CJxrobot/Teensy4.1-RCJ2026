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


extern struct CamData{uint16_t ball_x = 65535;uint16_t ball_y = 65535;uint16_t ball_w = 65535;uint16_t ball_h = 65535; bool ball_valid = false;uint16_t goal_x = 65535;uint16_t goal_y = 65535;uint16_t goal_w = 65535;uint16_t goal_h = 65535; bool  goal_valid = false;} camData;
extern struct BallData{uint16_t dist = 255; uint16_t angle = 255; uint16_t possession = 255; bool valid = false; float Vx; float Vy;} ballData;
extern struct USSensor{uint16_t dist_b = 0; uint16_t dist_l = 0; uint16_t dist_r = 0;uint16_t dist_f = 0; } usData;

// --- OLED Instance (Extern) ---
extern Adafruit_SSD1306 display;

// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void readBallCam();
void readFrontCam();
void readussensor();
void kicker_control(bool kick);
void localization();

#endif