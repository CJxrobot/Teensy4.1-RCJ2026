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
#define BTN_UP    32
#define BTN_DOWN  33
#define BTN_ENTER 34
#define BTN_ESC   35

// --- UI / Menu Globals (Externs) ---
extern int _page;
extern int _cursor;
extern unsigned long _lastPress;
extern unsigned long _lastUpdate;

// --- OLED Instance (Extern) ---
extern Adafruit_SSD1306 display;

// --- Function Prototypes ---
void main_core_init();
void drawMessage(const char* msg);
void updateMenu();        // Logic for button presses
void displayInterface();  // Logic for drawing the screen
void readCameraData();
void readBallCam();
void kicker_control(bool kick);
void localization();

#endif