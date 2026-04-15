#include "main_core.h"

// 1. FUNCTION PROTOTYPES (Tells the compiler these exist later)
void c_mode_main_function();
void t_mode_main_function();

// Toggle these as needed
#define T_MODE 0x1E
//#define C_MODE 0x1C

void setup() {
    main_core_init(); 
    
    #ifdef C_MODE
        drawMessage("C Mode Locked",);
        Serial8.write(C_MODE);
    #endif
    
    #ifdef T_MODE
        drawMessage("T Mode Locked");
        Serial8.write(T_MODE);
    #endif
}

void loop() {
    // Wait for UI to finish
    while(UI_Interface()); 

    // The code only reaches here AFTER UI_Interface() returns false
    #ifdef C_MODE
    c_mode_main_function();
    #endif
    
    #ifdef T_MODE
    t_mode_main_function();
    #endif
}

// --- Main Logic Functions ---

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while(1) {
        readussensor(); // Keep updating sensors!
        readBallCam();
        // Add logic here
    }
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1) {
        readussensor(); // Keep updating sensors!
        readBallCam();
        // Add logic here
    }
}