#include "main_core.h"

// 1. FUNCTION PROTOTYPES (Tells the compiler these exist later)
void c_mode_main_function();
void t_mode_main_function();

void setup() {
    main_core_init();
    uint8_t header = 0;

    #ifdef C_MODE
        header = C_MODE_HEADER;
        drawMessage("C Mode Locked");
    #elif defined(T_MODE)
        header = T_MODE_HEADER;
        drawMessage("T Mode Locked");
    #else
        header = C_MODE_HEADER;
        drawMessage("Default");
    #endif

    // Universal Handshake Loop
    while(1) {
        Serial8.write(header);
        
        // Wait a short moment for the sub-core to respond
        delay(50); 
        
        if(Serial8.available() > 0) {
            if(Serial8.read() == ACT) {
                break; // Connection confirmed
            }
        }
    }
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