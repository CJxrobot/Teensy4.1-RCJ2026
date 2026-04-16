#include "main_core.h"

// 1. FUNCTION PROTOTYPES (Tells the compiler these exist later)
void c_mode_main_function();
void t_mode_main_function();

// --- Main Logic Functions ---

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while(1) {
        ;
    }
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1) {
        Serial.println("Step 1");
        readGyroAndLineFromSubCore();
        Serial.println("Step 2");
        readBallCam();
        Serial.println("Step 3");
        readFrontCam();
        Serial.println("Step 4");
        readussensor();
        Serial.printf("gyro:%d, line:%lu\n", subCoreData.gyroHeading, subCoreData.lineState);
    }
}

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

    while(1) {
        Serial.println("Waiting for SubCore...");
        Serial8.write(header);
        // Wait a short moment for the sub-core to respond 
        if(Serial8.available() > 0) {
            if(Serial8.read() == PROTOCAL_ACT) {
                break; // Connection confirmed
            }
        }
    }
    Serial.println("SubCore exists");
}

void loop() {
    // Wait for UI to finish
    while(UI_Interface()) {
        ;
    }
    Serial8.read(); // Clear the MOVE_CMD from the buffer
    while(Serial8.read() != PROTOCAL_ACT) {
        Serial8.write(MOVE_CMD); // Tell SubCore to start sending commands
    }
    // The code only reaches here AFTER UI_Interface() returns false
    #ifdef C_MODE
        c_mode_main_function();
    #endif
    
    #ifdef T_MODE
        t_mode_main_function();
    #endif
}