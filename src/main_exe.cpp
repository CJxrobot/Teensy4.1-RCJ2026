#include "main_core.h"

enum RobotState { STATE_READY, STATE_CALIBRATING, STATE_SAVING };
RobotState currentState = STATE_READY;
unsigned long displayTimer = 0;

void setup() {
    main_core_init(); // Init OLED/Serials
    
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);
    
    drawMessage("READY");
}

void loop() {
    switch (currentState) {
        case STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START); // Command to Sensor Board
                drawMessage("SCANNING");
                currentState = STATE_CALIBRATING;
                delay(200);
            }
            //offense
            //defense
            break;

        case STATE_CALIBRATING:
            if (digitalRead(BTN_ESC) == LOW) {
                Serial8.write(LS_CAL_END); // Command to Save
                drawMessage("SAVING...");
                currentState = STATE_SAVING;
                delay(200);
            }
            break;

        case STATE_SAVING:
            if (Serial8.available() && Serial8.read() == LS_CAL_ACK) { // Wait for acknowledgment
                drawMessage("SAVED!");
                displayTimer = millis();
            }
            if (displayTimer > 0 && (millis() - displayTimer > 1000)) {
                drawMessage("READY");
                displayTimer = 0;
                currentState = STATE_READY;
            }
            break;
    }
}