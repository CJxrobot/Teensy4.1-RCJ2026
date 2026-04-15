void loop() {
    readussensor();
    readBallCam();
    
    static uint32_t lastDisplayTime = 0;
    static uint32_t stateTimer = 0; // For non-blocking state durations

    switch (currentState) {
        case STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START);
                drawMessage("SCANNING");
                currentState = STATE_CALIBRATING;
            }

            // Non-blocking display update
            if (millis() - lastDisplayTime > 100) {
                updateReadyDisplay(); // Move display logic to a function to keep loop clean
                lastDisplayTime = millis();
            }
            break;

        case STATE_CALIBRATING:
            if (digitalRead(BTN_ESC) == LOW) {
                Serial8.write(LS_CAL_END);
                drawMessage("SAVING...");
                stateTimer = millis(); // Start timeout timer
                currentState = STATE_SAVING;
            }
            break;

        case STATE_SAVING:
            // Check for response OR timeout (2 seconds)
            if (Serial8.available()) {
                uint8_t c = Serial8.read();
                if (c == 0xDD) {
                    drawMessage("SAVED!");
                    stateTimer = millis(); // Reuse timer to show "SAVED" for a moment
                }
            }

            // Return to READY after 1 second of showing "SAVED" or if 2s timeout hits
            if (millis() - stateTimer > 2000) {
                drawMessage("READY");
                currentState = STATE_READY;
            }
            break;
    }
}
