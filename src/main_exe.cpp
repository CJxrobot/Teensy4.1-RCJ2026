#include "main_core.h"

enum RobotState { STATE_READY, STATE_CALIBRATING, STATE_SAVING };
RobotState currentState = STATE_READY;
unsigned long displayTimer = 0;

void setup() {
    main_core_init(); // Init OLED/Serials
    drawMessage("READY");
}

void loop() {
    readussensor();
    readBallCam();
    static uint32_t lastDisplayTime = 0;
    Serial.println("running");
    switch (currentState) {
        case STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {
                Serial8.write(LS_CAL_START); // Command to Sensor Board
                drawMessage("SCANNING");
                currentState = STATE_CALIBRATING;
                delay(200);
            }
            if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                
                // 顯示指南針 (Heading) 輔助確認感測器是否正常
                display.setCursor(0, 10);
                //display.printf("pitch: %.1f", gyroData.pitch);
                display.printf("angle: %d\n", ballData.angle);
                display.setCursor(0, 25);
                //display.printf("pitch: %.1f", gyroData.pitch);
                display.printf("dist: %d\n", ballData.dist);
                display.display();
                lastDisplayTime = millis();
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
            if(Serial8.available()){
                uint8_t c = Serial8.read();
                if(c == 0xDD){
                    drawMessage("SAVED!");
                    delay(1000); // 讓 SAVED 停一下
                }
                    // 回到初始狀態
                drawMessage("READY");
                delay(200);
                display.clearDisplay();
                currentState = STATE_READY;
            }
            
            break;
    }
}