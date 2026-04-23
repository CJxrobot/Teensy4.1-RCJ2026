#include <Servo.h>
#include <Arduino.h>
Servo ESC;  // Create ESC control object

void setup() {
    main_core_init();
}

void loop() {
    /*
    for (int signal = 1450; signal <= 1500; signal += 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1460
    }
    for (int signal = 1550; signal >= 1500; signal -= 10) {
        ESC.writeMicroseconds(signal);
        Serial.println(signal);
        delay(2000);  // Slow enough for ESC to recognize change
        //Stop at 1500
    }*/
}