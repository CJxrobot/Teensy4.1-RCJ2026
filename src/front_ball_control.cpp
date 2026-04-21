#include "Arduino.h"
#include "sub_core.h"

#define trig1 32
#define trig2 31
#define trig3 30
#define trig4 29
#define trig5 28
#define trig6 27
#define trig7 26


void setup() {
    sub_core_init();
    pinMode(trig1, INPUT_PULLDOWN);
    pinMode(trig2, INPUT_PULLDOWN);
    pinMode(trig3, INPUT_PULLDOWN);
    pinMode(trig4, INPUT_PULLDOWN);
    pinMode(trig5, INPUT_PULLDOWN);
    pinMode(trig6, INPUT_PULLDOWN);
    pinMode(trig7, INPUT_PULLDOWN);
}

void loop(){
    update_gyro_sensor();
    uint8_t final_ball_control = 0b00000000; // Bit 0-6 for trig1-7, Bit 7 for ball possession
    // Example: If you have a way to detect ball possession, set bit 7 accordingly
    float vx = 0;
    float vy = 0;
    Serial.printf("GPIO States: %d%d%d%d%d%d%d\n", digitalReadFast(trig1), digitalReadFast(trig2), digitalReadFast(trig3), digitalReadFast(trig4), digitalReadFast(trig5), digitalReadFast(trig6), digitalReadFast(trig7));
    if(digitalReadFast(trig1) && digitalReadFast(trig2) && digitalReadFast(trig3) && digitalReadFast(trig4) && digitalReadFast(trig5) && digitalReadFast(trig6) && digitalReadFast(trig7)){
        vy = 100;
    }
    else if(digitalReadFast(trig1)){
        vx = -MAX_V;
    }
    else if(digitalReadFast(trig2)){
        vx = MAX_V * 0.5;
    }
    else if(digitalReadFast(trig3)){
        vx = MAX_V * 0.1;
    }
    else if(digitalReadFast(trig4)){
        vx = 0;
    }
    else if(digitalReadFast(trig5)){
        vx = -MAX_V * 0.1;
    }
    else if(digitalReadFast(trig6)){
        vx = -MAX_V * 0.5;
    }
    else if(digitalReadFast(trig7)){
        vx = MAX_V;
    }
    Serial.printf("Vx: %f, Vy: %f\n", vx, vy);
    Vector_Motion(vx, vy, 0);
}