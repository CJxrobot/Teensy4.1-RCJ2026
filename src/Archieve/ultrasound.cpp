#include <Arduino.h>
#include <Robot.h>
#include <math.h>


float vx = 0;
float vy = 0;
float repl = 0;

void setup(){
Robot_Init();
}
void loop() {
    readBNO085Yaw();
    readussensor();
    vy = 40;
    if(usData.dist_f < 70){;
        repl --;
        if(repl < -40){
            repl = -40;
        }
        vy += repl;
    }
    Vector_Motion(0,vy);
    Serial.print(vy);
    repl = 0;
}