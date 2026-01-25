#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

const int Pin=41;
const int Pin1=40;
const int Pin2=39;

volatile bool backtouch = false;
volatile bool lefttouch = false;
volatile bool righttouch = false;
float Vx = 0;
float Vy = 0;

void back();
void left();
void right();

void setup() {
    pinMode(Pin,INPUT_PULLUP);
    pinMode(Pin1,INPUT_PULLUP);
    pinMode(Pin2,INPUT_PULLUP);
    Robot_Init();
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Pin), back,RISING);
    attachInterrupt(digitalPinToInterrupt(Pin1), left,RISING);
    attachInterrupt(digitalPinToInterrupt(Pin2), right,RISING);

}

void loop(){
    /*readBNO085Yaw();
    ballsensor();
    Serial.println(ballData.dis);
    Serial.println(ballData.dir);
    Serial.println(" ");
    if(ballData.dir==3||ballData.dir==4){
        int speed = map(ballData.dis, 2, 9, 0, 30);
        Serial.print("speed");
        Serial.println(speed);
        Vector_Motion(0,speed);
    }
    else{
        Vector_Motion(0,0);
    }*/
    readBNO085Yaw();
    ballsensor();

    if(ballData.dir == 255){
        Serial.println("No ball detected");
        Vector_Motion(0, 0);  // 停止動作
    }
    else{
        float ballDegree = ballDegreelist[ballData.dir];
        float offset = 0;

        Serial.print("balldir=");Serial.println(ballData.dir);
        Serial.print("ballDegree=");Serial.println(ballDegree);
        Serial.print("ballData.dis=");Serial.println(ballData.dis);
        Serial.print("exp");Serial.println(exp(-0.75*ballData.dis));

        map(ballData.dis,0,12,0,10); 

        if((ballDegree >= 22.5 && ballDegree < 87.5) || (ballDegree > 92.5 && ballDegree <= 167.5) ){
            offset = 90*(exp(-0.75*(ballData.dis-4)));            
            offset = (ballDegree > 90 ) ? offset : -offset;
            Serial.print("offset="); Serial.println(offset);
        }

        else if( ballDegree == 87.5 || ballDegree == 92.5){
            offset = 0;
        }

        else if(ballDegree > 167.5 && ballDegree < 385.5){
            offset = 90*(exp(-0.75*(ballData.dis-7)));            
            offset = (ballDegree < 270 ) ? offset : -offset;    
            Serial.print("offset="); Serial.println(offset);           
        }

        float moving_Degree = ballDegree + offset;
        Serial.print("moving_Degree="); Serial.println(moving_Degree);
        
        Vx = 20 * cos(moving_Degree* DtoR_const );
        Vy = 20 * sin( moving_Degree * DtoR_const );

        if (backtouch && Vy < 0) {Vy = 0;}
        if (lefttouch && Vx < 0)  {Vx = 0;}
        if (righttouch && Vx > 0)  {Vx = 0;}


        Vector_Motion( int(Vx) , int(Vy) );
        
    }
    //delay(1000);
    if(digitalRead(Pin)==0){
            backtouch = false;
        }
    if(digitalRead(Pin1)==0){
            lefttouch = false;
        }
    if(digitalRead(Pin2)==0){
            righttouch = false;
        }
}
void back() {
  backtouch = true;
}
void left() {
  lefttouch = true;
}
void right() {
  righttouch = true;
}