#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>



const int Pin=41;
const int Pin1=40;
const int Pin2=39;
volatile bool touch = false;
//void stop();

void setup() {
  pinMode(Pin,INPUT_PULLUP);
  pinMode(Pin1,INPUT_PULLUP);
  pinMode(Pin2,INPUT_PULLUP);
  Serial.begin(9600);
  
  //attachInterrupt(digitalPinToInterrupt(Pin), stop,RISING);
  //attachInterrupt(digitalPinToInterrupt(Pin1), stop,RISING);
  //attachInterrupt(digitalPinToInterrupt(Pin2), stop,RISING);
}
void loop() {
  //Serial.println("Pin");
  //Serial.println(digitalRead(Pin));
 /* Serial.println("Pin1");
  Serial.println(analogRead(Pin1));
  Serial.println(digitalRead(Pin1));*/

  //Serial.println("Pin2");
    
  //Serial.println(analogRead(Pin2));
  //Serial.println(digitalRead(Pin2));

  Serial.println("Pin");
  Serial.println(analogRead(Pin));
  Serial.println(digitalRead(Pin));



  Serial.println("Pin1");
  Serial.println(analogRead(Pin1));
  Serial.println(digitalRead(Pin1));


  Serial.println("Pin2");
  Serial.println(analogRead(Pin2));
  Serial.println(digitalRead(Pin2));


  delay(500); // 每 0.5 秒讀一次

 /*if(touch){
  Serial.println("stop");*/
 }
 /*
 else{
  Serial.println("running");
 }

 delay(500);
}
void stop() {
  touch=true;
}*/