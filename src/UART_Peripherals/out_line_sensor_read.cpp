#include <Arduino.h>
void setup(){
    pinMode(A8, INPUT);
    Serial.begin(9600);
}
void loop(){
    Serial.println(analogRead(A8));
    delay(100);
}