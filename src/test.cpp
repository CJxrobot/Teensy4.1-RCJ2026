#include <Arduino.h>
#include <Robot.h>

#define HEADER 0xCC
#define TAIL   0xEE

// === 解碼 MaixPy 傳來的 UART 封包 ===


void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);  // 接 MaixPy 的 UART
    Robot_Init();
}

void loop() {
    // 讀取 BNO085 姿態
    readBNO085Yaw();
    Serial.print("Pitch: ");
    Serial.println(gyroData.pitch);

    // 解碼 UART 封包
    readCameraData();

    // 如果有有效影像資料
    if (targetData.valid) {
        Serial.print("Target X=");
        Serial.print(targetData.x);
        Serial.print(" Y=");
        Serial.print(targetData.y);
        Serial.print(" W=");
        Serial.print(targetData.w);
        Serial.print(" H=");
        Serial.println(targetData.h);
    }
    control.robot_heading = 90;
    Vector_Motion(0,0);
    
}
