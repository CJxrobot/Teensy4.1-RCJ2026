#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define DIST_THRESHOLD 50
#define MAX_VY 20
#define MAX_VX 30
#define HORZ_RANGE 70
#define STOP_OFFSET 20
#define OUT_OF_PANELTY 50
#define IN_PANELTY 60
#define SLOW_RATIO 0.2

typedef enum { DEFENSE, OFFENSE, BACK_TO_PENALTY } State;
State robot_state;

uint32_t circular_shift(uint32_t val, int shift);
void update_line();
void defense_main();
void independent_test();

void setup(){
  Robot_Init();
  showMessage("Start");
  display.clearDisplay();
}

bool Debug() {
  readBNO085Yaw();
  linesensor(); 
  readBallCam();
  readGoalCam();
  // --- 顯示球的數據 (新增部分) ---
  static uint32_t lastDisplayTime = 0;
  if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.println("--- DEBUG MODE ---");

    // 顯示球的資訊
    display.setCursor(0, 20);
    if (Ball.exist) {
      display.printf("Ball: DETECTED\n");
      display.printf("deg: %d\n", Ball.deg);
      display.printf("Dist:  %d\n", Ball.dist);
    } else {
      display.printf("Ball: NOT FOUND\n");
    }

    // 顯示指南針 (Heading) 輔助確認感測器是否正常
    display.setCursor(0, 45);
    display.printf("Yaw: %.1f", gyroData.heading);
    //display.printf("pitch: %.1f", gyroData.pitch);
    display.printf("x: %d", Goal.x);
    display.printf("y: %d",Goal.y );
    display.printf("w: %d", Goal.w);
    display.printf("h: %d",Goal.h );
    display.display();
    lastDisplayTime = millis();
  }
  // --- 進入掃描模式 (ENTER) ---
  if (!digitalRead(BTN_ENTER)) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("SCANNING...");
    display.display();

    Serial.println("Enter Scanning Mode");
    Serial7.write(0xAA); 
    delay(300); // 防止按鍵抖動

    // 在掃描期間，依然要維持讀取數據，才看得到結果
    // 這裡用 while 迴圈卡住，直到「再次按下 ESC」
    while (digitalRead(BTN_ESC)) { 
      delay(20); 
    }

    Serial7.write(0xEE); // 發送結束/儲存指令
    Serial.println("Save and Exit Scanning");
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("SAVED!");
    display.display();
    
    delay(500); // 給 ESP32 時間寫入 EEPROM，並防止 ESC 鍵誤觸下一個判斷
  }

  // --- 2. 退出 Debug 模式 (按住 UP 鍵或是長按 ESC) ---
  // 改用 BTN_UP 進入 attack，避免跟掃描模式的 ESC 鍵打架
  if (!digitalRead(BTN_UP)) {
    Serial.println(">>> Exiting Debug. Starting Attack!");
    return false; // 跳出 while(Debug()) 進入 attack
  }

  return true;
}

void loop(){
  while(Debug());
  display.clearDisplay();
  display.display(); 
  Serial.println("OLED Blackout for Competition Mode");
  while (1) {
    defense_main(); // 執行你的比賽邏輯
    //independent_test();
  }
}

// Helper: Circular shift for 18-bit line sensor
uint32_t circular_shift(uint32_t val, int shift) {
    shift %= 18;
    if (shift < 0) shift += 18;
    return ((val << shift) | (val >> (18 - shift))) & 0x3FFFF;
}

void update_line(){
  linesensor();
  int offset = (int)(gyroData.heading) / 20;
  if (offset > 3){
    offset = 3;
    lineData.state = circular_shift(lineData.state, offset);
  } 
  if (offset < -3){
    offset = -3;
    lineData.state = circular_shift(lineData.state, offset);
  }   
}
void update_goal(){
  readGoalCam();
  float p_control = 0.005;
  float offset = 0;
  if(Goal.exist && Goal.x <= 320){
    if(abs(Goal.x -160) < HORZ_RANGE){//center
      if(robot.robot_heading < 80){
          offset = p_control * 20;
      }
      if(robot.robot_heading > 100){
          offset = -p_control * 20;
      }
      robot.def_pos = 0;
    }
    else if(Goal.x > 160 + HORZ_RANGE){//right
      offset = -p_control * abs(Goal.x - (HORZ_RANGE + 160));
      robot.def_pos = 1;
    }
    else if(Goal.x < 160 - HORZ_RANGE){//left
      offset = p_control * abs(Goal.x - (HORZ_RANGE -160)); 
      robot.def_pos = -1;
    }
    else if(Goal.x > 160 + HORZ_RANGE + STOP_OFFSET){//right limit
      offset = -p_control * abs(Goal.x - (HORZ_RANGE + 160));
      robot.def_pos = 2;
    }
    else if(Goal.x < 160 - HORZ_RANGE - STOP_OFFSET){//left limit
      offset = p_control * abs(Goal.x - (HORZ_RANGE -160)); 
      robot.def_pos = -2;
    }
    robot.robot_heading += offset;
    if(gyroData.heading > 30){
      robot.robot_heading = 60;
    } 
    if(gyroData.heading < -30){
      robot.robot_heading = 120;
    }
  }
}

void update_sensor(){
  readBallCam();  
  readBNO085Yaw();
  update_goal();
  update_line();
}

void defense_main(){
  update_sensor();

  Serial.printf("Bool: %d, %d, %d\n", Goal.exist, Ball.exist, lineData.exist);
  // 1. Determine State (Simplified Logic)
  if (lineData.exist && (Ball.exist || !Goal.exist)) {
      robot_state = DEFENSE;
  } 
  if(!lineData.exist){
      robot_state = BACK_TO_PENALTY;
  }
  // 2. Execute Action

  switch (robot_state) {
      case DEFENSE: 
        Serial.println("DEFENSE");
        update_line();
        update_goal();
        if(!((lineData.state >> 4) & 1)){
            robot.vy = MAX_VY;
        }
        else if(!((lineData.state >> 3) & 1) ||!((lineData.state >> 5) & 1)){
            robot.vy = MAX_VY*0.7;
        }
        else if(!((lineData.state >> 2) & 1 )|| !((lineData.state >> 6) & 1)){
            robot.vy = MAX_VY*0.5;
        }
        else if(!((lineData.state >> 1) & 1 )|| !((lineData.state >> 7) & 1)){
            robot.vy = MAX_VY*0.3;
        }
        else if(!((lineData.state >> 0) & 1 )|| !((lineData.state >> 8) & 1)){
            robot.vy = MAX_VY*0;
        }

        if(!((lineData.state >> 13) & 1)){
            robot.vy = -MAX_VY;
        }
        else if(!((lineData.state >> 12) & 1) || !((lineData.state >> 14) & 1)){
            robot.vy = -MAX_VY*0.7;
        }
        else if(!((lineData.state >> 11) & 1) || !((lineData.state >> 15) & 1)){
            robot.vy = -MAX_VY*0.5;
        }
        else if(!((lineData.state >> 10) & 1 )|| !((lineData.state >> 16) & 1)){
            robot.vy = -MAX_VY*0.3;
        }
        else if(!((lineData.state >> 9) & 1 )|| !((lineData.state >> 17) & 1)){
            robot.vy = -MAX_VY*0;
        }
        Serial.printf("vy = %f", robot.vy);
        if(Ball.exist){
          //move right
          if(Ball.deg > 100 && Ball.deg < 270){
            robot.vx = -MAX_VX;
          }
          else if(Ball.deg < 80 || Ball.deg > 270){
            robot.vx = MAX_VX;
          }
          else{
            robot.vx = 0;
          }
          if(robot.def_pos != 0){
            // 3. Apply Boundary Constraints (The "Weird Movement" Fix)
            if(robot.def_pos == 1 && robot.vx > 0) {
                robot.vx = robot.vx * SLOW_RATIO; // Slow down if heading further right
            }
            else if(robot.def_pos == -1 && robot.vx < 0) {
                robot.vx = robot.vx * SLOW_RATIO; // Slow down if heading further left
            }
            
            // Hard stops at the very edge of the goal
            if(robot.def_pos == 2) {
                robot.vx = -10; // Force move back left slightly
            }
            else if(robot.def_pos == -2) {
                robot.vx = 10;  // Force move back right slightly
            }
            if(robot.vy < 0){
              robot.vy = 0; 
            }
          }
        } 
        //robot.robot_heading = 90;
        //robot.vx = 0;
        //robot.vy = 0;
        robot.robot_heading = 90;
        Serial.printf("defPos%d Goal%d Ball%d\n", robot.def_pos, Goal.x, Ball.deg);
        Serial.printf("\nRobot pose: %d, %d, %f\nd", robot.vx, robot.vy, robot.robot_heading);
        FC_Vector_Motion(robot.vx, robot.vy, robot.robot_heading);        
        break;
      case BACK_TO_PENALTY:
        Serial.println("BACK");
        if (Goal.exist && !lineData.exist) {
          if(Goal.h >= IN_PANELTY){
            robot.vy = MAX_VY;
          }
          else if(Goal.h <= OUT_OF_PANELTY){
            robot.vy = -MAX_VY;
          }
        }
        Serial.printf("defPos%d Goal%d Ball%d\n", robot.def_pos, Goal.x, Ball.deg);
        Serial.printf("\nRobot pose: %d, %d, %f\nd", robot.vx, robot.vy, robot.robot_heading);
        FC_Vector_Motion(robot.vx, robot.vy, robot.robot_heading);
        break;
      case OFFENSE:
          // Idle
          break;
          
    }
}


void independent_test(){
  readBNO085Yaw();
  linesensor(); 
  readBallCam();
  readGoalCam();
  Serial.println("sensor read");
  // --- Ball Detection Info ---
  if (Ball.exist) {
      printf("Ball: DETECTED\n");
      printf("deg: %d | Dist: %d\n", Ball.deg, Ball.dist);
  } 
  else {
      printf("Ball: NOT FOUND\n");
  }

  // --- Sensor & Goal Info ---
  // \t adds a tab for cleaner columns in the Serial Monitor
  printf("Yaw: %.1f\t", gyroData.heading);
  printf("Goal - x: %d y: %d w: %d h: %d\n", Goal.x, Goal.y, Goal.w, Goal.h);

  // Adding a separator for readability
  printf("----------------------------\n");
  FC_Vector_Motion(0, 20, 60);
  //Vector_Motion(20, 20);
  for(uint8_t i = 0; i < 18; i++){
    Serial.print(lineData.state >> (i)&1);
  }
  update_line();
  Serial.print("update\n");

  for(uint8_t i = 0; i < 18; i++){
    Serial.print(lineData.state >> (i)&1);
  }
}