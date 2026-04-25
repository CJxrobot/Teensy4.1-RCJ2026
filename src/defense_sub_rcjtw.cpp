#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;

// ── 八區 bitmask ──────────────────────────────────────────────────────────────
#define LS_ZONE_R   0x80000007UL  // sensor 31, 0, 1, 2   (中心  0°)
#define LS_ZONE_RU  0x00000078UL  // sensor  3, 4, 5, 6   (中心 45°)
#define LS_ZONE_U   0x00000780UL  // sensor  7, 8, 9,10   (中心 90°)
#define LS_ZONE_LU  0x00007800UL  // sensor 11,12,13,14   (中心135°)
#define LS_ZONE_L   0x00078000UL  // sensor 15,16,17,18   (中心180°)
#define LS_ZONE_LD  0x00780000UL  // sensor 19,20,21,22   (中心225°)
#define LS_ZONE_D   0x07800000UL  // sensor 23,24,25,26   (中心270°)
#define LS_ZONE_RD  0x78000000UL  // sensor 27,28,29,30   (中心315°)

#define ZONE_HIT(mask) ((~lineData.state & (mask)) != 0)

#define SPD   35
#define SPD7  20  // SPD * 0.707


/**
 * @param line_state  32-bit 數據 (0代表壓線, 1代表無線)
 * @param center      目前動態中心點 (0-31)
 * @param range       掃描半徑 (例如 7 代表掃描中心點左右各 7 顆)
 * @return float      輸出的絕對移動角度 (若沒壓線則返回 -1.0)
 */
float get_line_move_angle(uint32_t line_state, uint8_t center, int range) {
    float sum_angle = 0;
    int count = 0;
    const float unit = 11.25f;

    // 從 -range 掃描到 +range
    for (int i = -range; i <= range; i++) {
        
        // 核心：算出環狀索引 (處理 0 和 31 的銜接)
        uint8_t idx = (center + i + 32) % 32;

        // 檢查該位元是否為 0 (代表壓線)
        if (!(line_state & (1UL << idx))) {
            
            // 計算該感測器的絕對角度
            // 這裡直接計算：(索引編號 * 11.25)
            float abs_angle = idx * unit;
            
            sum_angle += abs_angle;
            count++;
        }
    }

    // 如果範圍內有燈亮，計算平均角度
    if (count > 0) {
        return sum_angle / count;
    }

    return -1.0f; // 範圍內沒有壓線
}

void defense_mode() {
    //read_cam_and_pos_data();
    readMainPacket();
    update_line_sensor(); // Keep updating sensors!
    update_gyro_sensor();
    // 預設變數
    float move_deg = -1;
    uint8_t my_center = 0; 

    if (ballData.valid) {
        // 簡單邏輯：球在左邊中心 16，右邊中心 0
        my_center = (ballData.angle > 180) ? 16 : 0;

        // 調用函數：掃描中心點左右各 7 顆感測器 (共 15 顆)
        move_deg = get_line_move_angle(lineData.state, my_center, 7);
    }
    float vx = 0;
    float vy = 0;

    FC_Vector_Motion(vx, vy, 90);
}

void main_function() {
  Serial.println("Cmode Started");
  while (1){
    //read_cam_and_pos_data();
    readMainPacket();
    update_line_sensor(); // Keep updating sensors!
    update_gyro_sensor();
    //Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    //Serial.printf("Ball Valid: %d, Ball Angle: %d, Ball Distance: %d \n", ballData.valid, ballData.angle, ballData.dist);
    Serial.printf("Robot Pos: (%d, %d)\n", RobotPos.x, RobotPos.y);
    Serial.printf("Goal Valid: %d, Goal X: %d, Goal W: %d , Y: %d\n, B: %d\n", goalData.valid, goalData.x, goalData.w, RobotPos.y, usData.dist_b);
    bool move_dir = 0;
    if(ballData.valid){
        if(ballData.angle > 100 && ballData.angle < 270){
            move_dir = -1;
        }
        else if(ballData.angle <80 && ballData.angle > 270){
            move_dir = 1;
        }
        else{
            move_dir = 0;
        }
    }
    if(lineData.state != 0xFFFF){
        if(move_dir == 1){
            ;
        }
        else if(move_dir == -1){
            ;
        }
    }
    else{
        //move back to field
    }

  }
}

void setup(){
  sub_core_init();
  while(1){
    Serial.println("Waiting for MainCore...");
    if(Serial8.available()){
      op_mode = Serial8.read();
      Serial.printf("Received mode: 0x%X\n", op_mode);
      if(op_mode == T_MODE_HEADER || op_mode == C_MODE_HEADER){
        Serial8.write(PROTOCAL_ACT); // Acknowledge receipt
        break;
      }
    }
  }
}

void loop(){
  while(1){
    update_gyro_sensor();
    update_line_sensor();
    Serial.printf("Gyro Heading: %f\n", gyroData.heading);
    for(uint8_t i = 0; i < 32; i++){
      Serial.printf("%d", (lineData.state >> i) & 1);
    }
    Serial.println();
    if (Serial8.available()) {
      uint8_t cmd = Serial8.read();
      //Serial.print(cmd);
      if(cmd == LS_CAL_START) {
          uint16_t max_ls[32], min_ls[32];
          uint16_t front_max = 0, front_min = 4095;
          uint16_t mid_max = 0, mid_min = 4095;
          for (int i = 0; i < 32; i++) { 
            max_ls[i] = 0; 
            min_ls[i] = 4095; 
          }
          int timer = 0;
          while (1) {
            if(micros() - timer > 100000) { // 每 100ms 更新一次
              digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED for visual feedback
              timer = micros();
            }
            if(Serial8.available()){
              uint8_t cmd = Serial8.read();
              if(cmd == LS_CAL_END){ // End calibration command
                break;
              }
            }
            for (int i = 0; i < LS_count; i++) {
              int r = readMux(i % 16, (i < 16) ? 1 : 2);
              if (r > max_ls[i]) max_ls[i] = r; 
              if (r < min_ls[i]) min_ls[i] = r;
            }
            uint16_t reading = analogRead(Front_LS);
            if(reading > front_max) front_max = reading;
            if(reading < front_min) front_min = reading;
          }
          for (int i = 0; i < LS_count; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          for (int i = 0; i < LS_count; i++) {
            Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
          }
          avg_ls[32] = (front_max + front_min) / 2;
          EEPROM.put(0, avg_ls);
          delay(1000); // Ensure EEPROM write completes
          Serial8.write(LS_CAL_ACK); // Send end calibration acknowledgment
      } 
      else if (cmd == MOVE_CMD) {// When BTN_UP is pressed, send a move command to the main core
        Serial8.write(PROTOCAL_ACT);
        break;
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); // Toggle LED for visual feedback
  while(1){
    defense_mode();
  }
}