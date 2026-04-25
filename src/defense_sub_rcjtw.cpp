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
#define DtoR_const 0.0174529f
#define SPD   35
#define SPD7  20  // SPD * 0.707

#include <math.h>

/**
 * 使用向量求和計算法
 * @param line_state  32-bit 數據 (0代表壓線)
 * @param center      中心點 (0-31)
 * @param range       掃描半徑
 * @return float      絕對角度 (0~360)，沒壓線返回 -1.0
 */
float get_line_move_angle(uint32_t line_state, uint8_t center, int range) {
    float x_sum = 0.0f;
    float y_sum = 0.0f;
    int count = 0;
    const float deg2rad = M_PI / 180.0f; // 角度轉弧度常數

    for (int i = -range; i <= range; i++) {
        uint8_t idx = (center + i + 32) % 32;

        // 檢查該位元是否為 0 (壓線)
        if (!((line_state >> idx) & 1)) {
            // 計算該感測器的絕對物理角度
            float angle_deg = idx * 11.25f;
            float angle_rad = angle_deg * deg2rad;

            // 向量累加
            x_sum += cosf(angle_rad);
            y_sum += sinf(angle_rad);
            count++;
        }
    }

    if (count > 0) {
        // 使用 atan2 算出弧度，並轉回角度
        float result_rad = atan2f(y_sum, x_sum);
        float result_deg = result_rad * (180.0f / M_PI);

        // 將結果標準化到 0~360 度
        if (result_deg < 0) result_deg += 360.0f;
        
        return result_deg;
    }

    return -1.0f;
}

// 計算兩個角度之間的最短夾角
float get_angle_diff(float a, float b) {
    float diff = fabs(a - b);
    if (diff > 180) diff = 360 - diff;
    return diff;
}

void defense_mode() {
    //read_cam_and_pos_data();
    readMainPacket();
    update_line_sensor(); // Keep updating sensors!
    update_gyro_sensor();
    // 預設變數

    //球門限制
    //右側 220
    //左側 100
    float move_deg = -1;
    bool side = false;
    // 簡單邏輯：球在左邊中心 16，右邊中心 0
    if(ballData.valid && (ballData.angle > 270 || ballData.angle < 80)){
        move_deg = get_line_move_angle(lineData.state, 0, 7);
        if(move_deg < 315 && move_deg > 270){
            side = true;
        }
    }
    else if(ballData.valid && (ballData.angle > 100 && ballData.angle < 270)){
        move_deg = get_line_move_angle(lineData.state, 16, 7);
        if(move_deg > 225 && move_deg < 270){
            side = true;
        }
    }
    else{
        move_deg = -1;
        float left_lock_angle = get_line_move_angle(lineData.state, 16, 7);
        float right_lock_angle = get_line_move_angle(lineData.state, 0, 7);

        // 只有當至少有一個角度有效時才計算合向量
        if (left_lock_angle != -1 || right_lock_angle != -1) {
            float vx = 0, vy = 0;

            // 處理左側向量
            if (left_lock_angle != -1) {
                float rad = left_lock_angle * (M_PI / 180.0f);
                vx += cosf(rad);
                vy += sinf(rad);
            }

            // 處理右側向量
            if (right_lock_angle != -1) {
                float rad = right_lock_angle * (M_PI / 180.0f);
                vx += cosf(rad);
                vy += sinf(rad);
            }

            // 兩者的和向量轉回角度
            float res_rad = atan2f(vy, vx);
            move_deg = res_rad * (180.0f / M_PI);
            if (move_deg < 0) move_deg += 360.0f;
            // --- 你的邏輯套用 ---
            if (left_lock_angle != -1 && right_lock_angle != -1) {
                float angle_dist = get_angle_diff(left_lock_angle, right_lock_angle);
                
                // 如果兩側感應到的線夾角大於 120 度 (各自偏離中心 60 度)
                if (angle_dist > 120.0f) { 
                    move_deg = -1; // 衝突太大，視為無效指令
                }
            }
        }
        if(get_angle_diff(left_lock_angle, 180) > 60 || get_angle_diff(right_lock_angle, 0) > 60){
            side = true;
        }
    } 
        // 調用函數：掃描中心點左右各 7 顆感測器 (共 15 顆)
    Serial.printf("Deg %f\n", move_deg);
    float vx = 0;
    float vy = 0;
    if(move_deg != -1){
        float temp = move_deg * DtoR_const;
        vx = 30 * cos(temp);
        vy = 30 * sin(temp);
    }
    if(side && vy < 0){
        vy = 0;
    }
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