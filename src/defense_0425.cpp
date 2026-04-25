#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;


void c_mode_main_function() {
    Serial.println("Cmode Started");
    while (1){
      read_cam_and_pos_data();
      fast_update_line_sensor(); // Keep updating sensors!
      update_gyro_sensor();
    
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
      if (cmd == LS_CAL_START) {
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
            reading = analogRead(Mid_LS);
            if(reading > mid_max) mid_max = reading;
            if(reading < mid_min) mid_min = reading;

          }

          for (int i = 0; i < LS_count; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          for (int i = 0; i < LS_count; i++) {
            Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
          }
          avg_ls[32] = (front_max + front_min) / 2;
          avg_ls[33] = (mid_max + mid_min) / 2;
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
  if(op_mode == C_MODE_HEADER){
    c_mode_main_function();
  }
  else if(op_mode == T_MODE_HEADER){
    t_mode_main_function();
  }
}
