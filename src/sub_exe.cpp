#include <sub_core.h>
uint8_t op_mode;

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while(1) {
        update_line_sensor(); // Keep updating sensors!
        update_gyro_sensor();
        // Add logic here
        readMotorCommand();
        //White Line Handling Example
        /*TODO*/
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
        Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
    }
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1){
      update_gyro_sensor();
      update_line_sensor();
      if(Serial8.available()) {
        uint8_t cmd = Serial8.read();
        if(cmd == SUBCORE_SENSOR_DATA){
          sendGyroAndLineToMainCore();
        }
      }
      //readMotorCommand();
      //Serial.printf("vx:%f, vy:%f, heading:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v);
      //FC_Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.heading);
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
    if (Serial8.available()) {
      uint8_t cmd = Serial8.read();
      //Serial.print(cmd);
      if (cmd == LS_CAL_START) {
          uint16_t max_ls[32], min_ls[32];
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
            for (int i = 0; i < 32; i++) {
              int r = readMux(i % 16, (i < 16) ? M1 : M2);
              if (r > max_ls[i]) max_ls[i] = r; 
              if (r < min_ls[i]) min_ls[i] = r;
            }
          }

          for (int i = 0; i < 32; i++) avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
          EEPROM.put(0, avg_ls);
          delay(100); // Ensure EEPROM write completes
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
/*
void loop(){
  update_line_sensor();
  update_gyro_sensor();
  readfrom_MainCore();
  switch (mainCommand.type) {
    case MainCoreCommand::ACTUATE:
      white_line_handle();
      break;
    case MainCoreCommand::CALIBRATE:
      calibrate();
      mainCommand.type = MainCoreCommand::ACTUATE; // Reset to default after calibration
      break;
  }
}
*/