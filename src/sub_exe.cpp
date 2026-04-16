#include <sub_core.h>
uint8_t op_mode;

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while(1) {
        update_line_sensor(); // Keep updating sensors!
        update_gyro_sensor();
        // Add logic here
        ReadMainCoreCommand();
        //White Line Handling Example
        /*TODO*/
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
      ReadMainCoreCommand();
      FC_Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.heading);
    }
}

void setup(){
  sub_core_init();
  while(1){
    if(Serial8.available()){
      op_mode = Serial8.read();
      if(op_mode == T_MODE_HEADER || op_mode == C_MODE_HEADER){
        Serial8.write(ACT);
        break;
      }
    }
  }
}

void loop(){
  update_gyro_sensor();
  while(1){
    if (Serial8.available()) {
      uint8_t cmd = Serial8.read();
      //Serial.print(cmd);
      if (cmd == LS_CAL_START) {
      }
      else if (cmd == MOVE_CMD) {
        Serial8.write(PROTOCAL_ACT);
        break;
      }
    }
  }
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