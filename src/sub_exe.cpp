#include <sub_core.h>
uint8_t op_mode;
void white_line_handle() {
    if (line.active) {
      Vector_Motion(0, 0, 0); // Stop if on line
    } 
    else {
      Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.deg);
    }
}

void setup(){
  sub_core_init();
  while(1){
    if(Serial8.available()){
      op_mode = Serial8.read();
      if(op_mode == T_MODE_HEADER || op_mode == C_MODE_HEADER){
        Seria8.write(ACT)
        break;
      }
    }
  }
}

void loop(){
  update_gyro_sensor();
  if (Serial8.available()) {
    uint8_t cmd = Serial8.read();
    //Serial.print(cmd);
    if (cmd == LS_CAL_START) {
      line_calibrate(); // 進入校準模式
    } n 
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