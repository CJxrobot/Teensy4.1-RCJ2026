#include <sub_core.h>

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
}

void loop(){
  update_gyro_sensor();
  Vector_Motion(0,0,0);
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