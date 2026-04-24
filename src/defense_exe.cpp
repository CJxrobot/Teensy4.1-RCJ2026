#include <sub_core.h>
uint8_t op_mode;

float lineVx = 0;
float lineVy = 0;

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const 0.0174529f
#define RtoD_const 57.2958f

float ballDegreelist[16]={22.5,45,67.5,87.5,92.5,112.5,135,157.5,202.5,225,247.5,265,275,292.5,315,337.5};
float linesensorDegreelist[32] = {
    0.00, 11.25, 22.50, 33.75, 45.00, 56.25, 67.50, 78.75, 
    90.00, 101.25, 112.50, 123.75, 135.00, 146.25, 157.50, 168.75, 
    180.00, 191.25, 202.50, 213.75, 225.00, 236.25, 247.50, 258.75, 
    270.00, 281.25, 292.50, 303.75, 315.00, 326.25, 337.50, 348.75
};

//Line Sensor
#define EMERGENCY_THRESHOLD 90


int prev_final_degree = -1;
bool moveBackInBounds(){
  //-----LINE SENSOR-----
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  bool linedetected = false;
  static float init_lineDegree = -1;
  static float diff = 0;
  static bool emergency = false;
  static bool start = false;
  static bool overhalf = false;
  static bool first_detect = false;
  static uint32_t speed_timer = 0;
  bool online = false;
  for(int i = 0; i < LS_count; i++){
    if(i==7 ||i==8 ||i==9){
      if(bitRead(lineData.state, i) == 0){
        online = true;
      }
      continue;
    }
    
      
    if(bitRead(lineData.state, i) == 0){
      
      float deg = linesensorDegreelist[i];
      sumX += cos(deg * DtoR_const);
      sumY += sin(deg * DtoR_const);
      count++;
      linedetected = true;
    }
  }

  // B : 反彈

  if(linedetected && count >= 1){
    float lineDegree = atan2(sumY, sumX) * RtoD_const;
    if (lineDegree < 0){lineDegree += 360;} 
    
    //Serial.print("degree=");Serial.println(lineDegree);

    if (!first_detect){
      init_lineDegree = lineDegree;
      first_detect = true;
      speed_timer = millis();
      
      Serial.println("LINE DETECTED !!!");
      Serial.print("initlineDegree =");Serial.println(init_lineDegree);
    }

    diff = fabs(lineDegree - init_lineDegree);
    if(diff > 180){diff = 360 - diff;}
    
    //Serial.print("diff =");Serial.println(diff);


    //-----BACK TO FIELD-----
    float finalDegree;
    if(diff > EMERGENCY_THRESHOLD){
      overhalf = true;
      finalDegree = fmod(init_lineDegree + 180.0f, 360.0f);
    }
    else{
      overhalf = false;
      finalDegree = fmod(lineDegree + 180.0f, 360.0f);
    }
    prev_final_degree = finalDegree;
    Serial.print("finalDegree =");Serial.println(finalDegree);
    /*
    if(finalDegree < 45 || finalDegree >= 315){
        finalDegree = 0;
    }
    */
    if(finalDegree < 135 && finalDegree >= 45){
        finalDegree = 90;
    }
    else if(finalDegree < 225 && finalDegree >= 135){
        finalDegree = 180;
    }
    /*
    else if(finalDegree >= 225 && finalDegree < 315){
      finalDegree = 270;
    }*/
    float speed = 40;

    lineVx = speed *cos(finalDegree * DtoR_const);
    lineVy = speed * 0.5 *sin(finalDegree * DtoR_const);
    bool left = (lineData.state & LS_MASK_LEFT) != LS_MASK_LEFT;
    bool right = (lineData.state & LS_MASK_RIGHT) != LS_MASK_RIGHT;
    bool front = (lineData.state & LS_MASK_FRONT) != LS_MASK_FRONT;
    bool front_in = analogRead(A6) < avg_ls[32];
    if(right && !left){
      if(!front_in){
        lineVy = 20;
      }
    }
    if(!right && left){
      if(!front_in){
        lineVy = 20;
      }
    }
    //if(left && right){
      //lineVy = -20;
    //}
    

/*
    //Reset Vy timer
    if(mid_touch){
    f_back_line_timer = 0;
    f_front_line_timer = 0;
    //          f_back_touch_state = false;
    f_front_touch_state = false; 
    }
    Serial.println(f_front_line_timer, f_back_line_timer);
    if(f_front_line_timer && f_back_line_timer == 0){
    ball_vy = (5 + (millis() - f_front_line_timer) * 0.1);
    if(ball_vy > MAX_V) ball_vy = MAX_V;
    }
    else if(f_front_line_timer == 0 && f_back_line_timer){
    ball_vy = -(5 + (millis() - f_back_line_timer) * 0.1);
    if(ball_vy < -MAX_V) ball_vy = -MAX_V;
    }
    if (RobotPos.y<-100){
    ball_vy = 15;
    }
  */
    return true;
  }
  else{
    prev_final_degree = -1;
    first_detect = false;
    lineVx = 0;
    lineVy = 0;
    speed_timer = 0;
    return false;
  }
}

void c_mode_main_function() {
    Serial.println("Cmode Started");
    while (1){
      read_cam_and_pos_data();
      update_line_sensor(); // Keep updating sensors!
      update_gyro_sensor();
      Serial.printf("Gyro Heading: %f\n", gyroData.heading);
      Serial.printf("Ball Valid: %d, Ball Angle: %d, Ball Distance: %d \n", ballData.valid, ballData.angle, ballData.dist);
      Serial.printf("Robot Pos: (%d, %d)\n", RobotPos.x, RobotPos.y);
       
      //use Ultrasonic Sensor for localization
      if(moveBackInBounds()){
        Serial.printf("MOVING BACK IN BOUNDS %f %f", lineVx, lineVy);
        //if (RobotPos.y<-100){
          //FC_Vector_Motion(lineVx, lineVy + 20, 90);
        //}
        //else{
          FC_Vector_Motion(lineVx, lineVy, 90);
        //}
        
      }
      else{
        static unsigned long f_front_line_timer = 0;
        static unsigned long f_back_line_timer = 0;
        //Vy logic
        float ball_vx = 0;
        float ball_vy = 0;
        //bool f_back_touch = !((lineData.state >> 8) & 1); // Example: using the first line sensor as f_back touch
        //static bool f_back_touch_state = false;
        bool front_touch = /*analogRead(A6) < avg_ls[32] || */analogRead(A7) < avg_ls[33];
        static bool f_front_touch_state = false;
        bool mid_touch = false;
        int checkBits[3] = {7, 8, 9};

        for (int i = 0; i < 3; i++) {
            if (!((lineData.state >> checkBits[i]) & 1)) {
              mid_touch = true;
              break;
            }
        }
        //!((lineData.state >> checkBits[i]) & 1)
        
        //!(lineData.state >> checkBits[i]) & 1
        
        Serial.printf("midt %d\n", A6 || A7);
        bool ball_left = ballData.valid && (ballData.angle > 105 && ballData.angle < 270);
        bool ball_right = ballData.valid && (ballData.angle < 85 || ballData.angle > 270);
        if (ball_left) {
            // 180° = fully left (MAX_V), 90°/270° = barely left (0)
            ball_vx = -MAX_V * (1.0f - abs(180 - ballData.angle) / 90.0f);
        }
        else if (ball_right) {
            // 0°/360° = fully right (MAX_V), 85°/275° = barely right (0)
            int angle = ballData.angle;
            float dist = (angle <= 85) ? angle : (360 - angle); // distance from 0°/360°
            ball_vx = MAX_V * (1.0f - dist / 90.0f);
        }
        else if(!ball_left && !ball_right){
          ball_vx = 0;
        }
        if(front_touch && !f_front_touch_state){
          f_front_touch_state = true;
          f_front_line_timer = millis();
        }

//        if(f_back_touch && !f_back_touch_state){
//          f_back_touch_state = true;
//          f_back_line_timer = millis();
//        }

        //Reset Vy timer
        if(mid_touch){
          f_back_line_timer = 0;
          f_front_line_timer = 0;
//          f_back_touch_state = false;
          f_front_touch_state = false; 
        }
        Serial.println(f_front_line_timer, f_back_line_timer);
        if(f_front_line_timer != 0){
          ball_vy = (5 + (millis() - f_front_line_timer) * 0.1);
          if(ball_vy > 30) ball_vy = 30;
        }
        /*
        if (RobotPos.y<-100){
          ball_vy = 15;
        }*/
        bool left = (lineData.state & LS_MASK_LEFT) != LS_MASK_LEFT;
        bool right = (lineData.state & LS_MASK_RIGHT) != LS_MASK_RIGHT;
        if (prev_final_degree > 135 && prev_final_degree < 225) {
            if (ball_vx > 0) ball_vx = 0;
            if (ball_vy < 0) ball_vy = 0;
        }

        // left side
        if (prev_final_degree  && prev_final_degree < 225) {
            if (ball_vx < 0) ball_vx = 0;
            if (ball_vy > 0) ball_vy = 0;
        }

        Serial.printf("Vx%f,Vy%f\n", ball_vx, ball_vy);
        FC_Vector_Motion(ball_vx, ball_vy, 90);
      }


      /*
      //Ball Tracking
      

      //Vy logic
      if(front_touch && !f_front_touch_state){
        f_front_touch_state = true;
        f_front_line_timer = millis();
      }

      if(f_back_touch && !f_back_touch_state){
        f_back_touch_state = true;
        f_back_line_timer = millis();
      }
      if(vertical_line && f_back_touch_state){
        f_back_touch_state = false;
        f_back_line_timer = 0;
        f_front_touch_state = true;
        f_front_line_timer = millis();
      }
      //Reset Vy timer
      if(mid_touch){
        f_back_line_timer = 0;
        f_front_line_timer = 0;
        f_back_touch_state = false;
        f_front_touch_state = false; 
      }

      //Assign Velocity
      float line_vx = 0; = ball_vx;
      }
      
      // Y axis
      
      */

      //Serial.printf("Front LS: %d, Mid LS: %d, Back LS: %d\n",  int(front_touch), int(mid_touch), int(f_back_touch));
      //Serial.printf("vx: %f, vy: %f\n", vx, vy);
      
    }
}

void t_mode_main_function() {
    Serial.println("Tmode Started");
    while(1) {
        update_line_sensor(); // Keep updating sensors!
        update_gyro_sensor();
        readMotorandSendSensors();
        Serial.printf("vx:%f, vy:%f, rot_v:%f\n", mainCommand.vx, mainCommand.vy, mainCommand.rot_v, mainCommand.heading);
        FC_Vector_Motion(mainCommand.vx, mainCommand.vy, mainCommand.heading); 
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
