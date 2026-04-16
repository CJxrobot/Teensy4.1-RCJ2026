#include "main_core.h"


// --- Sensor Data ---
CamData camData;
BallData ballData;
USSensor usData;
LineData lineData;

struct Point {
    float x;
    float y;
} targetPos;

// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum RobotState { STATE_READY, STATE_CALIBRATING, STATE_SAVING };
RobotState currentState = STATE_READY;
// --- Kicker Constants ---
#define Charge_Pin 24 // Update to your actual pins
#define Kicker_Pin 25

void main_core_init() {
    // Initialize all Hardware Serials
    Serial.begin(115200);
    Serial2.begin(115200); 
    Serial3.begin(115200);
    Serial4.begin(115200); 
    Serial5.begin(115200);
    Serial6.begin(115200); 
    Serial7.begin(115200);
    Serial8.begin(115200);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Lock if OLED fails
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Pin Setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(Charge_Pin, OUTPUT);
    pinMode(Kicker_Pin, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Ensure kicker starts safe
    digitalWrite(Charge_Pin, LOW);
    digitalWrite(Kicker_Pin, LOW);

    // Button
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_ENTER, INPUT_PULLUP);
    pinMode(BTN_ESC, INPUT_PULLUP);

    // Ultrasonic Sensor
    pinMode(front_us, INPUT_DISABLE);
    //pinMode(back_us, INPUT_DISABLE);
    //pinMode(left_us, INPUT_DISABLE);
    //pinMode(right_us, INPUT_DISABLE);
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

void kicker_control(bool kick) {
    static uint32_t charge_start_time = 0;
    static bool is_charged = false;
    static bool is_charging = false;

    const uint32_t CHARGE_MS = 5000; 
    uint32_t now = millis();

    // 1. Kick Logic (Priority)
    if (kick && is_charged) {
        digitalWrite(Kicker_Pin, HIGH);
        delay(15); // Solenoid pulse
        digitalWrite(Kicker_Pin, LOW);
        
        is_charged = false; 
        is_charging = false;
        Serial.println("KICKED");
        return; // Exit to prevent immediate re-charge trigger in same frame
    }

    // 2. Charging State Machine
    if (!is_charged && !is_charging) {
        // Start a new charge cycle
        charge_start_time = now;
        is_charging = true;
        digitalWrite(Charge_Pin, HIGH);
        Serial.println("CHARGING...");
    }

    if (is_charging) {
        if (now - charge_start_time >= CHARGE_MS) {
            // Charge complete
            digitalWrite(Charge_Pin, LOW);
            is_charging = false;
            is_charged = true;
            Serial.println("READY TO KICK");
        }
    }
}

void readBallCam() {
    static uint16_t buffer[6] = {0};
    static uint16_t idx = 0;
    while(Serial4.available()){
        uint16_t b = Serial4.read();
        if(idx == 0 && b != 0xCC){continue;} //wait for 0xCC
        buffer[idx++] = b;

        if(idx == 6){ //裝包 共6組
            if(buffer[0] == 0xCC && buffer[5] == 0xEE){
              ballData.angle = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
              ballData.dist  = (uint16_t)buffer[3] | ((uint16_t)buffer[4] << 8);
            
               if(ballData.angle != 65535 && ballData.dist != 65535)
                ballData.valid = true;
               else{
                ballData.valid = false;
               }  //無球
            }
            else{
                ballData.valid = false;
            }  //無數據
            idx = 0;  // reset buffer
        }
    }
}

void readFrontCam() {
    static uint8_t buffer[20];
    static uint8_t index = 0;
    while (Serial5.available()){
        uint8_t b = Serial5.read();
        if(index == 0 && b != 0xCC){
            continue;  // 等待開頭 0xCC
        }
        buffer[index++] = b;
        if (index == 18) {
            // 3. 檢查頭尾是否正確
            if (buffer[0] == 0xCC && buffer[17] == 0xEE) {
            
            // --- 解析球 (Ball) ---
            // 把兩個 byte 拼回 16-bit 整數
            int b_x = buffer[1] | (buffer[2] << 8);
            int b_y = buffer[3] | (buffer[4] << 8);
            int b_w = buffer[5] | (buffer[6] << 8);
            int b_h = buffer[7] | (buffer[8] << 8);

            // --- 解析球門 (Goal) ---
            int g_x = buffer[9] | (buffer[10] << 8);
            int g_y = buffer[11] | (buffer[12] << 8);
            int g_w = buffer[13] | (buffer[14] << 8);
            int g_h = buffer[15] | (buffer[16] << 8);

            // 4. 將解析後的資料存入你的 rightData 結構
            // 判斷是否有效：如果在 K210 端沒看到球會傳 65535 (0xFFFF)
            camData.ball_x = b_x;
            camData.ball_y = b_y;
            camData.ball_w = b_w;
            camData.ball_h = b_h;
            camData.ball_valid = (b_x != 65535);

            camData.goal_x = g_x;
            camData.goal_y = g_y;
            camData.goal_w = g_w;
            camData.goal_h = g_h;
            camData.goal_valid = (g_x != 65535);
            }
            index = 0;  // reset buffer
        }
    }
}

void readussensor() {
    
    // 2. Timing control: Sequential reading (The "Better Break")
    static uint32_t lastReadTime = 0;
    static int sensorStep = 0;
    const int pingInterval = 50; // 35ms break between sensors

    // Static variables for low-pass filtering
    static float dist_b_f, dist_l_f, dist_r_f, dist_f_f;

    // Only process ONE sensor every 35ms to prevent acoustic interference
    if (millis() - lastReadTime >= pingInterval) {
        float rawValue = 0;
        switch(sensorStep) {
            case 0: // BACK
                rawValue = analogRead(back_us) * 520.0f / 1024.0f;
                dist_b_f = (alpha * rawValue) + ((1.0f - alpha) * dist_b_f);
                usData.dist_b = (int)dist_b_f;
                sensorStep = 1;
                break;
                
            case 1: // LEFT
                rawValue = analogRead(left_us) * 520.0f / 1024.0f;
                dist_l_f = (alpha * rawValue) + ((1.0f - alpha) * dist_l_f);
                usData.dist_l = (int)dist_l_f;
                sensorStep = 2;
                break;
                
            case 2: // RIGHT
                rawValue = analogRead(right_us) * 520.0f / 1024.0f;
                dist_r_f = (alpha * rawValue) + ((1.0f - alpha) * dist_r_f);
                usData.dist_r = (int)dist_r_f;
                sensorStep = 3;
                break;
                
            case 3: // FRONT
                rawValue = analogRead(front_us) * 520.0f / 1024.0f;
                dist_f_f = (alpha * rawValue) + ((1.0f - alpha) * dist_f_f);
                usData.dist_f = (int)dist_f_f;
                sensorStep = 0; // Reset to start
                break;
        }
        lastReadTime = millis();
    }
/*
    // 防止除以零（設定一個極小距離）
    float safe_l = max(usData.dist_l, 1.0f);
    float safe_r = max(usData.dist_r, 1.0f);
    float safe_f = max(usData.dist_f, 1.0f);
    float safe_b = max(usData.dist_b, 1.0f);

    // --- 計算 X 坐標 (Left & Right) ---
    // 權重與距離成反比
    float w_l = 1.0f / safe_l;
    float w_r = 1.0f / safe_r;
    
    // 假設左傳感器坐標為 -10cm，右傳感器為 +10cm (請根據實際安裝位置修改)
    float left_pos_x = -8.0f; 
    float right_pos_x = 8.0f;
    
    targetPos.x = (w_l * left_pos_x + w_r * right_pos_x) / (w_l + w_r);

    // --- 計算 Y 坐標 (Front & Back) ---
    float w_f = 1.0f / safe_f;
    float w_b = 1.0f / safe_b;

    // 假設前傳感器坐標為 +15cm，後傳感器為 -15cm
    float front_pos_y = 8.0f;
    float back_pos_y = -8.0f;

    targetPos.y = (w_f * front_pos_y + w_b * back_pos_y) / (w_f + w_b);
*/
    // 1. 取得濾波後的距離（確保單位一致，假設為 cm）
    float dl = usData.dist_l;
    float dr = usData.dist_r;
    float df = usData.dist_f;
    float db = usData.dist_b;

    // 2. 避免極小值導致權重爆炸 (Divide by Zero)
    const float min_dist = 1.0f; 
    if (dl < min_dist) dl = min_dist;
    if (dr < min_dist) dr = min_dist;
    if (df < min_dist) df = min_dist;
    if (db < min_dist) db = min_dist;

    // 3. 定義傳感器的「物理安裝位置」(這決定了坐標的量級)
    // 如果你希望坐標範圍大一點，這裡的值要根據機器人實際尺寸設定
    const float L_POS_X = -8.0f; // 左傳感器在 x = -20cm
    const float R_POS_X =  8.0f; // 右傳感器在 x =  20cm
    const float F_POS_Y =  8.0f; // 前傳感器在 y =  20cm
    const float B_POS_Y = -8.0f; // 後傳感器在 y = -20cm

    // 4. 計算權重 (使用平方反比會讓「近距離」的影響力更誇張)
    // 如果想要平緩一點，用 1.0f / dl 即可
    float wl = 1.0f / (dl * dl); 
    float wr = 1.0f / (dr * dr);
    float wf = 1.0f / (df * df);
    float wb = 1.0f / (db * db);

    float obs_l = L_POS_X - dl; // 實際左側物體位置
    float obs_r = R_POS_X + dr; // 實際右側物體位置
    float obs_f = F_POS_Y + df; // 實際前方物體位置
    float obs_b = B_POS_Y - db; // 實際後方物體位置

    targetPos.x = (wl * obs_l + wr * obs_r) / (wl + wr);
    targetPos.y = (wf * obs_f + wb * obs_b) / (wf + wb);
    Serial.printf("pos_x = %f, pos_y = %f\n", targetPos.x, targetPos.y);
}

bool UI_Interface(){
    readussensor();
    readBallCam();

    static uint32_t lastDisplayTime = 0;


    switch (currentState) {

        case STATE_READY:
            if (digitalRead(BTN_ENTER) == LOW) {

                Serial8.write(LS_CAL_START); // Command to Sensor Board

                drawMessage("SCANNING");

                currentState = STATE_CALIBRATING;

                delay(200);

            }

            if (millis() - lastDisplayTime > 100) { // 每 0.1 秒更新一次螢幕
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(0, 0);
                display.printf("ball dist: %d\n", ballData.dist);
                display.printf("ball angle: %d\n",ballData.angle);
                display.printf("us f: %d\n", usData.dist_f);
                display.printf("us l: %d\n", usData.dist_l);
                display.printf("us r: %d\n", usData.dist_r);
                display.printf("us b: %d\n", usData.dist_b);
                display.display();
                lastDisplayTime = millis();
            }
            if(digitalRead(BTN_UP) == LOW){
                return false;
            }
            //offense

            //defense

            break;


        case STATE_CALIBRATING:

            if (digitalRead(BTN_ESC) == LOW) {

                Serial8.write(LS_CAL_END); // Command to Save

                drawMessage("SAVING...");

                currentState = STATE_SAVING;

                delay(200);

            }

            break;



        case STATE_SAVING:

            if(Serial8.available()){

                uint8_t c = Serial8.read();

                if(c == 0xDD){

                    drawMessage("SAVED!");

                    delay(1000); // 讓 SAVED 停一下

                }

                    // 回到初始狀態

                drawMessage("READY");

                delay(200);

                display.clearDisplay();

                currentState = STATE_READY;

            }
            break;
    }
    return true;
}

bool move_to(int pos_x, int pos_y){
    ;
    return true;
}

bool turn_to(int heading){
    ;
    return true;    
}

bool move_in_second(int vx, int vy, int s){
    ;
    return true;
}

bool turn_in_second(int vx, int vy, int s){
    ;
    return true;
}

void update_robot_sensor(){
    
}