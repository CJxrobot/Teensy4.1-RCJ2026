#include "main_core.h"


// --- Sensor Data ---
CamData camData;
BallData ballData;


// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


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
    pinMode(front_us, INPUT);
    pinMode(back_us, INPUT);
    pinMode(left_us, INPUT);
    pinMode(right_us, INPUT);
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

void read_BallCam() {
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

void read_FrontCam() {
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

void readussensor(){
    // static variables remember their values between calls
    static float dist_b_f = 0.0f;
    static float dist_l_f = 0.0f;
    static float dist_r_f = 0.0f;
    static float dist_f_f = 0.0f;

    // read raw ADC and convert to cm (or mm depending on your scaling)
    float dist_b_raw = analogRead(back_us) * 520.0f / 1024.0f;
    float dist_l_raw = analogRead(left_us) * 520.0f / 1024.0f;
    float dist_r_raw = analogRead(right_us) * 520.0f / 1024.0f;
    float dist_f_raw = analogRead(front_us) * 520.0f / 1024.0f;
    // complementary (low-pass) filtering
    dist_b_f = alpha * dist_b_f + (1.0f - alpha) * dist_b_raw;
    dist_l_f = alpha * dist_l_f + (1.0f - alpha) * dist_l_raw;
    dist_r_f = alpha * dist_r_f + (1.0f - alpha) * dist_r_raw;
    dist_f_f = alpha * dist_f_f + (1.0f - alpha) * dist_f_raw;
    // assign filtered values to struct
    usData.dist_b = dist_b_f;
    usData.dist_l = dist_l_f;
    usData.dist_r = dist_r_f;
    usData.dist_f = dist_f_f;
}
