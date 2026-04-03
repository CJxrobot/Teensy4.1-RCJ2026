#include "main_core.h"

// --- MEMORY ALLOCATION (Fixes Linker Errors) ---
int _page = 0;
int _cursor = 0;
unsigned long _lastPress = 0;
unsigned long _lastUpdate = 0;

// If these aren't defined in sub_core.cpp, they MUST be here:
// RobotStatus robot; 
// GyroData gyroData;

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

void read_OmniCam() {
    // To be implemented: Serial parsing for OmniCam
}