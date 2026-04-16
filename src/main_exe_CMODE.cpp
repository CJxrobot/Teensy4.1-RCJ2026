#include "main_core.h"

#define T_MODE
//#define C_MODE

void setup() {
    main_core_init(); // Init OLED/Serials
    #ifdef C_MODE
        drawMessage("C Mode");
        Serial8.write();
        Serial8.write();
        
    #endif
    
    #ifdef T_MODE
        drawMessage("T Mode");
        Serial8.write();
        Serial8.write();
        
    #endif
}

void loop() {
    while(UI_interface());
    #ifdef C_MODE
        c_mode_main_function();
    #endif
    
    #ifdef T_MODE
        t_mode_main_function();
    #endif
}

void ccmode_main_function(){
    while(1){
        readussensor();
        readBallCam();

    }
}

void t_mode_main_function(){
    while(1){
        readussensor();
        readBallCam();

    }
}