#ifndef DUAL_CORE_CONFIG_H
#define DUAL_CORE_CONFIG_H

#define LS_CAL_START 0xAA   // Start Calibration
#define LS_CAL_END   0xEE   // End Calibration
#define LS_CAL_ACK   0xDD   // Acknowledgment (Save Complete)
#define ACT    0xCC   // Start Action/Match Mode
#define T_MODE_HEADER 0x1E
#define C_MODE_HEADER 0x1C



// Toggle these as needed
#define T_MODE
//#define C_MODE 0x1C


// --- Configuration Constants ---
#define MAX_V 60

#endif