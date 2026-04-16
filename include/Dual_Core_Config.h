#ifndef DUAL_CORE_CONFIG_H
#define DUAL_CORE_CONFIG_H


// Toggle these as needed
#define T_MODE


// Command List
#define LS_CAL_START 0x65  // Start Calibration
#define LS_CAL_END   0x66  // End Calibration
#define LS_CAL_ACK   0x67  // Acknowledgment (Save Complete)
#define MOVE_CMD     0x68  // Start Command
#define SUBCORE_SENSOR_DATA 0x69  // Sub-core sends sensor data


// Protocal
#define PROTOCAL_HEDAER 0xBB   // Start Action/Match Mode
#define PROTOCAL_HEDAER 0xEE   // Start Action/Match Mode
#define PROTOCAL_ACT    0xCC   // Start Action/Match Mode



// --- Configuration Constants ---
#define MAX_V 60

#endif