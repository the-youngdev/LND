#ifndef PIDVALUES_H
#define PIDVALUES_H

// PID & motor defaults
#define DEFAULT_KP 15
#define DEFAULT_KI 0
#define DEFAULT_KD 40
#define DEFAULT_MOTOR_SPEED 90

#define DEFAULT_LOOP_DELAY 2        // minimum loop wait (ms) - small non-blocking wait
#define BRAKE_DURATION_MILLIS 60

// features
#define BLUETOOTH_TUNING_ENABLED 1
#define GAPS_ENABLED 1
#define BRAKING_ENABLED 0

#define TURN_SPEED_REDUCTION_ENABLED 1
#define TURN_SPEED_REDUCTION_PERCENT 15

#define OUT_OF_LINE_ERROR_VALUE 20
#define STOP_CHECK_DELAY 300
#define INTERSECTION_STRAIGHT_MS 50 
#define GAP_STRAIGHT_MS 100  

// debug
#define DEBUG_ENABLED 1             // set to 1 to enable sparse debug prints
#define SERIAL_BAUD 115200

#endif
