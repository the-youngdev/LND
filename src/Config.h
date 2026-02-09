#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
//                       HARDWARE SETTINGS
// ================================================================

// --- Motor Driver Type ---
// Set to 1 if using TB6612FNG (Requires Standby Pin to be HIGH).
// Set to 0 if using L298N (No Standby Pin needed).
#define USE_STANDBY_PIN 0 

// --- PID Control Constants ---
// Tuned values from your working Arduino sketch
#define DEFAULT_KP 10
#define DEFAULT_KI 0
#define DEFAULT_KD 50

// --- Motor Speed Settings ---
#define DEFAULT_MOTOR_SPEED 146   // Base speed (0-255)
#define LEFT_MOTOR_OFFSET 50      // Deduction to balance left motor
#define RIGHT_MOTOR_OFFSET 50     // Deduction to balance right motor

// --- Turning & Braking ---
#define TURN_SPEED_REDUCTION_ENABLED 1
#define TURN_SPEED_REDUCTION_PERCENT 25 // Slow down by 25% on sharp turns

#define BRAKING_ENABLED 0         // Active braking (set to 1 to enable)
#define BRAKE_DURATION_MILLIS 100

// --- System Timings ---
#define DEFAULT_LOOP_DELAY 6      // Loop stability delay
#define STOP_CHECK_DELAY 50       // Debounce for intersection detection
#define OUT_OF_LINE_ERROR_VALUE 20
#define GAPS_ENABLED 1            // Enable logic to handle gaps in the line

#endif