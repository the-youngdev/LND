#pragma once
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// --- Pin Definitions ---
// Standby Pin (Only used if USE_STANDBY_PIN is 1 in Config.h)
#define STBY 6

// Left Motor Pins
#define LEFT_MOTOR_PIN_1 8
#define LEFT_MOTOR_PIN_2 7
#define LEFT_MOTOR_PWM_PIN 9

// Right Motor Pins
#define RIGHT_MOTOR_PIN_1 4
#define RIGHT_MOTOR_PIN_2 5
#define RIGHT_MOTOR_PWM_PIN 11

// --- Function Prototypes ---
void motorInit();
void moveStraight(int leftMotorSpeed, int rightMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#endif