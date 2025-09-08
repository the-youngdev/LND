#include <Arduino.h>
#include "motor_control.h"
#include "Config.h"

void motorInit()
{
  // Motor control pins
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
  
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // Motor driver standby pin
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // default disabled until main enables

  // Initialize motors to stop
  digitalWrite(LEFT_MOTOR_PIN_1, LOW);
  digitalWrite(LEFT_MOTOR_PIN_2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}

void moveStraight(int leftMotorSpeed, int rightMotorSpeed)
{
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    if (leftMotorSpeed >= 0) {
        digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
        digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    } else {
        digitalWrite(LEFT_MOTOR_PIN_1, LOW);
        digitalWrite(LEFT_MOTOR_PIN_2, HIGH);
    }

    if (rightMotorSpeed >= 0) {
        digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
        digitalWrite(RIGHT_MOTOR_PIN_2, LOW);
    } else {
        digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
        digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);
    }

    analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftMotorSpeed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightMotorSpeed));
}

void turnCCW(int leftMotorSpeed, int rightMotorSpeed)
{
#if (TURN_SPEED_REDUCTION_ENABLED == 1)
  leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

#if DEBUG_ENABLED
  Serial.print("Turning CCW with ");
  Serial.print(leftMotorSpeed);
  Serial.print(" and ");
  Serial.println(rightMotorSpeed);
#endif

  // Left motor reverse, right motor forward
  digitalWrite(LEFT_MOTOR_PIN_1, LOW);
  digitalWrite(LEFT_MOTOR_PIN_2, HIGH);

  digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

  analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftMotorSpeed));
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightMotorSpeed));
}


void turnCW(int leftMotorSpeed, int rightMotorSpeed)
{
#if (TURN_SPEED_REDUCTION_ENABLED == 1)
  leftMotorSpeed = (leftMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightMotorSpeed = (rightMotorSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

#if DEBUG_ENABLED
  Serial.print("Turning CW with ");
  Serial.print(leftMotorSpeed);
  Serial.print(" and ");
  Serial.println(rightMotorSpeed);
#endif

  // Left forward, right reverse
  digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN_2, LOW);

  digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

  analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftMotorSpeed));
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightMotorSpeed));
}

void shortBrake(int durationMillis)
{
    // Set PWMs to 0
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);

    // Drive both motor inputs HIGH to short the motor terminals -> braking
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    delay(durationMillis); // short blocking brake is fine for a tiny duration
}

void stop()
{
    digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, LOW);

    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
}
