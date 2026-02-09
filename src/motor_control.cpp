#include "motor_control.h"
#include "Config.h"

void motorInit()
{
  // Set Motor Control Pins as Output
  pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
  
  // Set PWM Pins as Output
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  // Initialize Standby Pin only if the driver needs it (TB6612FNG)
  #if USE_STANDBY_PIN == 1
      pinMode(STBY, OUTPUT);
      digitalWrite(STBY, LOW); // Start Disabled
  #endif
  
  // Ensure motors are stopped initially
  stop();
}

void moveStraight(int leftMotorSpeed, int rightMotorSpeed)
{
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    // Left Motor Direction
    if(leftMotorSpeed >= 0){
        digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
        digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    } else {
        digitalWrite(LEFT_MOTOR_PIN_2, HIGH);
        digitalWrite(LEFT_MOTOR_PIN_1, LOW);
    }

    // Right Motor Direction
    if(rightMotorSpeed >= 0){
        digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
        digitalWrite(RIGHT_MOTOR_PIN_2, LOW);
    } else {
        digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);
        digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
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

    leftMotorSpeed = constrain(leftMotorSpeed, -80, 100);
    rightMotorSpeed = constrain(rightMotorSpeed, 140, 255);

    // Pivot Left (Left motor back, Right motor fwd)
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

    leftMotorSpeed = constrain(leftMotorSpeed, 140, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -80, 100);

    // Pivot Right (Left motor fwd, Right motor back)
    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    analogWrite(LEFT_MOTOR_PWM_PIN, abs(leftMotorSpeed));
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(rightMotorSpeed));
}

void shortBrake(int durationMillis)
{
    // Active braking: Set both terminals HIGH and PWM to 0
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);

    digitalWrite(LEFT_MOTOR_PIN_1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN_2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN_2, HIGH);

    delay(durationMillis);
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