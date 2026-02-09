#include <Arduino.h>
#include <stdint.h>
#include "motor_control.h"
#include "Config.h"

// --- Logic Definitions ---
#define WHITE_LINE_BLACK_TRACK 1
#define BLACK_LINE_WHITE_TRACK 0
#define SENSOR_COUNT 8
#define POSITIONAL_WEIGHT_W 100 
#define LED 3 // Indicator LED pin

// --- Macros for Sensor Logic ---
#define ALL_SENSORS_DETECT_LINE_COLOR(R) (R == 0b11111111)
#define ALL_SENSORS_OUT_OF_LINE_COLOR(R) (R == 0b00000000)
#define MID_6_SENSORS_DETECT_LINE_COLOR(R) ((R & 0b01111110) == 0b01111110)

// --- Global Variables ---
int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;

int P = 0, I = 0, D = 0;
int previousError = 0;
int PID_value = 0;

int TrackType = BLACK_LINE_WHITE_TRACK;

// ================================================================
//               SENSOR PIN CONFIGURATION (READ CAREFULLY)
// ================================================================

/* --- FOR ARDUINO NANO USERS (DEFAULT) ---
   The Nano has 8 Analog pins (A0-A7). We use all of them.
   - S0 -> A7 (Analog Input Only)
   - S1 -> A6 (Analog Input Only)
   - S2-S7 -> A5-A0 (Digital Capable)
*/
int S0 = A7; 
int S1 = A6; 

/* --- FOR ARDUINO UNO USERS ---
   The Uno ONLY has A0-A5. Pins A6 and A7 DO NOT EXIST.
   You must move S0 and S1 to available Digital Pins.
   
   RECOMMENDED UNO WIRING:
   - Connect Sensor S0 to Digital Pin 2
   - Connect Sensor S1 to Digital Pin 10
   
   INSTRUCTIONS:
   1. Comment out the "Nano" lines above (int S0=A7...).
   2. Uncomment the "Uno" lines below:
*/

// int S0 = 2;   // <--- UNCOMMENT FOR UNO (Pin D2)
// int S1 = 10;  // <--- UNCOMMENT FOR UNO (Pin D10)

// ----------------------------------------------------------------

// Remaining sensors (Compatible with both Uno and Nano)
int S2 = A5;
int S3 = A4;
int S4 = A3;
int S5 = A2;
int S6 = A1;
int S7 = A0;

int error = 0;
int error_dir = 0;
int baseMotorSpeed = DEFAULT_MOTOR_SPEED;

// --- Function to read 8 sensors and pack into a byte ---
uint8_t getSensorReadings()
{
  uint8_t sensorData = 0x00;
  int Sen1, Sen2;

  // ==============================================================
  //           SENSOR READING LOGIC (MODIFY FOR UNO)
  // ==============================================================
  
  // --- OPTION A: ARDUINO NANO (DEFAULT) ---
  // A6 and A7 are Analog-only pins. We must use analogRead().
  // Threshold > 250 converts the analog value to 0 or 1.
  Sen1 = (analogRead(S0) > 250) ? 1 : 0;
  Sen2 = (analogRead(S1) > 250) ? 1 : 0;

  // --- OPTION B: ARDUINO UNO ---
  // IF you switched S0/S1 to Digital Pins above, uncomment these:
  
  // Sen1 = digitalRead(S0); // <--- UNCOMMENT FOR UNO
  // Sen2 = digitalRead(S1); // <--- UNCOMMENT FOR UNO
  
  // Note: For digitalRead(), the sensitivity is set by the 
  // potentiometer on the sensor module itself, not the code.
  // ==============================================================

  // Read remaining sensors (A0-A5 treated as Digital)
  // Software thresholds are IGNORED here; adjust the sensor pot!
  int Sen3 = digitalRead(S2);
  int Sen4 = digitalRead(S3);
  int Sen5 = digitalRead(S4);
  int Sen6 = digitalRead(S5);
  int Sen7 = digitalRead(S6);
  int Sen8 = digitalRead(S7);

  // Pack bits into a single byte
  sensorData |= ((uint8_t)Sen1 << 7) | ((uint8_t)Sen2 << 6) | ((uint8_t)Sen3 << 5) | 
                ((uint8_t)Sen4 << 4) | ((uint8_t)Sen5 << 3) | ((uint8_t)Sen6 << 2) | 
                ((uint8_t)Sen7 << 1) | (uint8_t)Sen8;

  // Auto-detect track color
  // Logic simplified: If middle sensors see line, it's White on Black, etc.
  if (sensorData == 0b01100000 || sensorData == 0b01110000 || sensorData == 0b00110000 ||
      sensorData == 0b00111000 || sensorData == 0b00010000 || sensorData == 0b00011000 ||
      sensorData == 0b00011100 || sensorData == 0b00001000 || sensorData == 0b00001100 ||
      sensorData == 0b00001110 || sensorData == 0b00000100 || sensorData == 0b00000110)
  {
      TrackType = WHITE_LINE_BLACK_TRACK;
  }
  else if (sensorData == 0b10011111 || sensorData == 0b10001111 || sensorData == 0b11011111 ||
           sensorData == 0b11001111 || sensorData == 0b11000111 || sensorData == 0b11101111 ||
           sensorData == 0b11100111 || sensorData == 0b11100011 || sensorData == 0b11110111 ||
           sensorData == 0b11110011 || sensorData == 0b11110001 || sensorData == 0b11111011 ||
           sensorData == 0b11111001)
  {
      TrackType = BLACK_LINE_WHITE_TRACK;
  }

  // Invert logic if we are on a black line
  if (TrackType == BLACK_LINE_WHITE_TRACK)
      sensorData ^= 0b11111111;

  return sensorData;
}

// --- Calculate Weighted Error ---
int getCalculatedError(int fallbackError)
{
    uint8_t sensorReading = getSensorReadings();
    int numeratorSum = 0, denominatorSum = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        uint8_t sensorValue = ((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i));
        numeratorSum += (i + 1) * POSITIONAL_WEIGHT_W * (int)sensorValue;
        denominatorSum += sensorValue;
    }

    int error = fallbackError;
    if (denominatorSum != 0)
        error = ((numeratorSum / (denominatorSum * (POSITIONAL_WEIGHT_W/2))) - (SENSOR_COUNT + 1));
    return error;
}

// --- Check if completely lost ---
int isOutOfLine(uint8_t sensorReadings)
{
    // Patterns that indicate we are at least partially on the line
    uint8_t options[] = {
        0b01100000, 0b01110000, 0b00110000, 0b00111000, 0b00011000, 0b00011100,
        0b00001100, 0b00001110, 0b00000110, 0b10011111, 0b10001111, 0b11001111,
        0b11000111, 0b11100111, 0b11100011, 0b11110011, 0b11110001, 0b11111001
    };
    int n = sizeof(options) / sizeof(uint8_t);
    for (int i = 0; i < n; i++) {
        if (sensorReadings == options[i]) return 0; // Found the line
    }
    return 1; // Lost the line
}

void readSensors()
{
    uint8_t sensorData = getSensorReadings();
    error = getCalculatedError(1);

    int s1 = (sensorData & (1 << 7)) >> 7;
    int s8 = (sensorData & (1 << 0)) >> 0;

    if (s1 != s8) error_dir = s1 - s8;

    // Logic for when line is lost (Gap Handling)
    if (ALL_SENSORS_OUT_OF_LINE_COLOR(sensorData))
    {
        if (error_dir < 0) error = OUT_OF_LINE_ERROR_VALUE;
        else if (error_dir > 0) error = -1 * OUT_OF_LINE_ERROR_VALUE;
    }
    // Logic for Intersection (All sensors see line)
    else if (ALL_SENSORS_DETECT_LINE_COLOR(sensorData))
    {
        moveStraight(baseMotorSpeed, baseMotorSpeed);
        delay(STOP_CHECK_DELAY);
        uint8_t sensorDataAgain = getSensorReadings();
        if (ALL_SENSORS_DETECT_LINE_COLOR(sensorDataAgain))
        {
            shortBrake(100);
            stop();
            delay(10000); // Stop at finish line
        }
    }

    if (MID_6_SENSORS_DETECT_LINE_COLOR(sensorData)) digitalWrite(LED, HIGH);
    else digitalWrite(LED, LOW);
}

void calculatePID()
{
    P = error;
    I = (error == 0) ? 0 : I + error;
    I = constrain(I, -200, 200);
    D = error - previousError;
    
    // Stabilize derivative
    if ((error - previousError) != 0) delay(5);
    
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    PID_value = constrain(PID_value, -200, 200);
    previousError = error;
}

void controlMotors()
{
    // Handle "Out of Line" cases (Rotate to find line)
    if (error == OUT_OF_LINE_ERROR_VALUE)
    {
        #if BRAKING_ENABLED == 1
            shortBrake(BRAKE_DURATION_MILLIS);
        #endif
        uint8_t sensorReadings = getSensorReadings();
        while (isOutOfLine(sensorReadings))
        {
            turnCCW(baseMotorSpeed - LEFT_MOTOR_OFFSET, baseMotorSpeed - RIGHT_MOTOR_OFFSET);
            sensorReadings = getSensorReadings();
        }
        #if GAPS_ENABLED == 1
            error_dir = 0;
        #endif
    }
    else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE))
    {
        #if BRAKING_ENABLED == 1
            shortBrake(BRAKE_DURATION_MILLIS);
        #endif
        uint8_t sensorReadings = getSensorReadings();
        while (isOutOfLine(sensorReadings))
        {
            turnCW(baseMotorSpeed - LEFT_MOTOR_OFFSET, baseMotorSpeed - RIGHT_MOTOR_OFFSET);
            sensorReadings = getSensorReadings();
        }
        #if GAPS_ENABLED == 1
            error_dir = 0;
        #endif
    }
    // Normal PID Following
    else
    {
        int rightMotorSpeed = baseMotorSpeed + PID_value - LEFT_MOTOR_OFFSET;
        int leftMotorSpeed = baseMotorSpeed - PID_value - RIGHT_MOTOR_OFFSET;

        moveStraight(leftMotorSpeed, rightMotorSpeed);
        if (D != 0) delay(DEFAULT_LOOP_DELAY);
    }
}

void setup() {
  Serial.begin(9600);

  // Initialize Pins (Default mode)
  pinMode(S0, INPUT); pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT); pinMode(S5, INPUT); pinMode(S6, INPUT); pinMode(S7, INPUT);

  pinMode(LED, OUTPUT);
  motorInit();
  
  // Enable Motor Driver if using TB6612FNG
  #if USE_STANDBY_PIN == 1
    digitalWrite(STBY, HIGH); 
  #endif
}

void loop() {
  readSensors();
  calculatePID();
  controlMotors();
}