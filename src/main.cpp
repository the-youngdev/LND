#include <Arduino.h>
#include <stdint.h>
#include "motor_control.h"
#include "Config.h"

#define WHITE_LINE_BLACK_TRACK 1
#define BLACK_LINE_WHITE_TRACK 0
#define SENSOR_COUNT 8
#define POSITIONAL_WEIGHT_W 100 

#define CMD_STANDBY    's'
#define CMD_RUN        'r'

#define ALL_SENSORS_DETECT_LINE_COLOR(R) (R == 0b11111111)
#define ALL_SENSORS_OUT_OF_LINE_COLOR(R) (R == 0b00000000)
#define IS_ACUTE_LEFT(R)  ((R == 0b11100000) || (R == 0b11000000))
#define IS_ACUTE_RIGHT(R) ((R == 0b00000111) || (R == 0b00000011))

#define MID_6_SENSORS_DETECT_LINE_COLOR(R) ((R & 0b01111110) == 0b01111110)

#define LED 3

String dataBL = "";

int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;

int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int PID_value = 0;

int TrackType = BLACK_LINE_WHITE_TRACK;

int S0 = A0;
int S1 = A1;
int S2 = A2;
int S3 = A3;
int S4 = A4;
int S5 = A5;
int S6 = A6;
int S7 = A7;

int loopDelay = DEFAULT_LOOP_DELAY;
int leftMotorOffset = 50;
int rightMotorOffset = 50;
int error = 0;
int error_dir = 0;
int baseMotorSpeed = DEFAULT_MOTOR_SPEED;

// calibration thresholds (0..1023)
int thresholds[SENSOR_COUNT] = {400,400,400,400,400,400,400,400};
bool calibrated = false;

// helper: map analog readings to digital 0/1 using thresholds
inline uint8_t analogToDigital(int reading, int threshold) {
  return (reading > threshold) ? 1 : 0;
}

uint16_t rawAnalog[SENSOR_COUNT];

uint8_t getSensorReadingsRaw() {
  // read all 8 as analog for consistency and precision
  rawAnalog[0] = analogRead(S0);
  rawAnalog[1] = analogRead(S1);
  rawAnalog[2] = analogRead(S2);
  rawAnalog[3] = analogRead(S3);
  rawAnalog[4] = analogRead(S4);
  rawAnalog[5] = analogRead(S5);
  rawAnalog[6] = analogRead(S6);
  rawAnalog[7] = analogRead(S7);

  uint8_t sensorData = 0;
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    uint8_t bit = analogToDigital(rawAnalog[i], thresholds[i]);
    sensorData |= (bit << (7 - i)); // MSB = leftmost
  }
  return sensorData;
}

// In calibrateSensorsAndTrackType()
void calibrateSensorsAndTrackType(unsigned long timeoutMs = 3000) {
  // Simple calibration routine:
  // 1) User places robot over line center -> press power
  // 2) We sample for 'timeoutMs' and compute thresholds between line and background
  unsigned long start = millis();
  uint32_t sumOn[SENSOR_COUNT] = {0};
  uint32_t cntOn = 0;

  Serial.println("Calibration: place robot with sensors on LINE (center) for 2s...");
  delay(2000);
  while (millis() - start < timeoutMs) {
    for (int i = 0; i < SENSOR_COUNT; ++i) sumOn[i] += analogRead(A0 + i);
    cntOn++;
    delay(10);
  }

  // then read background
  Serial.println("Calibration: place robot off line (background) for 2s...");
  delay(2000);
  start = millis();
  uint32_t sumOff[SENSOR_COUNT] = {0};
  uint32_t cntOff = 0;
  while (millis() - start < timeoutMs) {
    for (int i = 0; i < SENSOR_COUNT; ++i) sumOff[i] += analogRead(A0 + i);
    cntOff++;
    delay(10);
  }

  // compute thresholds halfway between average on-line and off-line
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    int avgOn = (int)(sumOn[i] / max(1U, cntOn));
    int avgOff = (int)(sumOff[i] / max(1U, cntOff));
    thresholds[i] = (avgOn + avgOff) / 2;
    // ensure threshold is within reasonable bounds
    thresholds[i] = constrain(thresholds[i], 80, 900);
  }

  // Determine track color by comparing averages
  int brightnessOn = 0, brightnessOff = 0;
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    brightnessOn += (int)(sumOn[i] / max(1U, cntOn));
    brightnessOff += (int)(sumOff[i] / max(1U, cntOff));
  }

  // *** FIX: Logic swapped to correctly identify track type ***
  if (brightnessOn < brightnessOff) {
    TrackType = WHITE_LINE_BLACK_TRACK; // Line is brighter (lower analog value)
    Serial.println("TrackType: WHITE_LINE_BLACK_TRACK");
  } else {
    TrackType = BLACK_LINE_WHITE_TRACK; // Line is darker (higher analog value)
    Serial.println("TrackType: BLACK_LINE_WHITE_TRACK");
  }

  calibrated = true;
  Serial.println("Calibration complete. Thresholds:");
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(thresholds[i]); Serial.print(i == SENSOR_COUNT-1 ? "\n" : ", ");
  }
}
// Convert sensor bitmap to normalized error (integer). fallbackError used if none detected.
// In getCalculatedError()
// In getCalculatedError()
int getCalculatedError(int fallbackError)
{
    uint8_t sensorReading = getSensorReadingsRaw();

    if (TrackType == WHITE_LINE_BLACK_TRACK) {
      sensorReading = ~sensorReading;
    }

    int numeratorSum = 0;
    int denominatorSum = 0;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        uint8_t sensorValue = ((sensorReading >> (SENSOR_COUNT - 1 - i)) & 0x01);
        int weight = (i * 2); 
        numeratorSum += weight * sensorValue;
        denominatorSum += sensorValue;
    }

    if (denominatorSum == 0) {
      return fallbackError;
    }

    // *** OPTIMIZATION: Using fixed-point integer math to avoid slow floats ***
    // We scale everything up by 100 to preserve precision during division.
    long position_fixed = (long)numeratorSum * 100 / denominatorSum; // position is now 0..1400
    long center_fixed = ( (SENSOR_COUNT - 1) * 2 * 100 ) / 2;       // center is now 700
    long e_fixed = position_fixed - center_fixed;                   // error is now -700..+700

    // Scale the fixed-point error back down to the desired range
    int scaledError = (int)((e_fixed * POSITIONAL_WEIGHT_W) / 700); 
    return scaledError;
}


int isOutOfLine(uint8_t sensorReadings)
{
    // When all sensors 0 => out of line
    if (sensorReadings == 0) return 1;
    return 0;
}

void indicateOn() { digitalWrite(LED, HIGH); }
void indicateOff() { digitalWrite(LED, LOW); }

void readSensors()
{
    uint8_t sensorData = getSensorReadingsRaw();
    // compute error with fallback = previous error to maintain direction
    int fallback = (previousError == 0) ? 1 : (previousError > 0 ? 1 : -1);
    error = getCalculatedError(fallback);

    int s1 = (sensorData & (1 << 7)) >> 7;
    int s8 = (sensorData & (1 << 0)) >> 0;
    if (s1 != s8) error_dir = s1 - s8;

    if (sensorData == 0xFF) {
      digitalWrite(LED, HIGH);
    } else {
      digitalWrite(LED, LOW);
    }

    if (sensorData == 0x00) {
      // completely off line
      if (error_dir < 0) error = OUT_OF_LINE_ERROR_VALUE;
      else if (error_dir > 0) error = -OUT_OF_LINE_ERROR_VALUE;
    } 
    else if (sensorData == 0xFF) {
      // checkpoint logic: move straight then check again
      moveStraight(baseMotorSpeed, baseMotorSpeed);
      delay(STOP_CHECK_DELAY);
      uint8_t sensorDataAgain = getSensorReadingsRaw();
      if (sensorDataAgain == 0xFF) {
        shortBrake(100);
        stop();
        delay(3000);
      }
    } 
    else if (IS_ACUTE_LEFT(sensorData)) {
        error = OUT_OF_LINE_ERROR_VALUE;
        error_dir = -1;
    }
    else if (IS_ACUTE_RIGHT(sensorData)) {
        error = -OUT_OF_LINE_ERROR_VALUE;
        error_dir = 1;
    }

    if (MID_6_SENSORS_DETECT_LINE_COLOR(sensorData))
        indicateOn();
    else
        indicateOff();
}

// In calculatePID()
void calculatePID()
{
    // Proportional
    P = error;

    // Integral with anti-windup
    I = (error == 0) ? 0 : I + error;
    I = constrain(I, -500, 500);

    // *** OPTIMIZATION: Using fixed-point integer math for derivative smoothing ***
    long rawD = error - previousError;
    static long smoothD_fixed = 0;
    const int alpha_fixed = 25; // Represents 0.25 (smoothing factor)
    
    // Perform the smoothing calculation with integers
    smoothD_fixed = (alpha_fixed * rawD + (100 - alpha_fixed) * smoothD_fixed) / 100;
    D = (int)smoothD_fixed;
    
    // Combine
    long pid = (long)Kp * P + (long)Ki * I + (long)Kd * D;
    PID_value = (int)constrain(pid / 1, -200, 200);

    previousError = error;
}

// New helper function to search for the line
void searchForLine() {
#if BRAKING_ENABLED == 1
    shortBrake(BRAKE_DURATION_MILLIS);
#endif
    uint8_t sensorReadings = getSensorReadingsRaw();
    while (isOutOfLine(sensorReadings)) {
        if (error_dir < 0) { // Last seen to the right, so turn left (CCW)
            turnCCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
        } else { // Last seen to the left or unknown, so turn right (CW)
            turnCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
        }
        sensorReadings = getSensorReadingsRaw();
    }
#if GAPS_ENABLED == 1
    error_dir = 0;
#endif
}


// Refactored controlMotors function
void controlMotors()
{
  #if DEBUG_ENABLED
  Serial.print("Error: "); Serial.println(error);
  #endif

  if (abs(error) == OUT_OF_LINE_ERROR_VALUE)
  {
    searchForLine();
  }
  else
  {
    // Apply offsets as additive calibration, not as subtraction that counteracts PID
    int leftMotorSpeed = baseMotorSpeed + PID_value + leftMotorOffset;
    int rightMotorSpeed = baseMotorSpeed - PID_value + rightMotorOffset;

    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    moveStraight(leftMotorSpeed, rightMotorSpeed);

    // small non-blocking delay for loop pacing
    if (D != 0) delay(loopDelay);
  }
}

void parseBluetoothCommand() {
  #if BLUETOOTH_TUNING_ENABLED
  static char buffer[32];
  static size_t idx = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buffer[idx] = '\0';
      if (idx > 0) {
        char cmd = tolower(buffer[0]);
        if (cmd == 's') {
          shortBrake(200);
          stop();
          digitalWrite(STBY, LOW);
          Serial.println("STANDBY");
        } else if (cmd == 'r') {
          digitalWrite(STBY, HIGH);
          Serial.println("RUNNING");
        } else if (cmd == 'p' || cmd == 'i' || cmd == 'd' || cmd == 'm') {
          int value = atoi(buffer + 1);
          if (value != 0 || buffer[1] == '0') {
            switch (cmd) {
              case 'p': Kp = value; break;
              case 'i': Ki = value; break;
              case 'd': Kd = value; break;
              case 'm': baseMotorSpeed = constrain(value, 0, 255); break;
            }
            Serial.print(cmd); Serial.print(":"); Serial.println(value);
          }
        }
      }
      idx = 0;
    } else if (idx < sizeof(buffer)-1) {
      buffer[idx++] = c;
    }
  }
  #endif
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);
  pinMode(LED, OUTPUT);

  motorInit();
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // calibrate sensors and detect track type on startup
  calibrateSensorsAndTrackType();
}

void loop() {
  parseBluetoothCommand();
  readSensors();
  calculatePID();
  controlMotors();
}
