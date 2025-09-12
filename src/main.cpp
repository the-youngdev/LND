#include <Arduino.h>
#include <stdint.h>
#include <EEPROM.h>
#include "motor_control.h"
#include "Config.h"

#define CALIBRATION_BUTTON_PIN 2 
#define EEPROM_MAGIC_VALUE   'C' 
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
void saveCalibration() {
  Serial.println("Saving calibration data to EEPROM...");
  // Write a 'magic value' to the first byte of EEPROM to know that data is valid
  EEPROM.write(0, EEPROM_MAGIC_VALUE);
  
  // Use EEPROM.put() to store the TrackType and the thresholds array
  EEPROM.put(1, TrackType);
  EEPROM.put(1 + sizeof(TrackType), thresholds);
  
  Serial.println("Save complete.");
}

bool loadCalibration() {
  Serial.println("Attempting to load calibration from EEPROM...");
  // Check if our 'magic value' exists. If not, the EEPROM is empty or has old data.
  if (EEPROM.read(0) == EEPROM_MAGIC_VALUE) {
    // If it exists, load the data using EEPROM.get()
    EEPROM.get(1, TrackType);
    EEPROM.get(1 + sizeof(TrackType), thresholds);
    
    Serial.println("Load successful. Using stored values.");
    calibrated = true; // Make sure the global flag is set
    return true; // Return true to indicate success
  }
  
  Serial.println("No valid calibration data found.");
  return false; // Return false to indicate failure
}

// In main.cpp

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
    
    long position_fixed = (long)numeratorSum * 100 / denominatorSum; 
    long center_fixed = ( (SENSOR_COUNT - 1) * 2 * 100 ) / 2;       
    long e_fixed = position_fixed - center_fixed;                   

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


// In main.cpp

void readSensors()
{
    uint8_t sensorData = getSensorReadingsRaw();
    
    if (TrackType == WHITE_LINE_BLACK_TRACK) {
        sensorData = ~sensorData;
    }
    
    if (ALL_SENSORS_DETECT_LINE_COLOR(sensorData)) {
        #if DEBUG_ENABLED
        // --- TEMPORARY DEBUG MESSAGE ---
        Serial.println("DEBUG: In Intersection Logic");
        #endif
        // When we hit an intersection, we can determine the current track color
        int averageRawReading = 0;
        for(int i=0; i<SENSOR_COUNT; ++i) averageRawReading += rawAnalog[i];
        averageRawReading /= SENSOR_COUNT;

        if (averageRawReading < 400 && TrackType != WHITE_LINE_BLACK_TRACK) {
            TrackType = WHITE_LINE_BLACK_TRACK;
            Serial.println("RUNTIME SWAP to WHITE_LINE_BLACK_TRACK");
        } 
        else if (averageRawReading > 600 && TrackType != BLACK_LINE_WHITE_TRACK) {
            TrackType = BLACK_LINE_WHITE_TRACK;
            Serial.println("RUNTIME SWAP to BLACK_LINE_WHITE_TRACK");
        }

        moveStraight(baseMotorSpeed, baseMotorSpeed);
        delay(INTERSECTION_STRAIGHT_MS);
        previousError = 0;
        return;
    }
    else if (ALL_SENSORS_OUT_OF_LINE_COLOR(sensorData)) {
        #if DEBUG_ENABLED 
        // --- TEMPORARY DEBUG MESSAGE ---
        Serial.println("DEBUG: In Gap/Off-line Logic");
        #endif 

        moveStraight(baseMotorSpeed, baseMotorSpeed);
        delay(GAP_STRAIGHT_MS);
        
        uint8_t sensorDataAgain = getSensorReadingsRaw();
        if (TrackType == WHITE_LINE_BLACK_TRACK) sensorDataAgain = ~sensorDataAgain;

        if (ALL_SENSORS_OUT_OF_LINE_COLOR(sensorDataAgain)) {
            error = (error_dir < 0) ? OUT_OF_LINE_ERROR_VALUE : -OUT_OF_LINE_ERROR_VALUE;
        }
    }

    int s1 = (sensorData & (1 << 7));
    int s8 = (sensorData & (1 << 0));
    if (s1 && !s8) error_dir = -1;
    if (!s1 && s8) error_dir = 1;

    error = getCalculatedError(previousError); 
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




void controlMotors()
{
  #if DEBUG_ENABLED
  Serial.print("Error: "); Serial.println(error);
  #endif

  // If error is max value, it means we are lost (not in a gap) -> Search for line
  if (abs(error) == OUT_OF_LINE_ERROR_VALUE)
  {
    searchForLine(); // This function already exists and works well for this
  }
  else // Otherwise, use PID to steer
  {
    int leftMotorSpeed = baseMotorSpeed + PID_value;
    int rightMotorSpeed = baseMotorSpeed - PID_value;

    moveStraight(leftMotorSpeed, rightMotorSpeed);
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

// In main.cpp

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  // ... (all other pinModes)
  pinMode(LED, OUTPUT);
  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);

  motorInit();
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  if (digitalRead(CALIBRATION_BUTTON_PIN) == LOW) {
    Serial.println("Manual calibration triggered!");
    indicateOn();
    calibrateSensorsAndTrackType();
    indicateOff();
  } else {
    if (!loadCalibration()) {
      // If loading fails, use hardcoded default values instead of forcing calibration
      Serial.println("Load failed. Using hardcoded default thresholds.");
      int default_thresholds[] = DEFAULT_THRESHOLDS;
      for(int i=0; i<SENSOR_COUNT; ++i) {
          thresholds[i] = default_thresholds[i];
      }
      // Assume a black line on a white track as a safe default
      TrackType = BLACK_LINE_WHITE_TRACK;
      calibrated = true; // Mark as calibrated
    }
  }

  // Debugging print to show final values
  Serial.println("--- Robot is ready with these values: ---");
  Serial.print("TrackType: ");
  Serial.println(TrackType == BLACK_LINE_WHITE_TRACK ? "BLACK_LINE_WHITE_TRACK" : "WHITE_LINE_BLACK_TRACK");
  Serial.print("Thresholds: ");
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(thresholds[i]); Serial.print(i == SENSOR_COUNT-1 ? "\n" : ", ");
  }
  Serial.println("-----------------------------------------");
}

void loop() {
  parseBluetoothCommand();
  readSensors();
  calculatePID();
  controlMotors();
}
