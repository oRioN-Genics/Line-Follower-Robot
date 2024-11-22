#include <math.h>

int left_motors_pin1 = 3; // AIN1
int left_motors_pin2 = 4; // AIN2
int left_motors_en = 9;  // PWMA
int right_motors_pin1 = 6; // BIN1
int right_motors_pin2 = 7;  // BIN2
int right_motors_en = 10; // PWMB
// IR Sensor Pins
const int SENSOR_COUNT = 5;
int sensors[SENSOR_COUNT] = {A0, A1, A2, A3, A4}; 

int calibrationButtonPin = 2; 
int ledPin = 13;
volatile bool calibrateNow = false; // Flag for calibration

// calibration values
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT]; 
bool sensorsCalibrated = false;
int calibrationFailCount = 0;
const int MAX_CALIBRATION_FAILS = 2;

//=======================================================================
//=======================================================================

// PID 
float kp = 1.5;  
float ki = 0.1;  
float kd = 0.5;   
float error = 0, lastError = 0, integral = 0, derivative = 0;
float correction = 0;

int L_Speed = 0;
int R_Speed = 0;
const unsigned long searchDelay = 1000;

int baseSpeed = 150;
int maxSpeed = 255;
int minSpeed = 50;

volatile unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 50;  
const unsigned long calibrationTime = 5000; // 5 seconds for calibration

const int SHARP_TURN_THRESHOLD = 2000; 
const int SHARP_TURN_SPEED_MULTIPLIER = 1.5;

// error States
enum RobotState {
  CALIBRATING,
  NORMAL_OPERATION,
  NO_LINE_DETECTED,
  CALIBRATION_ERROR,
  RECOVERING
};

RobotState currentState = CALIBRATING;

bool isBlackBackground = true; // line type

//=======================================================================
//=======================================================================

void triggerCalibration() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    calibrateNow = true;
    lastDebounceTime = currentTime;
  }
}


int smoothSensorReading(int pin) {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100); 
  }
  return sum / 10;
}


bool validateCalibration() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Check for meaningful calibration range
    if (sensorMax[i] - sensorMin[i] < 50) {
      Serial.print("Calibration failed for sensor ");
      Serial.println(i);
      return false;
    }
  }
  return true;
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed); 
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  if (leftSpeed > 0) {
    analogWrite(left_motors_en, leftSpeed);
    digitalWrite(left_motors_pin1, HIGH);
    digitalWrite(left_motors_pin2, LOW);
  } else {
    analogWrite(left_motors_en, -leftSpeed);
    digitalWrite(left_motors_pin1, LOW);
    digitalWrite(left_motors_pin2, HIGH);
  }

  if (rightSpeed > 0) {
    analogWrite(right_motors_en, rightSpeed);
    digitalWrite(right_motors_pin1, HIGH);
    digitalWrite(right_motors_pin2, LOW);
  } else {
    analogWrite(right_motors_en, -rightSpeed);
    digitalWrite(right_motors_pin1, LOW);
    digitalWrite(right_motors_pin2, HIGH);
  }
}


void calibrate() {
  digitalWrite(ledPin, HIGH);
  currentState = CALIBRATING;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 1023; 
    sensorMax[i] = 0;    
  }

  unsigned long startTime = millis();
  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int value = smoothSensorReading(sensors[i]);
      sensorMin[i] = min(sensorMin[i], value);
      sensorMax[i] = max(sensorMax[i], value);
    }
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
        int range = sensorMax[i] - sensorMin[i];
        sensorMin[i] = max(sensorMin[i] - (range * 0.1), 0); // Adjust min by 10%
        sensorMax[i] = min(sensorMax[i] + (range * 0.1), 1023); // Adjust max by 10%
    }

  sensorsCalibrated = validateCalibration();

   if (!sensorsCalibrated) {
    calibrationFailCount++;
    currentState = CALIBRATION_ERROR;
    
    if (calibrationFailCount >= MAX_CALIBRATION_FAILS) {
      // critical failure - stop robot
      setMotorSpeeds(0, 0);
      while(true) {
        // blink error pattern
        digitalWrite(ledPin, HIGH);
        delay(200);
        digitalWrite(ledPin, LOW);
        delay(200);
      }

      if (digitalRead(calibrationButtonPin) == LOW) {
                calibrationFailCount = 0; 
                calibrateNow = true;    
      }
    } else {
      calibrationFailCount = 0;
      currentState = NORMAL_OPERATION;

      // reset PID 
      integral = 0;
      lastError = 0;
    }

    digitalWrite(ledPin, LOW);
   }
}


int getSensorReading(int sensorIndex) {
  int value = analogRead(sensors[sensorIndex]);
  int threshold = (sensorMin[sensorIndex] + sensorMax[sensorIndex]) / 2;
  return value < threshold ? 1 : 0; 
}


int normalizeSensorValue(int rawValue, int index) {
    if (rawValue < sensorMin[index]) return 0;
    if (rawValue > sensorMax[index]) return 1000;
    return ((rawValue - sensorMin[index]) * 1000) / (sensorMax[index] - sensorMin[index]);
}


int calculatePosition() {
    int weights[SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000}; 
    long weightedSum = 0, totalWeight = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int normalizedValue = normalizeSensorValue(analogRead(sensors[i]), i); 
        weightedSum += (long)normalizedValue * weights[i];
        totalWeight += normalizedValue;
    }

    if (totalWeight > 0) {
        return weightedSum / totalWeight;
    } else {
        currentState = NO_LINE_DETECTED;
        return -9999; 
    }
}


void detectBackgroundType() {
  const int sampleSize = 20;
  static int samples[sampleSize] = {0};
  static int sampleIndex = 0;
  
  int blackCount = 0, whiteCount = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int value = analogRead(sensors[i]);
    int threshold = (sensorMin[i] + sensorMax[i]) / 2;

    if (value < threshold) blackCount++;
    else whiteCount++;
  }

  // store the result of this sample (1 for black dominant, 0 for white dominant)
  samples[sampleIndex] = (blackCount > whiteCount) ? 1 : 0;
  sampleIndex = (sampleIndex + 1) % sampleSize;

  int blackDominantCount = 0;
  for (int i = 0; i < sampleSize; i++) {
    blackDominantCount += samples[i];
  }

  // determine background type based on majority of samples
  if (blackDominantCount > sampleSize / 2) {
    isBlackBackground = true;
  } else {
    isBlackBackground = false;
  }
}


bool detectIntersection() {
  int activeSensors = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int normalizedValue = normalizeSensorValue(analogRead(sensors[i]), i);
    if (normalizedValue > 800) {
      activeSensors++;
    }
  }
  return activeSensors >= SENSOR_COUNT - 1;
}


void handleIntersection() {
  setMotorSpeeds(baseSpeed / 2, baseSpeed / 2);
  delay(100); 
}


void dashOrLost() {
    static unsigned long dashStartTime = 0;
    static bool isDashing = true;

    if (isDashing) {
        if (millis() - dashStartTime < searchDelay) {
            setMotorSpeeds(L_Speed, R_Speed); 
            if (calculatePosition() != 0) {
                currentState = NORMAL_OPERATION;
                return;
            }
        } else {
            isDashing = false; // Switch to search mode
        }
    } else {
        searchForLine(); // fallback to search
        if (millis() - dashStartTime > 10000) { 
            currentState = RECOVERING;
        }
    }
}


void searchForLine() {
  static int searchDirection = 1;
  static unsigned long searchStartTime = millis();
  static int searchSpeed = baseSpeed / 2;
  
  if (millis() - searchStartTime > 5000) { 
    setMotorSpeeds(0, 0);
    return;
  }
  
  setMotorSpeeds(-searchSpeed * searchDirection, searchSpeed * searchDirection);
  delay(50);
  
  if (calculatePosition() != 0) { // line found
    currentState = NORMAL_OPERATION;
    searchDirection *= -1; // change direction for next search
    searchSpeed = baseSpeed / 2;
  } else {
        searchSpeed = min(searchSpeed + 5, maxSpeed / 2); 
    }
}


void recoverFromLineLoss() {
    static unsigned long recoveryStartTime = millis();
    
    setMotorSpeeds(-baseSpeed / 2, -baseSpeed / 2); // move backward
    delay(50);

    if (millis() - recoveryStartTime > 10000) { 
        setMotorSpeeds(0, 0);
        digitalWrite(ledPin, HIGH); // failure
        while (true) {
            digitalWrite(ledPin, HIGH);
            delay(200);
            digitalWrite(ledPin, LOW);
            delay(200);
        }
    }
}


void followLine(int position) {
  error = position;
  integral += error;
  derivative = error - lastError;
  correction = (kp * error) + (ki * integral) + (kd * derivative);

  integral = constrain(integral, -100, 100); 
  correction = constrain(correction, -baseSpeed, baseSpeed);

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Handle sharp turns
  if (abs(error) >= SHARP_TURN_THRESHOLD) {
    leftSpeed *= (error > 0) ? SHARP_TURN_SPEED_MULTIPLIER : 1;
    rightSpeed *= (error < 0) ? SHARP_TURN_SPEED_MULTIPLIER : 1;
  }

  setMotorSpeeds(leftSpeed, rightSpeed);
  L_Speed = leftSpeed;
  R_Speed = rightSpeed;
  lastError = error;
}

//=======================================================================
//=======================================================================

void setup() {
  Serial.begin(9600);

  pinMode(calibrationButtonPin, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(calibrationButtonPin), triggerCalibration, FALLING);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensors[i], INPUT);
  }

  pinMode(left_motors_pin1, OUTPUT);
  pinMode(left_motors_pin2, OUTPUT);
  pinMode(right_motors_pin1, OUTPUT);
  pinMode(right_motors_pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  calibrate();
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------

void loop() {
  if (calibrateNow) {
    calibrate();       
    calibrateNow = false; 
  }

  if (!sensorsCalibrated) {
    setMotorSpeeds(0, 0);
    return;
  }

  detectBackgroundType();

  if (detectIntersection()) {
    handleIntersection();
    return;
  } 

  int position = calculatePosition();

  switch (currentState) {
    case NO_LINE_DETECTED:
      dashOrLost();
      break;
    
    case CALIBRATION_ERROR:
      setMotorSpeeds(0, 0);
      break;
    
    case NORMAL_OPERATION:
      followLine(position);
      break;

    case RECOVERING:
      recoverFromLineLoss();
      break;

    case CALIBRATING:
      break;
  }
}
