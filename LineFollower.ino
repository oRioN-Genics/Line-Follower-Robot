#include <math.h>

int left_motors_pin1 = 3; // AIN1
int left_motors_pin2 = 4; // AIN2
int left_motors_en = 9;  // PWMA
int right_motors_pin1 = 6; // BIN1
int right_motors_pin2 = 7;  // BIN2
int right_motors_en = 10; // PWMB

int calibrationButtonPin = 2; 
int ledPin = 13;
volatile bool calibrateNow = false; // Flag for calibration

// IR Sensor Pins
const int SENSOR_COUNT = 8;
int sensors[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, A6, A7}; 

// calibration values
int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT]; 
bool sensorsCalibrated = false;
int calibrationFailCount = 0;
const int MAX_CALIBRATION_FAILS = 3;

//=======================================================================
//=======================================================================

// PID 
float kp = 1.5;  
float ki = 0.1;  
float kd = 0.5;   
float error = 0, lastError = 0, integral = 0, derivative = 0;
float correction = 0;

int baseSpeed = 150;
int maxSpeed = 255;
int minSpeed = 50;

volatile unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 50;  
const unsigned long calibrationTime = 5000; // 5 seconds for calibration

const int SHARP_TURN_THRESHOLD = 2; 
const int SHARP_TURN_SPEED_MULTIPLIER = 2;

// error States
enum RobotState {
  NORMAL_OPERATION,
  NO_LINE_DETECTED,
  CALIBRATION_ERROR
};

RobotState currentState = NORMAL_OPERATION;

bool isBlackBackground = true; // line type

//=======================================================================
//=======================================================================

void setup() {
  Serial.begin(9600);

  pinMode(calibrationButtonPin, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(calibrationButtonPin), triggerCalibration, FALLING);

  for (int i=0; i < SENSOR_COUNT; i++) {
    pinMode(sensors[i], INPUT);
  }

  pinMode(left_motors_pin1, OUTPUT);
  pinMode(left_motors_pin2, OUTPUT);
  pinMode(right_motors_pin1, OUTPUT);
  pinMode(right_motors_pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  calibrate();
}

//---------------------------------------------------------

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


void calibrate() {
  digitalWrite(ledPin, HIGH);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 1023; 
    sensorMax[i] = 0;    
  }

  unsigned long startTime = millis();
  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int value = smoothSensorReading(sensors[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
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
    }
  } else {
    calibrationFailCount = 0;
    currentState = NORMAL_OPERATION;
  }

  // reset PID 
  integral = 0.1;
  lastError = 0;

  digitalWrite(ledPin, LOW);
}


int getSensorReading(int sensorIndex) {
  int value = analogRead(sensors[sensorIndex]);
  int threshold = (sensorMin[sensorIndex] + sensorMax[sensorIndex]) / 2;
  return value < threshold ? 1 : 0; 
}


// based on sensor readings, cal.. position
int calculatePosition() {
  int weights[SENSOR_COUNT] = {-3, -2, -1, 0, 1, 2, 3, 4};
  int sum = 0, total = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int value = getSensorReading(i); 
    sum += value * weights[i];
    total += value;
  }

  if (total > 0) {
    return sum / total;
  } else {
    currentState = NO_LINE_DETECTED;
    return 0;
  }
}


void setMotorSpeeds(int leftSpeed,int rightSpeed) {
  // handle sharp turns
  if (abs(error) >= SHARP_TURN_THRESHOLD) {
    leftSpeed *= (error > 0) ? SHARP_TURN_SPEED_MULTIPLIER : 1;
    rightSpeed *= (error < 0) ? SHARP_TURN_SPEED_MULTIPLIER : 1;
  }

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


void detectBackgroundType() {
  static int blackDominantCount = 0;
  static int whiteDominantCount = 0; 
  const int detectionThreshold = 20; 

  int blackCount = 0, whiteCount = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int value = analogRead(sensors[i]);
    int threshold = (sensorMin[i] + sensorMax[i]) / 2;

    if (value < threshold) blackCount++;
    else whiteCount++;
  }

  if (blackCount > whiteCount) {
    blackDominantCount++;
    whiteDominantCount = 0;
  } else {
    whiteDominantCount++;
    blackDominantCount = 0;
  }

  if (blackDominantCount >= detectionThreshold) {
    isBlackBackground = true;
  } else if (whiteDominantCount >= detectionThreshold) {
    isBlackBackground = false;
  }
}

bool detectIntersection() {
  int activeSensors = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    activeSensors += getSensorReading(i);
  }

  return activeSensors == SENSOR_COUNT;
}

//---------------------------------------------------------

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
    while (leftSpeed > minSpeed && rightSpeed > minSpeed) {
      leftSpeed--;
      rightSpeed--;
      setMotorSpeed(leftSpeed, rightSpeed)
      delay(10);
    }
    delay(100); // Pause at intersection
    return;
  }

  int position = calculatePosition();

  switch (currentState) {
    case NO_LINE_DETECTED:
      setMotorSpeeds(0, 0);
      return;
    
    case CALIBRATION_ERROR:
      setMotorSpeeds(0, 0);
      return;
    
    case NORMAL_OPERATION:
      break;
  }

  // no line detected
  if (position == 0) { 
    setMotorSpeeds(0, 0); 
    return; 
  }

  error = position;
  integral += error;
  derivative = error - lastError;
  correction = (kp * error) + (ki * integral) + (kd * derivative);

  integral = constrain(integral, -100, 100); 
  correction = constrain(correction, -baseSpeed, baseSpeed);

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotorSpeeds(leftSpeed, rightSpeed);

  lastError = error;
}
