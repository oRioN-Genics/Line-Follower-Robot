const SENSOR_COUNT = 5;
int sensors[SENSOR_COUNT] = {A0, A1, A2, A3, A4}; 

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensors[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < SENSOR_COUNT; i++) {   
    int sensorInput = analogRead(sensors[i]);  // (0 – 1023)
    // int sensorInput = digitalRead(sensors[i]);  // output binary state (0 for dark, 1 for light)
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorInput); // should be lower if the sensor detects dark 
    Serial.print("   ");
  }
  Serial.println();
  delay(100); // just to make it easier to observe
}
