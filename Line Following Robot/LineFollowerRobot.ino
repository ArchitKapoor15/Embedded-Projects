#define SENSOR_COUNT 8
#define THRESHOLD 4000 // Adjust as per calibration

// Check GPIO Pins
const uint8_t sensorPins[] = {27, 26, 25, 33, 32, 35, 34, 39};
const int8_t weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
uint8_t threshold[8] = {0};
int baseSpeed = 100; // Base Speed
int leftSpeed, rightSpeed;

struct Motor {
  uint8_t dir1;
  uint8_t dir2;
  uint8_t pwm;
};

const Motor motorLeft = {4, 2, 15};
const Motor motorRight = {17, 5, 18};

unsigned long blackStartTime = 0;
const unsigned long endTrackThreshold = 1000;

float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.5;

float error = 0, previousError = 0;
float P, D, I;
bool hasStarted = false;

int calculateError() {
  int sum = 0;
  for (int i = 0; i < 8; i++) {
    if (threshold[i] == 1) { // Black detected
        sum += weight[i];
    }
  }
  if(sum == 2 || sum == -2){
    return sum*3;
  }else{
    return sum*10;
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    analogWrite(motorLeft.pwm, abs(leftSpeed));
    analogWrite(motorRight.pwm, abs(rightSpeed));
    
    digitalWrite(motorLeft.dir1, leftSpeed > 0);
    digitalWrite(motorLeft.dir2, leftSpeed <= 0);
    digitalWrite(motorRight.dir1, rightSpeed > 0);
    digitalWrite(motorRight.dir2, rightSpeed <= 0);
}

void detectTrackConditions() {
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (threshold[i] == 1) {
      blackCount++;
    }
  }
  if (!hasStarted) {
    if (blackCount < 8 && blackCount > 0)
      hasStarted = true;
    else
      return;
  }
  if (blackCount == 0) {
    Serial.println("Dead end detected");
    bool lineFound = false;
    while (!lineFound) {
      for (int i = 0; i < 8; i++) {
        threshold[i] = analogRead(sensorPins[i]) > 4000 ? 1 : 0;
        Serial.print(threshold[i]);
        Serial.print("  ");
      }
      Serial.println();
      for (int i = 0; i < 8; i++) {
        if (threshold[i] == 1) {
          lineFound = true;
          break;
        }
      }
      setMotorSpeed(-100, 100); // Keep turning left.
      delay(250);
    }
    return;
  }
  if (blackCount == 8) {
    if (blackStartTime == 0){
     setMotorSpeed(0, 0);
      while (1); // Halt further processing.
    }else {
      blackStartTime = 0;
    }
  }     blackStartTime = millis();
    }
    else if (millis() - blackStartTime > endTrackThreshold) {
      Serial.println("End of track detected");
      digitalWrite(22, HIGH); 
  
}

void setup() {
  Serial.begin(115200);

  pinMode(motorLeft.dir1, OUTPUT);
  pinMode(motorLeft.dir2, OUTPUT);
  pinMode(motorLeft.pwm, OUTPUT);

  pinMode(motorRight.dir1, OUTPUT);
  pinMode(motorRight.dir2, OUTPUT);
  pinMode(motorRight.pwm, OUTPUT);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH); // Enable the motor driver
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH); // Enable IR sensor
}

void loop() {
    int sensorValues[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) {
      threshold[i] = analogRead(sensorPins[i]) > THRESHOLD ? 1 : 0;
      Serial.print(threshold[i]);
      Serial.print("\t");
    }
    Serial.println();

    detectTrackConditions();

    error = calculateError();
    P = error;
    I += error;
    D = error - previousError;

    float correction = Kp * P + Ki * I + Kd * D;

    leftSpeed = baseSpeed + correction;
    rightSpeed = baseSpeed - correction;
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    Serial.print("Error : ");
    Serial.println(error);
    Serial.print("Correction : ");
    Serial.println(correction);
    // Set motor speeds
    setMotorSpeed(leftSpeed, rightSpeed);
    Serial.print("LeftSpeed = ");
    Serial.println(leftSpeed);
    Serial.print("RightSpeed = ");
    Serial.println(rightSpeed);
    
    previousError = error;
    delay(25); // Small delay to stabilize readings
}