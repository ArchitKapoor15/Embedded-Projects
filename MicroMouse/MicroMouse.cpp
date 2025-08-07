/*
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Initializing IMU...");

    // Initialize I2C communication
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1);
    }
    Serial.println("MPU6050 found!");

    // Configure the sensor
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
}

void loop() {
    // Get new sensor events
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print accelerometer data
    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
    Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
    Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

    // Print gyroscope data
    Serial.print("Gyro X: "); Serial.print(g.gyro.x); Serial.print(" rad/s, ");
    Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s, ");
    Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");

    // Print temperature
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" C");

    Serial.println("-----------------------------------");
    delay(500); // Delay for readability
}
*/

/*
#include <Arduino.h>
#include "BluetoothSerial.h"

// Define sensor pins
const int trigPins[4] = {19, 4, 2, 5}; // Trig pins for each sensor
const int echoPins[4] = {18, 16, 15, 17}; // Echo pins for each sensor

// Function to measure distance
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2; // Convert to cm

    if (distance >= 400 || distance <= 2) {
        return -1; // Out of range
    }
    return distance;
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_Ultrasonic"); // Bluetooth device name

    // Initialize sensor pins
    for (int i = 0; i < 4; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
}

void loop() {
    Serial.println("Distance Measurements (cm):");
    SerialBT.println("Distance Measurements (cm):");

    for (int i = 0; i < 4; i++) {
        float distance = getDistance(trigPins[i], echoPins[i]);

        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        SerialBT.print("Sensor ");
        SerialBT.print(i + 1);
        SerialBT.print(": ");

        if (distance == -1) {
            Serial.println("Out of range");
            SerialBT.println("Out of range");
        } else {
            Serial.print(distance);
            Serial.println(" cm");
            SerialBT.print(distance);
            SerialBT.println(" cm");
        }
    }

    Serial.println("---------------------------");
    SerialBT.println("---------------------------");
    delay(500); // Adjust delay as needed
}
*/

#include <Arduino.h>
// For IMU
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define LOOP_TIME 10

// {inputPin1, inputPin2, pwmPin}
uint8_t motorLeft[] = {26, 25, 33};
uint8_t motorRight[] = {14, 12, 13};

void setMotor(uint8_t motor[]){
    pinMode(motor[0], OUTPUT);
    pinMode(motor[1], OUTPUT);
    pinMode(motor[2], OUTPUT);
}
void ctrlMotor(uint8_t motor[], uint8_t dir, uint8_t pwm){
    digitalWrite(motor[0], !dir);
    digitalWrite(motor[1], dir);
    analogWrite(motor[2], pwm);
}

// PID Constants
float Kp = 3.5;  // Proportional gain
float Ki = 0.02; // Integral gain
float Kd = 0.1;  // Derivative gain
float previousError = 0, integral = 0;

// PID Controller
float computePID(float error) {
    integral += error * (LOOP_TIME / 1000.0);
    float derivative = (error - previousError) / (LOOP_TIME / 1000.0);
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    return constrain(abs(output), 60, 255);
}

// 0 for Right 1 for Left
void turnAngle(float targetAngle, uint8_t dir) {
    int16_t gx, gy, gz;
    float angleZ = 0;
    unsigned long prevTime = millis();

    Serial.print("Turning ");
    Serial.print(targetAngle);
    Serial.println(" degrees...");

    while (abs(angleZ) < targetAngle) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float gyroZ = g.gyro.z * 57.2958;  // Convert rad/s to deg/s

        unsigned long currentTime = millis();
        float dt = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;

        angleZ += gyroZ * dt;
        float error = targetAngle - abs(angleZ);
        float pidOutput = computePID(error);

        ctrlMotor(motorLeft, dir, pidOutput);
        ctrlMotor(motorRight, !dir, pidOutput);
        delay(LOOP_TIME);
    }

    // Stop motors
    ctrlMotor(motorLeft, 0, 0);
    ctrlMotor(motorRight, 0, 0);
    Serial.println("Turn completed!");
}
void linear(uint8_t dir, uint8_t pwm){
    ctrlMotor(motorLeft, dir, pwm);
    ctrlMotor(motorRight, dir, pwm);
}

// Ultrasonic Sensor Pins
const int trigPins[4] = {19, 5, 4, 2};   // Front, Left, Right, Back
const int echoPins[4] = {18, 17, 16, 15}; // Front, Left, Right, Back

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return (duration * 0.0343) / 2;  // Convert to cm
}

void setup() {
    // For motors
    setMotor(motorLeft);
    setMotor(motorRight);
    
    Serial.begin(115200);

    // For ultraSonic
    for (int i = 0; i < 4; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    // For IMU
    while (!Serial);
    Serial.println("Initializing IMU...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1);
    }
    Serial.println("MPU6050 found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);

    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH); // Enable the motor driver

    delay(10000);
}

void loop() {
    float frontDist = getDistance(trigPins[0], echoPins[0]);
    float leftDist = getDistance(trigPins[1], echoPins[1]);
    float rightDist = getDistance(trigPins[2], echoPins[2]);

    Serial.print("Front Distance: ");
    Serial.println(frontDist);
    Serial.print("Left Distance: ");
    Serial.println(leftDist);
    Serial.print("Right Distance: ");
    Serial.println(rightDist);

    if(leftDist > 10){
        turnAngle(90, 1);
    }
    else if(rightDist > 10){
        turnAngle(90, 0);
    }
    else if(frontDist < 10){
        turnAngle(180, 0);
    }
    else{
        linear(0, 255);
    }

    delay(1000);
}