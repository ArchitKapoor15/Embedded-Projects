#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// {inputPin1, inputPin2, pwmPin}
uint8_t motorLeft[] = {5,18,19};
uint8_t motorRight[] = {21,22,23};

void setMotor(uint8_t motor[]){
  pinMode(motor[0], OUTPUT);
  pinMode(motor[1], OUTPUT);
  pinMode(motor[2], OUTPUT);
}

void ctrlMotor(uint8_t motor[], uint8_t dir, uint8_t pwm){
  if(dir == 0){
    digitalWrite(motor[0], HIGH);
    digitalWrite(motor[1], LOW);

    analogWrite(motor[2], pwm);
  }
  else if(dir == 1){
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], HIGH);

    analogWrite(motor[2], pwm);
  }
}

void setup() {
  setMotor(motorLeft);
  setMotor(motorRight);
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT");

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // Enable the motor driver
}

void loop() {
  if(SerialBT.available()){
    char cmd = SerialBT.read();
    Serial.println(cmd);
    SerialBT.println(cmd);
    switch(cmd){
      case 'w':
        ctrlMotor(motorLeft, 0, 255);
        ctrlMotor(motorRight, 0, 255);
        break;
      case 's':
        ctrlMotor(motorLeft, 1, 255);
        ctrlMotor(motorRight, 1, 255);
        break;
      case 'a':
        ctrlMotor(motorLeft, 1, 255);
        ctrlMotor(motorRight, 0, 255);
        break;
      case 'd':
        ctrlMotor(motorLeft, 0, 255);
        ctrlMotor(motorRight, 1, 255);
        break;
      case 't':
        ctrlMotor(motorLeft, 0, 0);
        ctrlMotor(motorRight, 0, 0);
        break;
    }
  }
}
