#include <Arduino.h>
#include <QTRSensors.h>
#include "motor.h"

#define QTR0 0
#define QTR1 1
#define QTR2 2
#define QTR3 3
#define QTR4 4
#define QTR5 5
#define QTR6 6
#define QTR7 7

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

uint16_t sensorThresholds[SensorCount];

float initialPos = 0;
unsigned int line_position = 0;

float Kp = 0.2;
float Ki = 0;
float Kd = 2;

const uint8_t rightMaxSpeed = 220;
const uint8_t leftMaxSpeed = 220;
const uint8_t rightBaseSpeed = 160;
const uint8_t leftBaseSpeed = 130;

void setup()
{

  Serial.begin(9600);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, SensorCount);
  qtr.setEmitterPin(2);

  // calibration
  for (uint16_t i = 0; i < 200; i++)
  {
    // turnLeft(200);
    qtr.calibrate();
  }

  // set initial line pos. a.k.a. center of the line
  for (uint8_t i = 0; i < 8; i++)
  {
    initialPos = qtr.readLineWhite(sensorValues);
    Serial.print(initialPos);
    Serial.print(" ");
    delay(100);
  }

  // Force motor to turn
  // driveMotor(150, 0); // Set left motor speed to 150 (forward), right motor to 0 (stopped)
  // delay(1000);        // Wait for 1 second
  // driveMotor(0, 150); // Set left motor to 0 (stopped), right motor speed to 150 (forward)
  // delay(1000);        // Wait for 1 second
}
unsigned int threshold = 700; // Default threshold value, can be adjusted as needed

void follow_line() // follow the line
{

  int lastError = 0;

  while (1)
  {

    line_position = qtr.readLineWhite(sensorValues);
    delay(1000);
    Serial.print(line_position);
    Serial.print(" ");

    int error = line_position - initialPos;
    // Serial.print(error);
    // Serial.print(" ");
    int error1 = error - lastError;
    int error2 = (2.0 / 3.0) * error2 + error;
    int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;

    // driveMotor(leftMotorSpeed, rightMotorSpeed);

    // lastError = error;

    // qtr.readLineWhite(sensorValues);
    // if (sensorValues[0] < threshold || sensorValues[7] < threshold)
    // {
    //   driveMotor(leftBaseSpeed, rightBaseSpeed);
    //   return;
    // }
    // if (sensorValues[0] < threshold && sensorValues[1] < threshold && sensorValues[2] < threshold && sensorValues[3] < threshold && sensorValues[4] < threshold && sensorValues[5] < threshold && sensorValues[6] < threshold && sensorValues[7] < threshold)
    // {

    //   driveMotor(leftBaseSpeed, rightBaseSpeed);
    //   return;
    // }
  }
}

void loop()
{

  follow_line();
  delay(500);
}
