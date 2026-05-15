/*
  Arduino BMI270 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"
float angleGyroX;
float angleGyroPrev;
float lastLoopTime = 0;
float ax, ay, az;
float degreesAccY = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  angleGyroPrev = 0;
  lastLoopTime = millis();
}

void loop() {
  float gx, gy, gz;
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    degreesAccY = -1*atan(ay/az)*180/PI;
    //Serial.println(degreesAccY, 4);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    float currentLoopTime = millis();
    float loopDuration = currentLoopTime - lastLoopTime;
    lastLoopTime = currentLoopTime;

    angleGyroX = degreesAccY + gx * (loopDuration/1000);

    Serial.print(degreesAccY, 4);
    Serial.print(",");
    Serial.println(angleGyroX, 4);
  }
  delay(200);

}