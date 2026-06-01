#include "Arduino_BMI270_BMM150.h"
float angleGyroX;
float angleGyroPrev;
float angleComp;
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
  angleComp = 0;
  lastLoopTime = millis();
}

void loop() {
  float gx, gy, gz;
  float ax, ay, az;
  int k = 0.1;

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

    angleGyroX = angleComp + (gx-0.3) * (loopDuration/1000);
    angleComp = (k*(angleGyroX)) + ((1-k)*degreesAccY);
    
    Serial.print(degreesAccY, 4);
    Serial.print(",");
    Serial.print(angleGyroX, 4);
    Serial.print(",");
    Serial.println(angleComp, 4);
  }
  delay(300);

}