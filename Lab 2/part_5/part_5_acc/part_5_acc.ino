#include "Arduino_BMI270_BMM150.h"

// ── Pin Definitions (CHANGED FROM PART 3) ─────────────────────────
const int ledPin    = LED_BUILTIN;

const int motor1in1 = 5;   // moved off interrupt pins
const int motor1in2 = 6;
const int motor2in1 = 7;
const int motor2in2 = 8;

const int enc1A = 2;   // INT0 – must be 2 or 3 on Uno
const int enc1B = 9;
const int enc2A = 3;   // INT1 – must be 2 or 3 on Uno
const int enc2B = 10;

// ── PWM / Deadband ────────────────────────────────────────────────
int pwm1 = 0;
int pwm2 = 0;
int db1f = 36, db1r = 40;
int db2f = 35, db2r = 35;

// ── Encoder counts (volatile = modified in ISR) ───────────────────
volatile long encCount1 = 0;
volatile long encCount2 = 0;

// -- Acc and gyro vars ------------------------------------------
float angleGyroX;
float angleGyroPrev;
float angleComp = 0;
float lastLoopTime = 0;
float ax, ay, az;
float degreesAccY = 0;

// ── ISRs ──────────────────────────────────────────────────────────
void ISR_enc1() {
  int a = digitalRead(enc1A);
  int b = digitalRead(enc1B);
  encCount1 += (a == b) ? -2 : 2;
}

void ISR_enc2() {
  int a = digitalRead(enc2A);
  int b = digitalRead(enc2B);
  encCount2 += (a == b) ? 2 : -2;
}

// ── RPM Calculation ───────────────────────────────────────────────
// Separate state for each motor
float calcRPM(volatile long &encCount, long &prevCount, unsigned long &prevTime) {
  unsigned long now = millis();
  unsigned long dt = now - prevTime;
  if (dt == 0) return 0;

  noInterrupts();
  long current = encCount;
  interrupts();

  long delta = current - prevCount;
  prevCount = current;
  prevTime = now;

  float revs = (float)delta / 1920.0; //MAKE INTO CONST!
  return (revs / (float)dt) * 60000.0f;   // RPM
}

// ── H-Bridge Control ──────────────────────────────────────────────
void setMotor1(int pwm) {
  pwm = constrain(pwm, -100, 100);
  if (pwm > 0) {
    int out = (int)(((255 - db1f) * pwm / 100.0) + db1f);
    analogWrite(motor1in2, out);
    analogWrite(motor1in1, 0);
  } else if (pwm < 0) {
    int out = (int)(((255 - db1r) * (-pwm) / 100.0) + db1r);
    analogWrite(motor1in2, 0);
    analogWrite(motor1in1, out);
  } else {
    analogWrite(motor1in1, 255);
    analogWrite(motor1in2, 255);
  }
  //Serial.print(" M1="); Serial.print(pwm);
}

void setMotor2(int pwm) {
  pwm = constrain(pwm, -100, 100);
  if (pwm > 0) {
    int out = (int)(((255 - db2f) * pwm / 100.0) + db2f);
    analogWrite(motor2in1, out);
    analogWrite(motor2in2, 0);
  } else if (pwm < 0) {
    int out = (int)(((255 - db2r) * (-pwm) / 100.0) + db2r);
    analogWrite(motor2in1, 0);
    analogWrite(motor2in2, out);
  } else {
    analogWrite(motor2in1, 255);
    analogWrite(motor2in2, 255);
  }
  //Serial.print("  M2="); Serial.print(pwm);
}

// float angle() {
//   float gx, gy, gz;
//   float ax, ay, az;
//   float k = 0.91;
//   if (IMU.accelerationAvailable()) {
//     IMU.readAcceleration(ax, ay, az);
//     degreesAccY = -1*atan(ay/az)*180/PI;
//     //Serial.println(degreesAccY, 4);
//   }
//   if (IMU.gyroscopeAvailable()) {
//     IMU.readGyroscope(gx, gy, gz);
//     float currentLoopTime = millis();
//     float loopDuration = currentLoopTime - lastLoopTime;
//     lastLoopTime = currentLoopTime;
//     angleGyroX = degreesAccY + gx * (loopDuration/1000);
//     angleComp = (k*(angleGyroX)) + ((1-k)*degreesAccY);
//     return angleComp;
//     // Serial.print(degreesAccY, 4);
//     // Serial.print(",");
//     // Serial.print(angleGyroX, 4);
//     // Serial.print(",");
//     // Serial.println(angleComp, 4);
//   }
//  // delay(300);
// }
float angle() {
  float gx, gy, gz;
  float ax, ay, az;
  float k = 0.91;

  // Only compute if BOTH are available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    degreesAccY = -1 * atan(ay / az) * 180 / PI;

    float currentLoopTime = millis();
    float loopDuration = currentLoopTime - lastLoopTime;
    lastLoopTime = currentLoopTime;

    angleGyroX = degreesAccY + gx * (loopDuration / 1000.0);
    angleComp = (k * angleGyroX) + ((1 - k) * degreesAccY);
    //Serial.print(" gyro angle="); Serial.print(angleGyroX, 4);
    return angleComp;
  }

  // Safe fallback — return last known good angle instead of garbage
  return angleComp;
}


// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);  // INPUT_PULLUP avoids floating pin noise
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE); //??
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);

  // setMotor1(pwm1);
  // setMotor2(pwm2);

    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  angleGyroPrev = 0;
  lastLoopTime = millis();

}

// ── Loop ──────────────────────────────────────────────────────────
// Per-motor state for calcRPM
long  prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime1 = 0, prevTime2 = 0;

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  // if (now - lastPrint >= 100) {   // print every 100 ms
  //   // float rpm1 = calcRPM(encCount1, prevCount1, prevTime1);
  //   // float rpm2 = calcRPM(encCount2, prevCount2, prevTime2);

  //   // Serial.print(" RPM1 = "); Serial.print(rpm1);
  //   // Serial.print("  RPM2 = "); Serial.println(rpm2);
  //   // Serial.print("  enc1 = "); Serial.print(encCount1);
  //   // Serial.print("  enc2 = "); Serial.println(encCount2);

  //   lastPrint = now;
  // }
    float rpm1 = calcRPM(encCount1, prevCount1, prevTime1);
    float rpm2 = calcRPM(encCount2, prevCount2, prevTime2);

    Serial.print(" RPM1 = "); Serial.print(rpm1);
    Serial.print("  RPM2 = "); Serial.print(rpm2);

  float tilt_angle = angle();

  if (tilt_angle <= -0.5) {
    pwm1 = 100.0 * (float)tilt_angle / 90.0;
    pwm2 = 100.0 * (float)tilt_angle / 90.0;
  }

  else if (tilt_angle > 0.5) {
    pwm1 = 100.0 * (float)tilt_angle / 90.0;
    pwm2 = 100.0 * (float)tilt_angle / 90.0;
  }

  else if (tilt_angle > -0.5 && tilt_angle < 0.5) { //in here, wheels r goin crazy idk why so between -2 and 2 degrees, deadband messing with it
    pwm1 = 0;
    pwm2 = 0;
  }

  setMotor1(pwm1);
  setMotor2(pwm2);

  Serial.print("  angle = "); Serial.print(tilt_angle);
  Serial.print("  pwm1 = "); Serial.print(pwm1);
  Serial.print("  pwm2 = "); Serial.print(pwm2);
  Serial.print("  encoder 1 = "); Serial.print(encCount1);
  Serial.print("  encoder 2 = "); Serial.println(encCount2);


}