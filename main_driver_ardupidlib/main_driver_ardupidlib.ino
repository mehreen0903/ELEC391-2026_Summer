#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>
#include <string.h>
#define BUFFER_SIZE 20

//----------ANGLE PID---------------
double angleInput;
double angleOutput;
double angleSetpoint;

// tuning gains
double Kp = 3;
double Ki = 0.08;
double Kd = 0.5;

// PID object
PID anglePID(&angleInput,&angleOutput,&angleSetpoint,Kp, Ki, Kd, DIRECT);

// ---------------- DRIVE PID ----------------
double driveInput;
double driveOutput;
double driveSetpoint;

double Kp_drive = 0.0;
double Ki_drive = 0.0;
double Kd_drive = 0.0;

PID drivePID(&driveInput, &driveOutput, &driveSetpoint,Kp_drive, Ki_drive, Kd_drive, DIRECT);

// Define a custom BLE service and characteristic --------------------
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


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
int db1f = 60, db1r = 60; //old f is 36
int db2f = 60, db2r = 60; //old f is 35

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


float avePWM;
float difPWM;
float aveRPM;
float difRPM; 

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
}

float angle() {
  float gx, gy, gz;
  float ax, ay, az;
  float k = 0.99;

  // Only compute if BOTH are available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    degreesAccY = -1 * atan(ay / az) * 180 / PI + 0.25;

    float currentLoopTime = millis();
    float loopDuration = currentLoopTime - lastLoopTime;
    lastLoopTime = currentLoopTime;

    angleGyroX = angleComp + (gx-0.3) * (loopDuration / 1000.0);
    angleComp = (k * angleGyroX) + ((1 - k) * degreesAccY);
    return angleComp;
  }

  // Safe fallback — return last known good angle instead of garbage
  return angleComp;
}


// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  //Serial.begin(9600);
  //while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);  // INPUT_PULLUP avoids floating pin noise
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);

  if (!IMU.begin()) {
   // Serial.println("Failed to initialize IMU!");
    while (1);
  }

  angleGyroPrev = 0;

  angleSetpoint = 1.18;
  driveSetpoint = 0;

  anglePID.SetMode(AUTOMATIC);
  drivePID.SetMode(AUTOMATIC);
  anglePID.SetOutputLimits(-100, 100);
  drivePID.SetOutputLimits(-20, 20);
  anglePID.SetSampleTime(5);   // 
  drivePID.SetSampleTime(20);          // slower is fine for velocity

  avePWM = 0;
  difPWM = 0;
  aveRPM = 0;
  difRPM = 0;
    
  lastLoopTime = millis();
}

// ── Loop ──────────────────────────────────────────────────────────
// Per-motor state for calcRPM
long  prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime1 = 0, prevTime2 = 0;

void loop() {
  // float start_time = millis();
  // static unsigned long lastPrint = 0;
  // unsigned long now = millis();

  float tilt_angle = angle();
  //Serial.println(tilt_angle);

  if ((tilt_angle >= -30)&&(tilt_angle <= 30)) {

    if (anglePID.GetMode() == MANUAL) {
        anglePID.SetMode(AUTOMATIC);
    }

    angleInput = tilt_angle;
    anglePID.Compute();

    avePWM = angleOutput;
    
    pwm1 = avePWM + difPWM/2; //100.0 * (float)tilt_angle / 90.0;
    pwm2 = avePWM - difPWM/2; //100.0 * (float)tilt_angle / 90.0;

    if (pwm1 > 100) {pwm1 = 100;} else if (pwm1 < -100) {pwm1 = -100;}
		if (pwm2 > 100) {pwm2 = 100;} else if (pwm2 < -100) {pwm2 = -100;}
  }
  
  else {
    pwm1 = 0;
    pwm2 = 0;
    anglePID.SetMode(MANUAL); 
  }

  setMotor1(pwm1);
  setMotor2(pwm2);
  //Serial.println(millis() - start_time);
}