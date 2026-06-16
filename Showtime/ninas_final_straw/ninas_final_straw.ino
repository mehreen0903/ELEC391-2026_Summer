// SHOWTIME: BALANCES FOR 30 SECONDS WITH +/- 2 CM
// 11.3 V
#include "customIMU.h"

struct PID_t {
  float Kp;
  float Ki;
  float Kd;
  float TargetDefault;
  float OutputMax;
  float ErrorIntMax;

  float Actual;
  float Target;
  float Output;
  float Error0;
  float Error1;
  float ErrorInt;
};

unsigned long lastLoopTime = 0;

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
int db1f = 70, db1r = 70; //old f is 36
int db2f = 70, db2r = 70; //old f is 35

// ── Encoder counts (volatile = modified in ISR) ───────────────────
volatile long encCount1 = 0;
volatile long encCount2 = 0;

// -- Acc and gyro vars ------------------------------------------
float angleGyroX = 0;
float angleGyroPrev = 0;
float angleComp = 0;
float degreesAccY = 0;

PID_t anglePID;
PID_t drivePID;
PID_t turnPID;

float avePWM = 0;
float difPWM = 0;
float aveRPM = 0;
float difRPM = 0; 

int driveCount = 0;
float driveTime = 0;

void PID_Init(PID_t &pid) {
  pid.Actual = 0;
  pid.Target = 0;
  pid.Output = 0;
  pid.Error0 = 0;
  pid.Error1 = 0;
  pid.ErrorInt = 0;
}


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
float calcRPM(volatile long &encCount, long &prevCount, float dt_seconds) {

  noInterrupts();
  long current = encCount;
  interrupts();

  long delta = current - prevCount;
  prevCount = current;

  float revs = (float)delta / 1920.0f;    //MAKE INTO CONST!
  return -revs / dt_seconds;
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

float angle(float dt_seconds) {
  float gx, gy, gz;
  float ax, ay, az;
  float k = 0.99;

  // Only compute if BOTH are available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    degreesAccY = -1 * atan(ay / az) * 180 / PI;

    angleGyroX = angleComp + (gx-0.3) * (dt_seconds);
    angleComp = (k * angleGyroX) + ((1.0f - k) * degreesAccY);

    //turnAngle += (gz+0.06f) * dt_seconds;
    //Serial.println(gz);
    //Serial.println(gx);
  }
  return angleComp;
}

void updatePID(PID_t &pid, float dt_seconds) {
  
  pid.Error1 = pid.Error0;
  pid.Error0 = pid.Target - pid.Actual;

  if (pid.Ki != 0) {
    pid.ErrorInt += pid.Error0 * dt_seconds;
    pid.ErrorInt = constrain(pid.ErrorInt, -pid.ErrorIntMax, pid.ErrorIntMax);
  } else {
    pid.ErrorInt = 0;
  }  
  float derivative = (pid.Error0 - pid.Error1) / dt_seconds;
  pid.Output = (pid.Kp * pid.Error0) + (pid.Ki * pid.ErrorInt) + (pid.Kd * derivative);

  pid.Output = constrain(pid.Output, -pid.OutputMax, pid.OutputMax);
}


// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  // Serial.begin(9600);
  // delay(1500);
  // Serial.println("Hello World");
    //IMU.setAccelODR(100);  // 25 / 50 / 100 / 200 / 400 Hz
    //IMU.setGyroODR(400);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //while (!Serial);
  lastLoopTime = micros();

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);  // INPUT_PULLUP avoids floating pin noise
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);
  
  // if (!IMU.begin()) {
  //  Serial.println("Failed to initialize IMU!");
  //   while (1);
  // }

  //angleGyroPrev = 0;

  PID_Init(anglePID);  
  anglePID = {
    .Kp = 3,
    .Ki = 8, //0.08,
    .Kd = 0.05,//5,
    .TargetDefault = 1.18,
    .OutputMax = 100,
    .ErrorIntMax = 12
  };

  PID_Init(drivePID); 
  drivePID = {
    .Kp = 2,
    .Ki = 1.5,
    .Kd = 0.005,
    .TargetDefault = 0, // default target speed
    .OutputMax = 2.0, //max target angle
    .ErrorIntMax = 1.33
  };

  PID_Init(turnPID); 
  turnPID = {
    .Kp = 15,
    .Ki = 1,
    .Kd = 0,
    .TargetDefault = 0, // default target speed
    .OutputMax = 15.0, //max target angle
    .ErrorIntMax = 15.0
  };

  anglePID.Target = anglePID.TargetDefault;
  drivePID.Target = drivePID.TargetDefault;
  turnPID.Target = turnPID.TargetDefault;
  anglePID.ErrorInt = 0;
  drivePID.ErrorInt = 0;
  turnPID.ErrorInt = 0;
}

// ── Loop ──────────────────────────────────────────────────────────
// Per-motor state for calcRPM
long  prevCount1 = 0, prevCount2 = 0;
//unsigned long prevTime1 = 0, prevTime2 = 0;

void loop() {

  unsigned long currentTime = micros();

  if (currentTime - lastLoopTime >= 4000.0) {

    float dt_seconds = (currentTime - lastLoopTime) / 1000000.0;
    lastLoopTime = currentTime;

    driveCount++;
    driveTime += dt_seconds;

    float tilt_angle = angle(dt_seconds);

    if (tilt_angle >= -30.0 && tilt_angle <= 30.0) {
      anglePID.Actual = tilt_angle;
      updatePID(anglePID, dt_seconds); 

      avePWM = anglePID.Output;
      pwm1 = avePWM + (difPWM / 2.0f);
      pwm2 = avePWM - (difPWM / 2.0f);
      // Serial.print(pwm1);
      // Serial.print(" ");
      // Serial.println(pwm2);

    } else {
      pwm1 = 0; pwm2 = 0;
      anglePID.ErrorInt = 0;
      drivePID.ErrorInt = 0;
      turnPID.ErrorInt = 0;
      drivePID.Error0 = 0; 
      drivePID.Error1 = 0;  // new commands after falling should work better now
    }  

    setMotor1(pwm1);
    setMotor2(pwm2);
    
    if (driveCount >= 5) {
      float rpm1 = calcRPM(encCount1, prevCount1, driveTime);
      float rpm2 = calcRPM(encCount2, prevCount2, driveTime);
      aveRPM = (rpm1 + rpm2) * 0.5;
      difRPM = rpm2 - rpm1;
      drivePID.Actual = aveRPM;  //we changed it so that this is now in RPS!!
      turnPID.Actual = difRPM;

      updatePID(drivePID, driveTime);
      updatePID(turnPID, driveTime);
      
      anglePID.Target = drivePID.Output + anglePID.TargetDefault; //clamped drivepid.output to 3 deg
      difPWM = turnPID.Output;

      driveCount = 0;
      driveTime = 0; //dt for drive pid
    }
  }
}