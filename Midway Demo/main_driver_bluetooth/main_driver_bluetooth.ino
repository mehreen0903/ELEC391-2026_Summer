#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <string.h>
#include <Wire.h>
#define BUFFER_SIZE 1

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
int db1f = 67, db1r = 67; //old f is 36
int db2f = 67, db2r = 67; //old f is 35

// ── Encoder counts (volatile = modified in ISR) ───────────────────
volatile long encCount1 = 0;
volatile long encCount2 = 0;
// Per-motor state for calcRPM
long  prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime1 = 0, prevTime2 = 0;

// -- Acc and gyro vars ------------------------------------------
float angleGyroX = 0;
float angleGyroPrev = 0;
float angleComp = 0;
float lastLoopTime = 0;
//float ax, ay, az;
float degreesAccY = 0;

// ── PID Struct Vars ───────────────────────────────────────────────
PID_t anglePID;
PID_t drivePID;

float avePWM = 0;
float difPWM = 0;
float aveRPM = 0;
float difRPM = 0; 

// Define a custom BLE service and characteristic --------------------
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
"00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

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

// ── Angle Calc & Control ──────────────────────────────────────────
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

float setAveRPMWithFlutter(int data) {
  switch(data) {
    case 1: //FORWARD
        return 0.4; //max rpm is 300 ish
        break;
    case 3: //BACKWARDS
        return -0.4;
        break;
    case 2: //LEFT
        return 0.2;
        break;
    case 4: //RIGHT
        return 0.2;
        break;
    case 5: //A OR STOP
        return 0;
        break;
    default: //STOP
        return 0; // Invalid command, do nothing
        break;
    }
    // setMotor1(pwm1);
    // setMotor2(pwm2);
    return 0; // Default target angle
}

float setDifPWMWithFlutter(int data) {
  switch(data) {
    case 1: //FORWARD
        return 0.0;
        break;
    case 3: //BACKWARDS
        return 0.0;
        break;
    case 2: //LEFT
        return 20.0;
        break;
    case 4: //RIGHT
        return -20.0;
        break;
    case 5: //A OR STOP
        return 0.0;
        break;
    default: //STOP
        return 0.0; // Invalid command, do nothing
        break;
    }
    // setMotor1(pwm1);
    // setMotor2(pwm2);
    return 0.0; // Default target angle
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

// ── PID Stoff ─────────────────────────────────────────────────────
void PID_Init (PID_t &pid) {
  pid.Actual = 0;
  pid.Target = 0;
  pid.Output = 0;
  pid.Error0 = 0;
  pid.Error1 = 0;
  pid.ErrorInt = 0;
}

void updatePID(PID_t &pid){
  pid.Error1 = pid.Error0;
  pid.Error0 = pid.Target-pid.Actual;

  if (pid.Ki != 0){
		pid.ErrorInt += pid.Error0;
	} else {
		pid.ErrorInt = 0;
	}

  constrain(pid.ErrorInt, -pid.ErrorIntMax, pid.ErrorIntMax);

  pid.Output = pid.Kp * pid.Error0 + pid.Ki * pid.ErrorInt + pid.Kd * (pid.Error0 - pid.Error1);

  constrain(pid.Output, -pid.OutputMax, pid.OutputMax);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  //Serial.begin(9600);
  //while (!Serial);

  //--- MOTOR FLUTTER CODE BELOW-----------------------------------------
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    //Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("NINA");
  BLE.setDeviceName("NINA");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  //Serial.println("Bluetooth® device active, waiting for connections...");

  //--- NINAS LAST FIGHTING CHANCE CODE BELOW ----------------------------
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

  Wire.setClock(400000); // Bump I2C speed from 100kHz to 400kHz

  PID_Init(anglePID);
  
  anglePID = {
    .Kp = 3,
    .Ki = 0.04,
    .Kd = 10,
    .TargetDefault = 1.11,
    .OutputMax = 100,
    .ErrorIntMax = 50
  };

  PID_Init(drivePID);
  
  drivePID = {
    .Kp = 2,
    .Ki = 0,
    .Kd = 0,
    .TargetDefault = 0,
    .OutputMax = 10,
    .ErrorIntMax = 5
  };

  anglePID.Target = anglePID.TargetDefault; //setup angle PID
  drivePID.Target = drivePID.TargetDefault; //setup drive PID, this is target speed
  
  lastLoopTime = millis();
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  //float start_time = millis();
  // --- MOTOR FLUTTER CODE BELOW ------------------------------------
  BLEDevice central = BLE.central(); // Wait for a BLE central to connect
  int command = 0;  // ← declare here so both if and else can see it

  if (central) {
  //   //Serial.print("Connected to central: ");
  //   //Serial.println(central.address());

  //   

  //   // Keep running while connected
  //   if (central.connected()) {
  //     // Check if the characteristic was written
  //     
  // 
  if (customCharacteristic.written()) {

        // Read the single byte directly
        const unsigned char* receivedData = customCharacteristic.value();
        int command = receivedData[0];   // grab the byte as an int

        //Serial.print("Received command: ");
        //Serial.println(command);

        customCharacteristic.writeValue("Data received");
        drivePID.Target = setAveRPMWithFlutter(command);      // set target angle according to button press, and pass int, not char* 
        difPWM = setDifPWMWithFlutter(command);
      }
    
  } 

  else {
      drivePID.Target = drivePID.TargetDefault;      // set target angle according to button press, and pass int, not char* 
      difPWM = 0; //setDifPWMWithFlutter(command);
  }

    //Serial.println("Disconnected from central.");
  
  

  // --- NINAS LAST FIGHTING CHANCE CODE BELOW ---------------------------------
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  float rpm1 = calcRPM(encCount1, prevCount1, prevTime1);
  float rpm2 = calcRPM(encCount2, prevCount2, prevTime2);
  aveRPM = (rpm1 + rpm2)/2;
  difRPM = rpm1 - rpm2;

  float tilt_angle = angle();
  //Serial.println(tilt_angle);
  float driveAngle;

  if ((tilt_angle >= -30)&&(tilt_angle <= 30)) {
    
    if (drivePID.Target != 0) {
      drivePID.Actual = aveRPM / 60; //convertinf RPM to RPS
      updatePID(drivePID);
      driveAngle = drivePID.Output;
    } else {
      driveAngle = 0;
    }
    anglePID.Target = driveAngle + anglePID.TargetDefault; //set target angle according to movement
    anglePID.Actual = tilt_angle;
    updatePID(anglePID);

    avePWM = anglePID.Output;
    
    pwm1 = avePWM + difPWM/2;
    pwm2 = avePWM - difPWM/2;

    constrain(pwm1, -100, 100);
    constrain(pwm2, -100, 100);
  }
  else {
    pwm1 = 0;
    pwm2 = 0;
  }

  setMotor1(pwm1);
  setMotor2(pwm2);
  //Serial.println(millis() - start_time);
}