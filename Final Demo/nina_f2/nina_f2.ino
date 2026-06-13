#include "customIMU.h"
#include <ArduinoBLE.h>
#include <string.h>
#define BUFFER_SIZE 20

// Drive & turn implemented. Turn unlimited. Drive limited to 50 cm (3820 encoder counts)

//dt variables
const unsigned long LOOP_TIME_MS = 5000; //target loop time = 10 MICROSECONDS
unsigned long lastLoopTime = 0;

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
const int ledPin = LED_BUILTIN;

const int motor1in1 = 5;  // moved off interrupt pins
const int motor1in2 = 6;
const int motor2in1 = 7;
const int motor2in2 = 8;

const int enc1A = 2;  // INT0 – must be 2 or 3 on Uno
const int enc1B = 9;
const int enc2A = 3;  // INT1 – must be 2 or 3 on Uno
const int enc2B = 10;

// ── PWM / Deadband ────────────────────────────────────────────────
int pwm1 = 0;
int pwm2 = 0;
int db1f = 60, db1r = 60;  //old f is 36
int db2f = 60, db2r = 60;  //old f is 35

// ── Encoder counts (volatile = modified in ISR) ───────────────────
volatile long encCount1 = 0;
volatile long encCount2 = 0;
// Per-motor state for calcRPM
long prevCount1 = 0, prevCount2 = 0;

// -- Acc and gyro vars ------------------------------------------
float angleGyroX = 0;
float angleGyroPrev = 0;
float angleComp = 0;
float degreesAccY = 0;
float turnAngle = 0;

// ── PID Struct Vars ───────────────────────────────────────────────
PID_t anglePID;
PID_t drivePID;
PID_t turnPID;

float avePWM = 0;
float difPWM = 0;
float aveRPM = 0;
float difRPM = 0;

float drivePWM = 0;
float turnPWM = 0;

int driveCount = 0;
float driveTime = 0;
float driveEnc1 = 0;
float driveEnc2 = 0;
float driveCommand = 0;

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
float calcRPM(volatile long &encCount, long &prevCount, float dt_seconds, float &driveEnc) { //CORRECTED ENCODER
//   unsigned long now = millis();
//   unsigned long dt = now - prevTime;
//   if (dt == 0) return 0;

  noInterrupts();
  long current = encCount;
  interrupts();

  long delta = current - prevCount;
  prevCount = current;

  driveEnc += (float)delta;

  float revs = (float)delta / 1920.0f;    //MAKE INTO CONST!
  return -revs / dt_seconds;
}

// ── Angle Calc & Control ──────────────────────────────────────────
float angle(float dt_seconds) {
  float gx, gy, gz;
  float ax, ay, az;
  float k = 0.99;

  // Only compute if BOTH are available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    degreesAccY = -1 * atan(ay / az) * 180 / PI + 0.25;

    // float currentLoopTime = millis();
    // float loopDuration = currentLoopTime - lastLoopTime;
    // lastLoopTime = currentLoopTime;

    angleGyroX = angleComp + (gx - 0.3f) * (dt_seconds);
    angleComp = (k * angleGyroX) + ((1.0f - k) * degreesAccY);

    turnAngle += (gz+0.06f) * dt_seconds;
    
  }
  return angleComp;
}

float setDriveWithFlutter(int data) {
  switch (data) {
    case 1:        //FORWARD
      return 0.6;  //max rpm is 300 ish
      break;
    case 3:  //BACKWARDS
      return -0.5; //bc of unequal dbr
      break;
    case 2:  //LEFT
      return 0.0;
      break;
    case 4:  //RIGHT
      return 0.0;
      break;
    case 5:  //A OR STOP
      return 0.0;
      break;
    default:     //STOP
      return 0.0;  // Invalid command, do nothing
      break;
  }

  return 0;  // Default target angle
}

// sets diff pwm
float setTurnWithFlutter(int data) {
  switch (data) {
    case 1:  //FORWARD
      return 0.0;
      break;
    case 3:  //BACKWARDS
      return 0.0;
      break;
    case 2:  //LEFT
      return 10.0;
      break;
    case 4:  //RIGHT
      return -10.0;
      break;
    case 5:  //A OR STOP
      return 0.0;
      break;
    default:       //STOP
      return 0.0;  // Invalid command, do nothing
      break;
  }
  // setMotor1(pwm1);
  // setMotor2(pwm2);
  return 0.0;  // Default target angle
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

// ── BLE ──────────────────────────────────────────
void handleBLE() {
    BLEDevice central = BLE.central();  // Wait for a BLE central to connect

    if (central) {
      int command = 0;  // TODO: dont need this since else doesnt use it

      // Keep running while connected
      if (central.connected()) {
        // Check if the characteristic was written
        if (customCharacteristic.written()) {
          // Read the single byte directly
          const unsigned char *receivedData = customCharacteristic.value();
          int command = receivedData[0];

          customCharacteristic.writeValue("Data received");
          driveCommand = setDriveWithFlutter(command); // adds pwm
          turnPWM  = setTurnWithFlutter(command);
          // Reset errors so derivative doesn't spike on new command
        //   drivePID.Error0 = 0;
        //   drivePID.Error1 = 0;
          drivePID.ErrorInt = 0;
        }
      }
      else {
        driveCommand = 0.0;
        turnPWM = 0.0;  //setDifPWMWithFlutter(command);
      }
    }
}
// ── PID Stoff ─────────────────────────────────────────────────────
void PID_Init(PID_t &pid) {
  pid.Actual = 0;
  pid.Target = 0;
  pid.Output = 0;
  pid.Error0 = 0;
  pid.Error1 = 0;
  pid.ErrorInt = 0;
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
  //Serial.begin(9600);
  //while (!Serial);
  //--- MOTOR FLUTTER CODE BELOW-----------------------------------------
  // Initialize the built-in LED to indicate connection status

  lastLoopTime = micros(); //update once here for first dt

  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    //Serial.println("Starting BLE failed!");
    while (1)
      ;
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

  //--- NINAS LAST FIGHTING CHANCE CODE BELOW ----------------------------
  pinMode(motor1in1, OUTPUT);
  pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT);
  pinMode(motor2in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);  // INPUT_PULLUP avoids floating pin noise
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);

  if (!IMU.begin()) {
    // Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  PID_Init(anglePID);  // For DB = 58: 2.1, 6, 0.08, 1.18, 100, 16
  anglePID = {
    .Kp = 2.1, //2.1
    .Ki = 8,//7.3, //6
    .Kd = 0.15,//0.06, //0.08
    .TargetDefault = 1.22,//1.18 // target angle
    .OutputMax = 100,
    .ErrorIntMax = 12.5,//10, //16
  };

  PID_Init(drivePID); 
  drivePID = {
    .Kp = 2.5,
    .Ki = 0.75,
    .Kd = 0.005,
    .TargetDefault = 0, // default target speed
    .OutputMax = 2, //max target angle
    .ErrorIntMax = 2.67
  };

  PID_Init(turnPID); 
  turnPID = {
    .Kp = 10.0,
    .Ki = 0,
    .Kd = 0,
    .TargetDefault = 0, // default target speed
    .OutputMax = 15, //max target angle
    .ErrorIntMax = 5
  };

  anglePID.Target = anglePID.TargetDefault;  //setup angle PID
  drivePID.Target = drivePID.TargetDefault;  //setup drive PID, this is target speed
  turnPID.Target = turnPID.TargetDefault;    //setup turn PID, this is target angular velocity
  anglePID.ErrorInt = 0;  //clears the error so it doesn't jerk when propping it up again?
  drivePID.ErrorInt = 0;
  turnPID.ErrorInt = 0;
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
   //float start_time = millis();
    //core control loop window 
  unsigned long currentTime = micros();
  if (currentTime - lastLoopTime >= LOOP_TIME_MS) { //LOOP_TIME_MS currently set to 10000 microseconds (10 ms)
    float dt_seconds = (currentTime - lastLoopTime) / 1000000.0; // Convert to seconds
    lastLoopTime = currentTime; //iterate time
    //const float dt_seconds = 0.005f; //hard code dt_seconds to 5 ms to avoid integer rounding jitter
    
    driveCount++;
    driveTime += dt_seconds;
    
    float tilt_angle = angle(dt_seconds);

    if(turnPWM != 0 && abs(turnAngle) < 41){
      difPWM = turnPWM;
    } else {
      difPWM = turnPID.Output;
    }

    if (tilt_angle >= -30.0 && tilt_angle <= 30.0) {
      anglePID.Actual = tilt_angle;
      updatePID(anglePID, dt_seconds); 

      avePWM = anglePID.Output;
      pwm1 = avePWM + (difPWM / 2.0f) ;
      pwm2 = avePWM - (difPWM / 2.0f) ;

      // pwm1 = constrain(pwm1, -100, 100);
      // pwm2 = constrain(pwm2, -100, 100);
    // TO DO: we dont need this??
    } else {
      //fall protection
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
      float rpm1 = calcRPM(encCount1, prevCount1, driveTime, driveEnc1);
      float rpm2 = calcRPM(encCount2, prevCount2, driveTime, driveEnc2);

      aveRPM = (rpm1 + rpm2) * 0.5;
      difRPM = rpm2 - rpm1;
      drivePID.Actual = aveRPM;  //we changed it so that this is now in RPS!!
      turnPID.Actual = difRPM;

      updatePID(drivePID, driveTime);
      updatePID(turnPID, driveTime);
      
      anglePID.Target = drivePID.Output + anglePID.TargetDefault; //clamped drivepid.output to 3 deg
      
      handleBLE(); // set drivePWM and turnPWM w/ bleh

      if (turnPWM == 0) {
      turnAngle = 0;
      }

      if (driveCommand == 0) {
        driveEnc1 = 0;
        driveEnc2 = 0;
      }

      if(driveCommand != 0 && abs((driveEnc1 + driveEnc2)/2.0f) < 3840){
        drivePID.Target = driveCommand;
      } else {
        drivePID.Target = 0.0;        
      }

    //   if(drivePWM == 0){
    //     anglePID.Target = drivePID.Output + anglePID.TargetDefault;
    //   } else {
    //     anglePID.Target = -drivePID.Output + anglePID.TargetDefault;
    //   }

      // driveCount = 0;
      // driveTime = 0; //dt for drive pid
    }
  }
}