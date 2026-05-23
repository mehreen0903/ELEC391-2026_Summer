// ── Pin Definitions ───────────────────────────────────────────────
const int ledPin    = LED_BUILTIN; //make it so it hits a 100 pwm and then decreases. and in different directions (get reverse deadband)
const int motor1in1 = 5;
const int motor1in2 = 6;
const int motor2in1 = 7;
const int motor2in2 = 8;
const int enc1A = 2;
const int enc1B = 9;
const int enc2A = 3;
const int enc2B = 10;

// ── Encoder counts ────────────────────────────────────────────────
volatile long encCount1 = 0;
volatile long encCount2 = 0;

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
const float CPR = 1920.0;

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

  return ((float)delta / CPR / (float)dt) * 60000.0f;
}

// ── Motor Control (db=0 for characterization) ─────────────────────
// Writes raw PWM directly — no deadband mapping while characterizing
void setMotor1(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(motor1in2, pwm);
  digitalWrite(motor1in1, LOW);
}

void setMotor2(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(motor2in1, pwm);
  digitalWrite(motor2in2, LOW);
}

// ── State ─────────────────────────────────────────────────────────
int  currentPWM1  = 0;
int  currentPWM2  = 0;
bool motor1kicked = false;   // has M1 started moving?
bool motor2kicked = false;
bool rampDone     = false;

long  prevCount1 = 0, prevCount2 = 0;
unsigned long prevTime1 = 0, prevTime2 = 0;

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);

  setMotor1(0);
  setMotor2(0);

  Serial.println("Starting ramp. Motors off, encoders zeroed.");
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  if (rampDone) return;

  // ── Snapshot encoder counts safely ──
  noInterrupts();
  long count1 = encCount1;
  long count2 = encCount2;
  interrupts();

  // ── Detect first movement ────────────────────────────────────────
  if (!motor1kicked && count1 != 0) {
    motor1kicked = true;
    Serial.print(">>> M1 started moving at PWM = ");
    Serial.println(currentPWM1);
  }
  if (!motor2kicked && count2 != 0) {
    motor2kicked = true;
    Serial.print(">>> M2 started moving at PWM = ");
    Serial.println(currentPWM2);
  }

  // ── RPM print every 100ms ────────────────────────────────────────
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 100) {
    float rpm1 = calcRPM(encCount1, prevCount1, prevTime1);
    float rpm2 = calcRPM(encCount2, prevCount2, prevTime2);
    Serial.print("PWM1="); Serial.print(currentPWM1);
    Serial.print(" RPM1="); Serial.print(rpm1);
    Serial.print(" | PWM2="); Serial.print(currentPWM2);
    Serial.print(" RPM2="); Serial.println(rpm2);
    lastPrint = now;
  }

  // ── Ramp complete check ──────────────────────────────────────────
  if (currentPWM1 >= 255 && currentPWM2 >= 255) {
    rampDone = true;
    Serial.println("=== Ramp complete ===");
    if (!motor1kicked) Serial.println("WARNING: M1 never moved!");
    if (!motor2kicked) Serial.println("WARNING: M2 never moved!");
    return;
  }

  // ── Step PWM up ──────────────────────────────────────────────────
  if (currentPWM1 < 255) currentPWM1++;
  if (currentPWM2 < 255) currentPWM2++;

  setMotor1(currentPWM1);
  setMotor2(currentPWM2);

  delay(100);   // 100ms per step → full ramp in ~25s
}
// // ── Pin Definitions (CHANGED FROM PART 3) ─────────────────────────
// const int ledPin    = LED_BUILTIN;

// const int motor1in1 = 5;   // moved off interrupt pins
// const int motor1in2 = 6;
// const int motor2in1 = 7;
// const int motor2in2 = 8;

// const int enc1A = 2;   // INT0 – must be 2 or 3 on Uno
// const int enc1B = 9;
// const int enc2A = 3;   // INT1 – must be 2 or 3 on Uno
// const int enc2B = 10;

// // ── PWM / Deadband ────────────────────────────────────────────────
// int pwm1 = 0;
// int pwm2 = 0;
// int db1f = 0;//36, 
// int db1r = 0;//70;
// int db2f = 0;//35, 
// int db2r = 0;//70;

// // ── Encoder counts (volatile = modified in ISR) ───────────────────
// volatile long encCount1 = 0;
// volatile long encCount2 = 0;

// int  currentPWM1 = 0;
// int  currentPWM2 = 0;
// bool rampDone    = false;

// // ── ISRs ──────────────────────────────────────────────────────────
// void ISR_enc1() {
//   int a = digitalRead(enc1A);
//   int b = digitalRead(enc1B);
//   encCount1 += (a == b) ? -2 : 2;
// }

// void ISR_enc2() {
//   int a = digitalRead(enc2A);
//   int b = digitalRead(enc2B);
//   encCount2 += (a == b) ? 2 : -2;
// }

// // ── RPM Calculation ───────────────────────────────────────────────
// // Separate state for each motor
// float calcRPM(volatile long &encCount, long &prevCount, unsigned long &prevTime) {
//   unsigned long now = millis();
//   unsigned long dt = now - prevTime;
//   if (dt == 0) return 0;

//   noInterrupts();
//   long current = encCount;
//   interrupts();

//   long delta = current - prevCount;
//   prevCount = current;
//   prevTime = now;

//   float revs = (float)delta / 1920.0; //MAKE INTO CONST!
//   return (revs / (float)dt) * 60000.0f;   // RPM
// }

// // ── H-Bridge Control ──────────────────────────────────────────────
// void setMotor1(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   if      (pwm >= 0) { 
//     //pwm = (int)(38+( (float)(pwm/100) * 217));
//     pwm = ((float)((255-db1f)*pwm/100))+db1f;
//     analogWrite(motor1in2, (int)pwm);  
//     digitalWrite(motor1in1, LOW); 
//     }
//   else if (pwm < 0) { 
//     pwm = (int)((db1r + (100-db1r) * (float)(-pwm/100)) / 100 * 255);
//     digitalWrite(motor1in2, LOW);  
//     analogWrite(motor1in1, pwm);
//     }
//   else { digitalWrite(motor1in1, HIGH); digitalWrite(motor1in2, HIGH); }
//   Serial.print("M1="); Serial.println((int)pwm);
// }

// void setMotor2(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   if      (pwm >= 0) { 
//     //pwm = (int)((db2f + (100-db2f) * (float)(pwm/100)) / 100 * 255);
//     pwm = ((float)((255-db2f)*pwm/100))+db2f;
//     analogWrite(motor2in1, pwm);  
//     digitalWrite(motor2in2, LOW); 
//     }
//   else if (pwm < 0) { 
//     pwm = (int)((db2r + (100-db2r) * (float)(-pwm/100)) / 100 * 255);
//     digitalWrite(motor2in1, LOW);  
//     analogWrite(motor2in2, pwm); 
//     }
//   else { digitalWrite(motor2in1, HIGH); digitalWrite(motor2in2, HIGH); }
//   Serial.print("M2="); Serial.println(pwm);
// }

// // ── Setup ─────────────────────────────────────────────────────────
// void setup() {
//   Serial.begin(9600);
//   while (!Serial);

//   pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
//   pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
//   pinMode(ledPin,    OUTPUT);

//   pinMode(enc1A, INPUT_PULLUP);  // INPUT_PULLUP avoids floating pin noise
//   pinMode(enc1B, INPUT_PULLUP);
//   pinMode(enc2A, INPUT_PULLUP);
//   pinMode(enc2B, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(enc1A), ISR_enc1, CHANGE); //??
//   attachInterrupt(digitalPinToInterrupt(enc2A), ISR_enc2, CHANGE);

//   setMotor1(currentPWM1);
//   setMotor2(currentPWM2);
// }

// // ── Loop ──────────────────────────────────────────────────────────

// long  prevCount1 = 0, prevCount2 = 0;
// unsigned long prevTime1 = 0, prevTime2 = 0;

// void loop() {

//   static unsigned long lastPrint = 0;
//   unsigned long now = millis();

//   if (now - lastPrint >= 100) {   // print every 100 ms
//     float rpm1 = calcRPM(encCount1, prevCount1, prevTime1);
//     float rpm2 = calcRPM(encCount2, prevCount2, prevTime2);

//     Serial.print("RPM1 = "); Serial.print(rpm1);
//     Serial.print("  RPM2 = "); Serial.println(rpm2);
//     // Serial.print("  enc1 = "); Serial.print(encCount1);
//     // Serial.print("  enc2 = "); Serial.println(encCount2);

//     lastPrint = now;
//   }

//  if (rampDone) return;

//   Serial.print("PWM1: "); Serial.print(currentPWM1);
//   Serial.print(" | PWM2: "); Serial.println(currentPWM2);

//   setMotor1(currentPWM1);
//   setMotor2(currentPWM2);

//   if (currentPWM1 >= 100 && currentPWM2 >= 100) {
//     rampDone = true;
//     Serial.println("Ramp complete.");
//     Serial.print("Final PWM1: "); Serial.println(currentPWM1);
//     Serial.print("Final PWM2: "); Serial.println(currentPWM2);
//     return;
//   }

//   if (currentPWM1 < 100) currentPWM1++;
//   if (currentPWM2 < 100) currentPWM2++;

//   delay(100);
// }