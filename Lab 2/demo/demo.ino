// ── Pin Definitions ──────────────────────────────────────────────
const int ledPin    = LED_BUILTIN;
const int motor1in1 = 2;
const int motor1in2 = 3;
const int motor2in1 = 5;
const int motor2in2 = 6;
const int enc1A = 7;   // Motor 1 encoder channel A
const int enc1B = 8;   // Motor 1 encoder channel B
const int enc2A = 9;  // Motor 2 encoder channel A
const int enc2B = 10;   // Motor 2 encoder channel B

// ── SET YOUR PWM HERE (0–255) ─────────────────────────────────────
int pwm1 = 0; //db is 15, -16, usable range is 240
int pwm2 = 0; //db is 19, -16
int db1f = 36;
int db1r = 70;
int db2f = 35;
int db2r = 70;
//int motor1PWM = (int)(pwm1 / 100.0 * 255);  //
//int motor2PWM = (int)(pwm2 / 100.0 * 255);  //

int  currentPWM1 = 0;
int  currentPWM2 = 0;
bool rampDone    = false;

// ── H-Bridge Control ──────────────────────────────────────────────
void setMotor1(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { 
    //pwm = (int)(38+( (float)(pwm/100) * 217));
    pwm = ((float)((255-db1f)*pwm/100))+db1f;
    analogWrite(motor1in2, (int)pwm);  
    digitalWrite(motor1in1, LOW); 
    }
  else if (pwm < 0) { 
    pwm = (int)((db1r + (100-db1r) * (float)(-pwm/100)) / 100 * 255);
    digitalWrite(motor1in2, LOW);  
    analogWrite(motor1in1, pwm);
    }
  else { digitalWrite(motor1in1, HIGH); digitalWrite(motor1in2, HIGH); }
  Serial.print("M1="); Serial.println((int)pwm);
}

void setMotor2(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { 
    //pwm = (int)((db2f + (100-db2f) * (float)(pwm/100)) / 100 * 255);
    pwm = ((float)((255-db2f)*pwm/100))+db2f;
    analogWrite(motor2in1, pwm);  
    digitalWrite(motor2in2, LOW); 
    }
  else if (pwm < 0) { 
    pwm = (int)((db2r + (100-db2r) * (float)(-pwm/100)) / 100 * 255);
    digitalWrite(motor2in1, LOW);  
    analogWrite(motor2in2, pwm); 
    }
  else { digitalWrite(motor2in1, HIGH); digitalWrite(motor2in2, HIGH); }
  Serial.print("M2="); Serial.println(pwm);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin,    OUTPUT);

  Serial.print("Running M1="); Serial.print(currentPWM1);
  Serial.print(" M2=");        Serial.println(currentPWM2);

  setMotor1(currentPWM1);
  setMotor2(currentPWM2);
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
 if (rampDone) return;

  Serial.print("PWM1: "); Serial.print(currentPWM1);
  Serial.print(" | PWM2: "); Serial.println(currentPWM2);

  setMotor1(currentPWM1);
  setMotor2(currentPWM2);

  if (currentPWM1 >= 100 && currentPWM2 >= 100) {
    rampDone = true;
    Serial.println("Ramp complete.");
    Serial.print("Final PWM1: "); Serial.println(currentPWM1);
    Serial.print("Final PWM2: "); Serial.println(currentPWM2);
    return;
  }

  if (currentPWM1 < 100) currentPWM1++;
  if (currentPWM2 < 100) currentPWM2++;

  delay(1000);
}