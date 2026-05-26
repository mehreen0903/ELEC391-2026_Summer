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

// ── SET PWM HERE (0–255) ─────────────────────────────────────
float pwm1 = 75; //db is 15, -16, usable range is 240
float pwm2 = 75; //db is 19, -16
int db1f = 36;
int db1r = 35;
int db2f = 35;
int db2r = 35;
//int motor1PWM = (int)(pwm1 / 100.0 * 255);  //
//int motor2PWM = (int)(pwm2 / 100.0 * 255);  //

// ── H-Bridge Control ──────────────────────────────────────────────
void setMotor1(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm >= 0) { //Goes in forward direction
    //pwm = (int)(38+( (float)(pwm/100) * 217));
    pwm = ((float)((255-db1f)*pwm/100))+db1f; //usable range for motor 1
    analogWrite(motor1in2, pwm);  
    digitalWrite(motor1in1, LOW); 
    }
  else if (pwm < 0) { //Goes in reverse direction
    pwm = (int)((db1r + (100-db1r) * (float)(-pwm/100)) / 100 * 255);
    //calculates usable range according to db in reverse direction
    digitalWrite(motor1in2, LOW);  
    analogWrite(motor1in1, pwm);
    }
  else { digitalWrite(motor1in1, HIGH); digitalWrite(motor1in2, HIGH); } //brake
  Serial.print("M1="); Serial.println((int)pwm);
}

void setMotor2(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm >= 0) { //goes in forward direction
    pwm = ((float)((255-db2f)*pwm/100))+db2f; //usable range for motor 2
    analogWrite(motor2in1, (int)pwm);  
    digitalWrite(motor2in2, LOW); 
    }
  else if (pwm < 0) { //goes in reverse direction
    pwm = (int)((db2r + (100-db2r) * (float)(-pwm/100)) / 100 * 255);
    //calculates usable range according to db in reverse direction
    digitalWrite(motor2in1, LOW);
    analogWrite(motor2in2, pwm); 
    }
  else { digitalWrite(motor2in1, HIGH); digitalWrite(motor2in2, HIGH); } //brake
  Serial.print("M2="); Serial.println(pwm);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  //configure pins
  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT);
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin,    OUTPUT);

  Serial.print("Running M1="); Serial.print(pwm1);
  Serial.print(" M2=");        Serial.println(pwm2);

  setMotor1(pwm1); //initiate motor 1 movement
  setMotor2(pwm2); //initiate motor 2 movement
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  // Motors are set in setup
}