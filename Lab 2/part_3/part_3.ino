// ── Pin Definitions ──────────────────────────────────────────────
const int ledPin    = LED_BUILTIN;

//Motor pins
const int motor1in1 = 2;
const int motor1in2 = 3;
const int motor2in1 = 5;
const int motor2in2 = 6;

// encoder pins
const int enc1A = 7;  // motor 1, channel A...
const int enc1B = 8;
const int enc2A = 9;
const int enc2B = 10; 

// ── Set PWM here [-100, 100] ─────────────────────────────────────
int pwm1 = 25; //db is 15, -16, usable range is ~240
int pwm2 = 25; //db is 19, -16
int db1f = 36;
int db1r = 70; //recheck db for reverse
int db2f = 35;
int db2r = 70;

// ── Encoder Variables ─────────────────────────────────────────────
int prev1A; int prev1B; int prev2A; int prev2B;

int curr1A; int curr1B; int curr2A; int curr2B;

int encCount1; int encCount2;

// ── H-Bridge Control ──────────────────────────────────────────────
// sets motor 1 speed and direction based on pwm value
void setMotor1(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { //
    pwm = ((float)((255 - db1f) * pwm/100)) + db1f; //calculates usable range according to db in forward direction
    analogWrite(motor1in2, (int)pwm);  // in2 = pwm
    digitalWrite(motor1in1, LOW); // in1 = 0 for forward
    }

  else if (pwm < 0) { 
    pwm = (int)((db1r + (100-db1r) * (float)(-pwm/100)) / 100 * 255);
    //calculates usable range according to db in reverse direction
    digitalWrite(motor1in2, LOW);  // in2 = 0
    analogWrite(motor1in1, pwm);   // in1 = pwm for reverse
    }

  else { digitalWrite(motor1in1, HIGH); digitalWrite(motor1in2, HIGH); } // else brake

  Serial.print("M1="); Serial.println((int)pwm);
}

// sets motor 2 speed and direction based on pwm value
void setMotor2(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { 
    pwm = ((float)((255 - db2f) * pwm/100)) + db2f; //calculates usable range according to db in forward direction
    analogWrite(motor2in1, pwm);   // in1 = pwm
    digitalWrite(motor2in2, LOW);  // in2 = 0 for forward
    }

  else if (pwm < 0) { 
    pwm = (int)((db2r + (100-db2r) * (float)(-pwm/100)) / 100 * 255);
    //calculates usable range according to db in reverse direction
    digitalWrite(motor2in1, LOW);  // in1 = 0
    analogWrite(motor2in2, pwm);   // in2 = pwm for reverse
    }

  else { digitalWrite(motor2in1, HIGH); digitalWrite(motor2in2, HIGH); } //brake

  Serial.print("M2="); Serial.println(pwm);
}

// set encoder increment
int countEncoder(int currA, int currB, int prevA, int prevB) {
  if (currB != prevB){
      if (currA != currB){
        return -2; //CCW, 2 to account for both
      } else {
        return 2; //CW
      } //CW
  } else {
    return 0;
  }
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT); // sets pin to i/o
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin,    OUTPUT);

  pinMode(enc1A, INPUT); pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT); pinMode(enc2B, INPUT);

  Serial.print("Running M1="); Serial.print(pwm1);
  Serial.print(" M2=");        Serial.println(pwm2);

  setMotor1(pwm1); //sets motor 1 to pwm1
  setMotor2(pwm2); //sets motor 2 to pwm2

  prev1A = digitalRead(enc1A); // reads initial encoder states
  prev1B = digitalRead(enc1B); 
  prev2A = digitalRead(enc2A);
  prev2B = digitalRead(enc2B);
  encCount1 = 0; // initalise to 0
  encCount2 = 0;
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  curr1A = digitalRead(enc1A); // reads new encoder states
  curr1B = digitalRead(enc1B);
  curr2A = digitalRead(enc2A);
  curr2B = digitalRead(enc2B);

  encCount1 += countEncoder(curr1A, curr1B, prev1A, prev1B);  
  encCount2 -= countEncoder(curr2A, curr2B, prev2A, prev2B); 
  
  Serial.print("E1 = "); Serial.print(encCount1); //figure out spacing
  Serial.print(" E2= "); Serial.println(encCount2);

  prev1A = curr1A;
  prev1B = curr1B; 
  prev2A = curr2A;
  prev2B = curr2B;
}