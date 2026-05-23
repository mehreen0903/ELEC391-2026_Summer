// ── Pin Definitions ──────────────────────────────────────────────
const int ledPin    = LED_BUILTIN;

// motor pins
const int motor1in1 = 2;
const int motor1in2 = 3;
const int motor2in1 = 5;
const int motor2in2 = 6;

// encoder pins
const int enc1A = 7;
const int enc1B = 8;
const int enc2A = 9;
const int enc2B = 10;

// ── Set PWM here [-100, 100] ─────────────────────────────────────
int pwm1 = 50; //db is 15, -16, usable range is 240
int pwm2 = -50; //db is 19, -16
int db1f = 36;
int db1r = 70; //recheck db for reverse
int db2f = 35;
int db2r = 70;

// ── H-Bridge Control ──────────────────────────────────────────────
// sets motor 1 speed and direction based on pwm value
void setMotor1(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { 
    pwm = ((float)((255 - db1f) * pwm/100)) + db1f; //calculates usable range according to db in forward direction
    analogWrite(motor1in2, (int)pwm);  // in2 = pwm
    digitalWrite(motor1in1, LOW);  // in1 = 0 for forward
    }

  else if (pwm < 0) { 
    pwm = ((float)((255 - db1r) * pwm/100)) + db1r; //calculates usable range according to db in reverse direction
    digitalWrite(motor1in2, LOW);  // in2 = 0
    analogWrite(motor1in1, pwm); // in1 = pwm for reverse
    }

  else { digitalWrite(motor1in1, HIGH); digitalWrite(motor1in2, HIGH); } // else brake

  Serial.print("M1="); Serial.println((int)pwm);
}

// sets motor 2 speed and direction based on pwm value
void setMotor2(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if      (pwm >= 0) { 
    pwm = ((float)((255 - db2f) * pwm/100)) + db2f; //calculates usable range according to db in forward direction
    analogWrite(motor2in1, pwm);  // in1 = pwm
    digitalWrite(motor2in2, LOW); // in2 = 0 for forward
    }

  else if (pwm < 0) { 
    pwm = ((float)((255 - db2r) * pwm/100)) + db2r; //calculates usable range according to db in reverse direction
    digitalWrite(motor2in1, LOW); // in1 = 0
    analogWrite(motor2in2, pwm);  // in2 = pwm for reverse
    }

  else { digitalWrite(motor2in1, HIGH); digitalWrite(motor2in2, HIGH); } //brake
  Serial.print("M2="); Serial.println(pwm);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(motor1in1, OUTPUT); pinMode(motor1in2, OUTPUT); // configures a pin to input or output mode
  pinMode(motor2in1, OUTPUT); pinMode(motor2in2, OUTPUT);
  pinMode(ledPin,    OUTPUT);

  Serial.print("Running M1="); Serial.print(pwm1); // for debugging
  Serial.print(" M2=");        Serial.println(pwm2);

  setMotor1(pwm1);
  setMotor2(pwm2);
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {

}