#include <ArduinoBLE.h>
#include <string.h>
#define BUFFER_SIZE 20

// ── Pin Definitions ──────────────────────────────────────────────
const int ledPin    = LED_BUILTIN;

//motor pins
const int motor1in1 = 6;
const int motor1in2 = 5;
const int motor2in1 = 8;
const int motor2in2 = 7;

//encoder pins
const int enc1A = 2;   // Motor 1 encoder channel A
const int enc1B = 9;   // Motor 1 encoder channel B
const int enc2A = 3;  // Motor 2 encoder channel A
const int enc2B = 10;   // Motor 2 encoder channel B

// ── SET YOUR PWM HERE (0–255) ─────────────────────────────────────
int pwm1 = 0; //db is 15, -16, usable range is 240
int pwm2 = 0; //db is 19, -16
int db1f = 57, db1r = 57; //old f is 36
int db2f = 60, db2r = 57; //old f is 35

int prev1A; int prev1B; int prev2A; int prev2B;

int curr1A; int curr1B; int curr2A; int curr2B;

int encCount1 = 0; int encCount2 = 0;

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

//set pwm from input from app 
void setPWMWithFlutter(char* data) {

    if (strcmp(data, "FORWARD") == 0) {
        pwm1 = 100;
        pwm2 = 100;
    }
    else if (strcmp(data, "BACKWARD") == 0) {
        pwm1 = -100;
        pwm2 = -100;
    }
    else if (strcmp(data, "LEFT") == 0) {
        pwm1 = 100;
        pwm2 = -100;
    }
    else if (strcmp(data, "RIGHT") == 0) {
        pwm1 = -100;
        pwm2 = 100;
    }
    else if (strcmp(data, "A") == 0) {
        pwm1 = 0;
        pwm2 = 0;
    }
    else {
        pwm1 = 0;
        pwm2 = 0;
    }

    setMotor1(pwm1);
    setMotor2(pwm2);
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

void setup() {
  Serial.begin(9600);
  //while (!Serial);
    
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
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

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) {
       // Get the length of the received data
        int length = customCharacteristic.valueLength();

        // Read the received data
        const unsigned char* receivedData = customCharacteristic.value();

        // Create a properly terminated string
        char receivedString[length + 1]; // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0'; // Null-terminate the string

        // Print the received data to the Serial Monitor
        Serial.print("Received data: ");
        Serial.println(receivedString);


        // Optionally, respond by updating the characteristic's value
        customCharacteristic.writeValue("Data received");
        setPWMWithFlutter(receivedString);
      }
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }
}