#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(9, 10); // RX, TX
Servo horizontalServo;
Servo verticalServo;
int horizontalPin = 5;
int verticalPin = 6;

// LDR Configuration
const int ldrTop = A0;    // Top Center
const int ldrLeft = A1;   // Left Center
const int ldrRight = A2;  // Right Center
const int ldrBottom = A3; // Bottom Center
const int voltinp = A5;

// Control Parameters
float sensitivity = 0.05;
unsigned long lastUpdate = 0;
const int updateInterval = 20;

// Servo angle limits
const int H_MIN = 25, H_MAX = 155;
const int V_MIN = 25, V_MAX = 120;
const int H_CENTER = 90, V_CENTER = 70;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  horizontalServo.attach(horizontalPin);
  verticalServo.attach(verticalPin);
  horizontalServo.write(H_CENTER);
  verticalServo.write(V_CENTER);
  delay(1000);

}

void loop() {
  // Handle Bluetooth commands if available
  if (BT.available()) {
    handleBluetooth();
    lastUpdate = millis();
  }

  // If no Bluetooth command for 10 seconds, track light
  if (millis() - lastUpdate > 10000) {
    trackLight();
  }
}

void trackLight() {
  if (millis() - lastUpdate < updateInterval) return;
  
  // Read initial LDR values
  int top = 0;
  int left = 0;
  int right = 0;
  int bottom = 0;

  // Print initial LDR values
  Serial.println("Initial LDR values:");
  Serial.print("Top: "); Serial.println(top);
  Serial.print("Left: "); Serial.println(left);
  Serial.print("Right: "); Serial.println(right);
  Serial.print("Bottom: "); Serial.println(bottom);

  // Read LDR values
  top = analogRead(ldrTop);
  left = analogRead(ldrLeft);
  right = analogRead(ldrRight);
  bottom = analogRead(ldrBottom);

  // Print current LDR values and solar panel voltage
  Serial.print("LDR1: "); Serial.print(top);
  Serial.print(" | LDR2: "); Serial.print(left); 
  Serial.print(" | LDR3: "); Serial.print(right);
  Serial.print(" | LDR4: "); Serial.print(bottom);
  Serial.print(" | Solar panel: "); Serial.println(analogRead(voltinp));

  // Calculate differences
  int hDiff = left - right;  // Positive = left brighter
  int vDiff = top - bottom;  // Positive = top brighter

  adjustServo(horizontalServo, hDiff * sensitivity, H_MIN, H_MAX);
  adjustServo(verticalServo, vDiff * sensitivity, V_MIN, V_MAX);

  lastUpdate = millis();
}

void adjustServo(Servo &servo, float adjustment, int minAngle, int maxAngle) {
  float newAngle = servo.read() + adjustment;
  newAngle = constrain(newAngle, minAngle, maxAngle);
  servo.write((int)newAngle);
}

void handleBluetooth() {
  while (BT.available()) {
    char command = BT.read();
    command = toupper(command);

    int currentH = horizontalServo.read();
    int currentV = verticalServo.read();

    switch (command) {
      case 'L':
        horizontalServo.write(constrain(currentH + 5, H_MIN, H_MAX));
        break;
      case 'R':
        horizontalServo.write(constrain(currentH - 5, H_MIN, H_MAX));
        break;
      case 'U':
        verticalServo.write(constrain(currentV + 5, V_MIN, V_MAX));
        break;
      case 'D':
        verticalServo.write(constrain(currentV - 5, V_MIN, V_MAX));
        break;
      case 'C':
        horizontalServo.write(H_CENTER);
        verticalServo.write(V_CENTER);
        break;
      case 'S':
        sendDiagnostics();
        break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
        break;
    }
    lastUpdate = millis();
  }
}

void sendDiagnostics() {
  BT.print("Horizontal: ");
  BT.print(horizontalServo.read());
  BT.print("° Vertical: ");
  BT.print(verticalServo.read());
  BT.print("° LDRs T/L/R/B: ");
  BT.print(analogRead(ldrTop));
  BT.print(",");
  BT.print(analogRead(ldrLeft));
  BT.print(",");
  BT.print(analogRead(ldrRight));
  BT.print(",");
  BT.println(analogRead(ldrBottom));
}
