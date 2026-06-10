#include <Wire.h>
#include <OPT3101.h>
#include <Servo.h>

// WHEELS 

// M2_PWM1 = rear left wheel
// M2_PWM2 = rear right wheel
// M1_PWM2 = front left wheel
// M1_PWM1 = front right wheel


// Motor pins
#define M2_DIR1 22  // rear left
#define M2_DIR2 24  // rear right
#define M2_PWM1 5
#define M2_PWM2 6

#define M1_DIR1 26  // front left
#define M1_DIR2 28  // front right
#define M1_PWM1 7
#define M1_PWM2 8

Servo servo1;

int speedForward  = 70;
int speedTurn     = 100;
int speedBackward = 100;

void stopMotors() {
  analogWrite(M1_PWM1, 0);
  analogWrite(M1_PWM2, 0);
  analogWrite(M2_PWM1, 0);
  analogWrite(M2_PWM2, 0);
}

void forward() {
  digitalWrite(M2_DIR1, LOW);
  digitalWrite(M2_DIR2, LOW);
  digitalWrite(M1_DIR1, HIGH);
  digitalWrite(M1_DIR2, HIGH);
  analogWrite(M2_PWM1, speedForward);
  analogWrite(M2_PWM2, speedForward);
  analogWrite(M1_PWM1, speedForward);
  analogWrite(M1_PWM2, speedForward);
}

void backward() {
  digitalWrite(M2_DIR1, HIGH);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M1_DIR2, LOW);
  analogWrite(M2_PWM1, speedBackward);
  analogWrite(M2_PWM2, speedBackward);
  analogWrite(M1_PWM1, speedBackward);
  analogWrite(M1_PWM2, speedBackward);
}

void turnLeft() {
  // left side backward, right side forward
  digitalWrite(M2_DIR1, HIGH);
  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M2_DIR2, LOW);
  digitalWrite(M1_DIR2, HIGH);
  analogWrite(M2_PWM1, speedTurn);
  analogWrite(M2_PWM2, speedTurn);
  analogWrite(M1_PWM1, speedTurn);
  analogWrite(M1_PWM2, speedTurn);
}

void turnRight() {
  // left side forward, right side backward
  digitalWrite(M2_DIR1, LOW);
  digitalWrite(M1_DIR1, HIGH);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(M1_DIR2, LOW);
  analogWrite(M2_PWM1, speedTurn);
  analogWrite(M2_PWM2, speedTurn);
  analogWrite(M1_PWM1, speedTurn);
  analogWrite(M1_PWM2, speedTurn);
}

void backwardLeft() {
  digitalWrite(M2_DIR1, HIGH);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M1_DIR2, LOW);
  analogWrite(M2_PWM1, speedBackward);
  analogWrite(M1_PWM2, speedBackward);
  analogWrite(M2_PWM2, 0);
  analogWrite(M1_PWM1, 0);
}

void backwardRight() {
  digitalWrite(M2_DIR1, HIGH);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M1_DIR2, LOW);
  analogWrite(M2_PWM1, 0);
  analogWrite(M1_PWM2, 0);
  analogWrite(M2_PWM2, speedBackward);
  analogWrite(M1_PWM1, speedBackward);
}

void handleCommand(String cmd) {
  cmd.trim();

  if      (cmd == "FORWARD")        forward();
  else if (cmd == "BACKWARD")       backward();
  else if (cmd == "LEFT")           turnLeft();
  else if (cmd == "RIGHT")          turnRight();
  else if (cmd == "BACKWARD_LEFT")  backwardLeft();
  else if (cmd == "BACKWARD_RIGHT") backwardRight();
  else if (cmd == "STOP")           stopMotors();
  else if (cmd.startsWith("SPEED:")) {
    int s = cmd.substring(6).toInt();
    if (s > 0 && s <= 255) {
      speedForward  = s;
      speedTurn     = min(s + 30, 255);
      speedBackward = min(s + 30, 255);
      Serial.print("Speed set to: ");
      Serial.println(s);
    }
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void setup() {
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);
  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);

  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);

  servo1.attach(9);
  stopMotors();

  Serial.begin(9600);
  Serial.println("Arduino ready");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}




