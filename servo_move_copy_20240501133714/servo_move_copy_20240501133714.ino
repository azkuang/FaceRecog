#include <Servo.h>

Servo myServo;  // Create a servo object to control a servo
int currentAngle = 90;  // Current angle of the servo
int targetAngle = 90;  // Target angle to move the servo to
int stepSize = 1;  // Size of each step in degrees
unsigned long moveInterval = 20;  // Time between moves in milliseconds
unsigned long lastMoveTime = 0;  // Last move time

void setup() {
  myServo.attach(3);
  Serial.begin(9600);
  myServo.write(currentAngle);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the new target angle from the serial port
    int newAngle = Serial.parseInt();
    if (newAngle >= 0 && newAngle <= 180) {  // Simple validation
      targetAngle = newAngle;
    }
    // Clear the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // Check if it's time to move the servo again
  if (millis() - lastMoveTime > moveInterval) {
    moveServo();
    lastMoveTime = millis();
  }
}

void moveServo() {
  if (currentAngle < targetAngle) {
    currentAngle += min(stepSize, targetAngle - currentAngle);
    myServo.write(currentAngle);
  } else if (currentAngle > targetAngle) {
    currentAngle -= min(stepSize, currentAngle - targetAngle);
    myServo.write(currentAngle);
  }
}
