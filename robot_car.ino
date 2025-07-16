#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DiffCar.h"

// --- PIN DEFINITIONS ---
SoftwareSerial BTSerial(10, 11); // RX, TX
DiffCar car(
  5,  // ENA
  7,  // IN1
  6,  // IN2
  3,  // ENB
  4,  // IN3
  2   // IN4
);

// --- SENSOR PIN DEFINITIONS ---
const int leftSensorPin = 8;   // Left IR sensor
const int rightSensorPin = 9;  // Right IR sensor

// --- GLOBAL STATE VARIABLES ---
int throttle = 0;        // Direction: -1 (reverse), 0 (stop), 1 (forward)
int steering = 0;        // Steering: -1 (right), 0 (straight), 1 (left)
float speed = 0.5;       // Speed multiplier: 0.0 to 1.0
float steerSpeed = 0.5;  // Steering speed multiplier: 0.0 to 1.0
char receivedChar = 0;   // Current received character (0 = null/none)
char currentMode = 'a';  // Current mode: 'a' = remote control, 'b' = dance, etc.

// --- LINE FOLLOWING STATE VARIABLES ---
int lostCounter = 0;       // counts iterations since line lost
int lastCorrection = 0;    // -1 = turning left, +1 = turning right, 0 = straight
const int LOST_MAX_ITER = 10;  // number of loops to keep turning before giving up
const float LINE_STRAIGHT_SPEED = 0.4;   // forward speed for line following
const float LINE_REVERSE_SPEED = 0.4;    // reverse speed for corrections

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // Configure sensor pins for line following
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  
  println("Robot car ready. Send 'F' for forward, 'S' for stop, '0'-'9' for speed.");
}

void loop() {
  // Always process incoming commands
  receiveChar();
  receiveModeSwitch();  // Check for mode changes first
  receiveDirection();
  receiveSpeed();
  
  // Execute current mode behavior
  if (currentMode == 'a') {
    updateCar(); // Remote control mode
  }
  else if (currentMode == 'b') {
    dance(); // Dance mode
  }
  else if (currentMode == 'c') {
    followLine(); // Line following mode
  }
}

void receiveChar() {
  if (available()) {
    receivedChar = read();
  } else {
    receivedChar = 0;  // No character received
  }
}

void receiveDirection() {
  if (receivedChar == 'F') {
    throttle = 1; steering = 0;
  }
  else if (receivedChar == 'B') {
    throttle = -1; steering = 0;
  }
  else if (receivedChar == 'L') {
    throttle = 0; steering = 1;
  }
  else if (receivedChar == 'R') {
    throttle = 0; steering = -1;
  }
  else if (receivedChar == 'G') {
    throttle = 1; steering = 1;
  }
  else if (receivedChar == 'H') {
    throttle = 1; steering = -1;
  }
  else if (receivedChar == 'I') {
    throttle = -1; steering = 1;
  }
  else if (receivedChar == 'J') {
    throttle = -1; steering = -1;
  }
  else if (receivedChar == 'S') {
    throttle = 0; steering = 0;
  }
  // If not a direction command, throttle and steering remain unchanged
}

void receiveSpeed() {
  if (receivedChar >= '0' && receivedChar <= '9') {
    int digit = receivedChar - '0';  // Convert char to int
    speed = (digit + 1) / 10.0;      // 0->0.1, 1->0.2, ..., 9->1.0
  }
  else if (receivedChar >= '!' && receivedChar <= '*') {
    int digit = receivedChar - '!';  // Convert symbol to int (! = 0, " = 1, ..., * = 9)
    steerSpeed = (digit + 1) / 10.0; // 0->0.1, 1->0.2, ..., 9->1.0
  }
  // If not 0-9 or !-*, speeds remain unchanged
}

void updateCar() {
  float finalThrottle = throttle * speed;                // Combine direction and magnitude
  float finalSteering = steering * steerSpeed;           // Combine steering direction and steering speed
  
  car.setThrottle(finalThrottle);
  car.setSteering(finalSteering);
  
  // Minimal debug output - only for significant commands
  if (receivedChar == 'F' || receivedChar == 'B' || receivedChar == 'S') {
    Serial.print(receivedChar); Serial.print(":T="); Serial.print(finalThrottle); 
    Serial.print(",S="); Serial.println(finalSteering);
  }
}

void dance() {
  car.setSteering(0.5f);  // left
  delay(500);
  car.setSteering(-0.5f); // right
  delay(500);
}

void receiveModeSwitch() {
  if (receivedChar >= 'a' && receivedChar <= 'z') {
    currentMode = receivedChar;
    Serial.print("Mode switched to: ");
    Serial.println(currentMode);
  }
}

void followLine() {
  // Read sensor states (HIGH = black line, LOW = white background)
  bool leftOnLine = digitalRead(leftSensorPin);
  bool rightOnLine = digitalRead(rightSensorPin);
  
  // 1) Both sensors on line → drive straight + reset counters
  if (leftOnLine && rightOnLine) {
    lostCounter = 0;
    lastCorrection = 0;
    // Drive straight - both wheels forward at same speed
    car.setLeft(LINE_STRAIGHT_SPEED);
    car.setRight(LINE_STRAIGHT_SPEED);
  }
  // 2) Left sensor sees white → pivot left (stop right wheel, reverse left)
  else if (!leftOnLine && rightOnLine) {
    lostCounter = 0;
    lastCorrection = -1;
    // Stop right wheel, reverse left wheel
    car.setLeft(-LINE_REVERSE_SPEED);  // Reverse left
    car.setRight(0.0);                 // Stop right
  }
  // 3) Right sensor sees white → pivot right (stop left wheel, reverse right)
  else if (!rightOnLine && leftOnLine) {
    lostCounter = 0;
    lastCorrection = 1;
    // Stop left wheel, reverse right wheel
    car.setLeft(0.0);                  // Stop left
    car.setRight(-LINE_REVERSE_SPEED); // Reverse right
  }
  // 4) Both sensors see white → line lost, try to reacquire
  else {
    lostCounter++;
    if (lostCounter <= LOST_MAX_ITER && lastCorrection != 0) {
      // Continue the previous correction direction
      if (lastCorrection < 0) {
        // Continue pivot left
        car.setLeft(-LINE_REVERSE_SPEED);
        car.setRight(0.0);
      } else {
        // Continue pivot right
        car.setLeft(0.0);
        car.setRight(-LINE_REVERSE_SPEED);
      }
    } else {
      // Give up → stop both wheels
      car.setLeft(0.0);
      car.setRight(0.0);
    }
  }
  delay(1);
}
