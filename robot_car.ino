#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DiffCar.h"

// --- PIN DEFINITIONS ---
SoftwareSerial BTSerial(2, 3); // RX, TX
DiffCar car(
  11, // ENA (Timer 2)
  12, // IN1
  10, // IN2
  9,  // ENB (Timer 1)
  8,  // IN3
  7,  // IN4
  -0.04f // Trim factor
);

// --- SENSOR PIN DEFINITIONS ---
const int leftSensorPin = A5;   // Left IR sensor (analog pin used as digital)
const int rightSensorPin = A4;  // Right IR sensor (analog pin used as digital)
const int trigPin = 4;          // Ultrasonic trigger pin
const int echoPin = 5;          // Ultrasonic echo pin

// --- MODE DEFINITIONS ---
void (*modeFunctions[5])() = { nullptr, nullptr, nullptr, nullptr, nullptr };

const char* modeDescriptions[] = {
  "Remote control",       // 'a' (index 0)
  "Dance mode",           // 'b' (index 1)
  "Line following",       // 'c' (index 2)
  "Cage wandering",       // 'd' (index 3)
  "Collision avoidance"   // 'e' (index 4)
};

const int NUM_MODES = sizeof(modeFunctions) / sizeof(modeFunctions[0]);

// --- GLOBAL STATE VARIABLES ---
int throttle = 0;        // Direction: -1 (reverse), 0 (stop), 1 (forward)
int steering = 0;        // Steering: -1 (right), 0 (straight), 1 (left)
float speed = 0.5;       // Speed multiplier: 0.0 to 1.0
float steerSpeed = 0.5;  // Steering speed multiplier: 0.0 to 1.0
char receivedChar = 0;   // Current received character (0 = null/none)
int currentMode = 0;     // Current mode index: 0 = remote control, 1 = dance, etc.
float trim_factor = 0.3f; // Trim adjustment factor

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // Configure PWM frequencies for better low-speed control (31Hz)
  TCCR1B = (TCCR1B & 0xF8) | 0x05; // Prescaler 1024
  TCCR2B = (TCCR2B & 0xF8) | 0x07; // Prescaler 1024
  
  // Configure sensor pins for line following
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  
  // Setup ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize mode functions after all functions are defined
  modeFunctions[0] = updateCar;        // Remote control
  modeFunctions[1] = dance;            // Dance mode
  modeFunctions[2] = followLine;       // Line following
  modeFunctions[3] = wanderCage;       // Cage wandering
  modeFunctions[4] = collisionAvoidance;   // Manual + Ultrasonic

  println("Robot car ready. PWM freq 31Hz. Send 'F' for forward, 'S' for stop, '0'-'9' for speed.");
}

void loop() {
  // Process ALL incoming commands until buffer is empty
  while (available()) {
    receiveChar();
    receiveModeSwitch();  // Check for mode changes first
    receiveDirection();
    receiveSpeed();
    receiveTrimAdjust();  // Check for trim adjustments
  }
  
  // Execute current mode behavior
  if (currentMode >= 0 && currentMode < NUM_MODES) {
    modeFunctions[currentMode]();  // Call the function at the index
  } else {
    print("Unknown mode. Defaulting to: ");
    currentMode = 0; // Fallback to first mode
    println(modeDescriptions[currentMode]);
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

void receiveModeSwitch() {
  if (receivedChar >= 'a' && receivedChar <= 'j') {  // Limit to a-j for modes
    int newMode = receivedChar - 'a';  // Convert 'a'-'j' to 0-9
    if (newMode < NUM_MODES) {
      currentMode = newMode;
      print("Mode: ");
      println(modeDescriptions[currentMode]);
    }
  }
}

void receiveTrimAdjust() {
  if (receivedChar >= 'k' && receivedChar <= '~') {
    // Map k-~ (107-126) to -9 to +9, with 's' (115) as center (0)
    int trimStep = receivedChar - 's';  // 's' becomes 0
    float trimValue = trimStep * 0.1 * trim_factor;
    car.setTrim(trimValue);
    print("Trim: ");
    println(trimValue);
  }
}

void getUserVels(float &outThrottle, float &outSteering) {
  outThrottle = throttle * speed;
  outSteering = steering * steerSpeed;
}

void updateCar() {
  float finalThrottle, finalSteering;
  getUserVels(finalThrottle, finalSteering);
  car.setThrottle(finalThrottle);
  car.setSteering(finalSteering);
}
