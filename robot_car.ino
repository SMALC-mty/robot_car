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
  receiveTrimAdjust();  // Check for trim adjustments
  
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
  else if (currentMode == 'd') {
    followLineV2(); // Alternative line following mode
  }
  else if (currentMode == 'e') {
    wanderCage(); // Cage wandering mode
  }
  else {
    Serial.println("Unknown mode. Defaulting to remote control.");
    currentMode = 'a'; // Fallback to remote control
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
    currentMode = receivedChar;
    Serial.print("Mode switched to: ");
    Serial.println(currentMode);
  }
}

void receiveTrimAdjust() {
  if (receivedChar >= 'k' && receivedChar <= '~') {
    // Map k-~ (107-126) to -9 to +9, with 's' (115) as center (0)
    int trimStep = receivedChar - 's';  // 's' becomes 0
    float trim_factor = 0.2f;
    float trimValue = trimStep * 0.1 * trim_factor;
    car.setTrim(trimValue);
    Serial.print("Trim: "); Serial.println(trimValue);
  }
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

void followLine() {
  // Static variables for line following state (encapsulated within function)
  static unsigned long lineLostTime = 0;    // timestamp when line was lost (millis)
  static int lastCorrection = 0;            // -1 = turning left, +1 = turning right, 0 = straight
  static const unsigned long LOST_TIMEOUT_MS = 500;  // milliseconds to keep searching before giving up
  static const float LINE_STRAIGHT_SPEED = 0.4;      // forward speed for line following
  static const float LINE_REVERSE_SPEED = 0.4;       // reverse speed for corrections
  
  // Read sensor states (HIGH = black line, LOW = white background)
  bool leftOnLine = digitalRead(leftSensorPin);
  bool rightOnLine = digitalRead(rightSensorPin);
  
  // 1) Both sensors on line → drive straight + reset timer
  if (leftOnLine && rightOnLine) {
    lineLostTime = 0;  // Reset lost timer
    lastCorrection = 0;
    // Drive straight - both wheels forward at same speed
    car.setLeft(LINE_STRAIGHT_SPEED);
    car.setRight(LINE_STRAIGHT_SPEED);
  }
  // 2) Left sensor sees white → pivot left (stop right wheel, reverse left)
  else if (!leftOnLine && rightOnLine) {
    lineLostTime = 0;  // Reset lost timer (line partially detected)
    lastCorrection = -1;
    // Stop right wheel, reverse left wheel
    car.setLeft(-LINE_REVERSE_SPEED);  // Reverse left
    car.setRight(0.0);                 // Stop right
  }
  // 3) Right sensor sees white → pivot right (stop left wheel, reverse right)
  else if (!rightOnLine && leftOnLine) {
    lineLostTime = 0;  // Reset lost timer (line partially detected)
    lastCorrection = 1;
    // Stop left wheel, reverse right wheel
    car.setLeft(0.0);                  // Stop left
    car.setRight(-LINE_REVERSE_SPEED); // Reverse right
  }
  // 4) Both sensors see white → line lost, try to reacquire
  else {
    // Start timing if we just lost the line
    if (lineLostTime == 0) {
      lineLostTime = millis();
    }
    
    unsigned long elapsedTime = millis() - lineLostTime;
    
    if (elapsedTime < LOST_TIMEOUT_MS && lastCorrection != 0) {
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
      // Give up → stop
      car.stop();
    }
  }
  delay(1);
}

void followLineV2() {
  // Constant settings
  const unsigned long LOST_TIMEOUT_MS = 500;  // timeout for permanent line loss
  const float BASE_THROTTLE = 1.0;
  const float BASE_STEERING = 1.0;
  const float ALIGN_THRESHOLD = 0.0;  // Below this, robot will reverse

  // Static variables for error-based line following state
  static unsigned long lineLastSeen = 0;        // timestamp when line was last detected
  static float lastDirection = 0.0;         // remember last correction direction
  
  // Read sensor states (HIGH = black line, LOW = white background)
  int leftSensor = digitalRead(leftSensorPin);   // 0 or 1
  int rightSensor = digitalRead(rightSensorPin); // 0 or 1
  
  // Determine error based on sensor states
  float error = 0.0;
  if (leftSensor || rightSensor) { // Check if at least one sensor is on the line
    lastDirection = -leftSensor + rightSensor;  // Remember this direction
    error = lastDirection * 0.5; // Scale error to -0.5 to +0.5 range
    lineLastSeen = millis();      // Update timestamp when line is detected
  }
  else { // Both sensors off line - handle line loss case
    error = lastDirection; // Use last known direction
  }
  
  // Initialize motor commands
  float throttle = 0.0;
  float steering = 0.0;
  
  // Check if timeout has been exceeded
  if (millis() - lineLastSeen <= LOST_TIMEOUT_MS) {
    // Determine alignment (1 - absolute value of error)
    float alignment = 1.0 - abs(error);
    
    // Calculate throttle factor using alignment threshold
    float throttle_factor = (alignment - ALIGN_THRESHOLD) / (1.0 - ALIGN_THRESHOLD);
    
    // Calculate throttle and steering based on alignment and error
    throttle = BASE_THROTTLE * throttle_factor;  // Can go negative for reverse
    steering = BASE_STEERING * error;            // Steer proportionally to error
  }
  
  // Apply motor commands
  car.setThrottle(throttle);
  car.setSteering(steering);

  Serial.print("Error: "); Serial.print(error);
  Serial.print(", T: "); Serial.print(throttle);
  Serial.print(", S: "); Serial.println(steering);
}

void wanderCage() {
  // Constant settings
  const float THROTTLE = 1.0;
  const float STEERING = 1.0;
  const unsigned long SPIN_TIME_MS = 1000;
  
  // Static variables for cage wandering state
  static unsigned long spinStartTime = 0;
  static int spinDirection = 0;  // -1 = right, +1 = left
  
  // Read sensor states (HIGH = black line/boundary, LOW = white interior)
  bool leftSensor = digitalRead(leftSensorPin);
  bool rightSensor = digitalRead(rightSensorPin);
  
  // Check if we hit a boundary
  if (leftSensor || rightSensor) {
    // Hit boundary - start spinning away from it
    spinStartTime = millis();
    
    if (leftSensor && !rightSensor) {
      spinDirection = -1;  // Left sensor hit - spin right (away from boundary)
    } else {
      // Right sensor hit OR both sensors hit - spin left (away from boundary)
      spinDirection = 1;
    }
  }
  
  // Check if we're still within spin timeout
  if (millis() - spinStartTime < SPIN_TIME_MS) {
    // Continue spinning in the chosen direction
    car.setThrottle(0.0);
    car.setSteering(STEERING * spinDirection);
  } else {
    // Spin timeout elapsed or no boundary - go straight ahead
    car.setThrottle(THROTTLE);
    car.setSteering(0.0);
  }
}
