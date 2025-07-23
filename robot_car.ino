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
const int leftSensorPin = 4;   // Left IR sensor
const int rightSensorPin = 5;  // Right IR sensor

// --- MODE DEFINITIONS ---
void (*modeFunctions[4])() = { nullptr, nullptr, nullptr, nullptr };

const char* modeDescriptions[] = {
  "Remote control",  // 'a' (index 0)
  "Dance mode",      // 'b' (index 1)
  "Line following",  // 'c' (index 2)
  "Cage wandering"   // 'd' (index 3)
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
  
  // Initialize mode functions after all functions are defined
  modeFunctions[0] = updateCar;     // Remote control
  modeFunctions[1] = dance;         // Dance mode
  modeFunctions[2] = followLine;    // Line following
  modeFunctions[3] = wanderCage;    // Cage wandering
  
  println("Robot car ready. PWM freq 31Hz. Send 'F' for forward, 'S' for stop, '0'-'9' for speed.");
}

void loop() {
  // Always process incoming commands
  receiveChar();
  receiveModeSwitch();  // Check for mode changes first
  receiveDirection();
  receiveSpeed();
  receiveTrimAdjust();  // Check for trim adjustments
  
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

void updateCar() {
  float finalThrottle = throttle * speed;                // Combine direction and magnitude
  float finalSteering = steering * steerSpeed;           // Combine steering direction and steering speed
  
  car.setThrottle(finalThrottle);
  car.setSteering(finalSteering);
}

void dance() {
  car.setSteering(0.5f);  // left
  delay(500);
  car.setSteering(-0.5f); // right
  delay(500);
}

void followLine() {
  // Constant settings
  const unsigned long LOST_TIMEOUT_MS = 500;  // timeout for permanent line loss
  const float BASE_THROTTLE = 0.45;
  const float BASE_STEERING = 0.15;
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
    lastDirection = leftSensor - rightSensor;  // Remember this direction (swapped)
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
}

void wanderCage() {
  // Constant settings
  const float THROTTLE = 0.3;
  const float STEERING = 0.3;
  const unsigned long SPIN_TIME_MS = 600;
  
  // Static variables for cage wandering state
  static unsigned long spinStartTime = 0;
  static int spinDirection = 0;  // -1 = right, +1 = left
  
  // Check if we're still within spin timeout
  if (millis() - spinStartTime < SPIN_TIME_MS) {
    // Continue spinning in the chosen direction
    car.setThrottle(0.0);
    car.setSteering(STEERING * spinDirection);
  } else {
    // Not spinning - read sensors and check for boundaries
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
      
      car.setThrottle(0.0);
      car.setSteering(STEERING * spinDirection);
    } else {
      // No boundary detected - go straight ahead
      car.setThrottle(THROTTLE);
      car.setSteering(0.0);
    }
  }
}
