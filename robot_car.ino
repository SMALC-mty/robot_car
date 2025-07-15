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

// --- GLOBAL STATE VARIABLES ---
int throttle = 0;        // Direction: -1 (reverse), 0 (stop), 1 (forward)
int steering = 0;        // Steering: -1 (right), 0 (straight), 1 (left)
float speed = 0.5;       // Speed multiplier: 0.0 to 1.0
float steerSpeed = 0.5;  // Steering speed multiplier: 0.0 to 1.0
char receivedChar = 0;   // Current received character (0 = null/none)

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  println("Robot car ready. Send 'F' for forward, 'S' for stop, '0'-'9' for speed.");
}

void loop() {
  receiveChar();
  receiveDirection();
  receiveSpeed();
  updateCar();

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
