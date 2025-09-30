// Example 4: Binary line following

// IR sensor pins
const int leftSensorPin = A3;
const int rightSensorPin = A2;

// Line following parameters
const float THROTTLE_ALIGNED = 0.3;    // Speed when both sensors on line
const float THROTTLE_MISALIGNED = 0.1; // Speed when off line
const float STEERING_MISALIGNED = 0.5; // Steering strength when correcting

void setup() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
}

void loop() {
  int leftSensor = digitalRead(leftSensorPin);
  int rightSensor = digitalRead(rightSensorPin);
  
  if (leftSensor == 0 && rightSensor == 0) {
    // Both sensors on line - go straight
    setCar(THROTTLE_ALIGNED, 0);
  }
  else if (leftSensor == 1 && rightSensor == 0) {
    // Left sensor off line - turn right
    setCar(THROTTLE_MISALIGNED, STEERING_MISALIGNED);
  }
  else if (leftSensor == 0 && rightSensor == 1) {
    // Right sensor off line - turn left
    setCar(THROTTLE_MISALIGNED, -STEERING_MISALIGNED);
  }
  else {
    // Both sensors off line - stop
    setCar(0, 0);
  }
}
