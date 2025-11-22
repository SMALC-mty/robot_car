// Example 5: Proportional line following

// IR sensor pins
const int leftSensorPin = A3;
const int rightSensorPin = A2;

// Line following parameters - tuned for curvy tracks with chained 90° turns
const unsigned long LOST_TIMEOUT_MS = 600;   // Timeout for curve navigation
const float BASE_THROTTLE = 0.35;            // Reduced speed for better curve control
const float BASE_STEERING = 0.2;             // Steering authority for tight turns
const float ALIGN_THRESHOLD = 0;             // Threshold for alignment-based throttle

// State variables
unsigned long lineLastSeen = 0;
float lastDirection = 0.0;

void setup() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
}

void loop() {
  // Read sensor states (HIGH = black line, LOW = white background)
  int leftSensor = digitalRead(leftSensorPin);
  int rightSensor = digitalRead(rightSensorPin);
  
  // Determine error based on sensor states
  float error = 0.0;
  if (leftSensor || rightSensor) {
    lastDirection = rightSensor - leftSensor;  // Swapped to fix steering direction
    error = lastDirection * 0.5;  // Scale error to -0.5 to +0.5 range
    lineLastSeen = millis();
  }
  else {
    error = lastDirection;  // Use last known direction when line is lost
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
    throttle = BASE_THROTTLE * throttle_factor;
    steering = BASE_STEERING * error;
  }
  
  // Apply motor commands
  setCar(throttle, steering);
}
