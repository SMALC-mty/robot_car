// ultrasonic_nb.ino - Non-blocking ultrasonic sensor library
// Easy drop-in replacement for blocking pulseIn() function
// Usage: Call setupUltrasonic() in setup(), then nbPulseIn() anytime for distance

// Internal interrupt variables
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool newDistanceReady = false;
volatile bool measurementInProgress = false;

// Distance state
float lastDistance = 0;
unsigned long lastTrigger = 0;
const unsigned long TRIGGER_INTERVAL = 50; // Auto-trigger every 50ms for 20Hz refresh rate

// ===== PUBLIC FUNCTIONS =====

// Call this once in setup() to initialize the ultrasonic sensor
void setupUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
}

// Call this anytime to get the latest distance in cm
// Returns the most recent measurement, never blocks!
float nbPulseIn() {
  // Auto-trigger new measurements at regular intervals
  if (millis() - lastTrigger >= TRIGGER_INTERVAL) {
    triggerMeasurement();
    lastTrigger = millis();
  }
  
  // Process any new readings that came in via interrupt
  if (newDistanceReady) {
    calculateDistance();
    newDistanceReady = false;
  }
  
  // Handle measurement timeouts (no echo received)
  if (measurementInProgress && (millis() - echoStart > 30)) {
    measurementInProgress = false;
  }
  
  return lastDistance; // Always returns latest available distance
}

// ===== INTERNAL FUNCTIONS =====

// Triggers a new ultrasonic measurement
void triggerMeasurement() {
  if (!measurementInProgress) {
    // Send 10µs trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    measurementInProgress = true;
  }
}

// Interrupt service routine for echo pin state changes
void echoISR() {
  if (digitalRead(echoPin) == HIGH) {
    // Echo pulse started
    echoStart = micros();
  } else {
    // Echo pulse ended
    echoEnd = micros();
    newDistanceReady = true;
    measurementInProgress = false;
  }
}

// Calculates distance from echo pulse duration
void calculateDistance() {
  if (echoEnd > echoStart) {
    unsigned long duration = echoEnd - echoStart;
    lastDistance = duration * 0.034 / 2; // Speed of sound = 343 m/s
  }
  // If calculation fails, keep previous distance value
}
