// Easy Non-blocking Ultrasonic Distance Sensor Library
// Drop-in replacement for blocking pulseIn() function
// Just call setupUltrasonic() in setup() and nbPulseIn() anytime for distance

const int trigPin = 2;  // Trigger pin (INT0)
const int echoPin = 3;  // Echo pin (INT1)

// Internal interrupt variables
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool newDistanceReady = false;
volatile bool measurementInProgress = false;

// Distance state
float lastDistance = 0;
unsigned long lastTrigger = 0;
const unsigned long TRIGGER_INTERVAL = 50; // Auto-trigger every 50ms for high refresh rate

void setup() {
  Serial.begin(9600);
  
  // Initialize ultrasonic sensor (one-line setup!)
  setupUltrasonic();
  
  Serial.println("=== EASY ULTRASONIC TEST ===");
  Serial.println("Using nbPulseIn() function");
  Serial.println("Auto-refreshing distance every 50ms");
  Serial.println("Just call nbPulseIn() to get distance!\n");
  
  // Demo the ease of use
  demoEasyUsage();
}

void loop() {
  // SUPER EASY USAGE - just call nbPulseIn() anytime!
  float distance = nbPulseIn();
  
  // Display with status
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm");
  
  if (distance < 10) {
    Serial.print(" [VERY CLOSE!]");
  } else if (distance < 20) {
    Serial.print(" [CLOSE]");
  } else if (distance < 40) {
    Serial.print(" [MODERATE]");
  } else if (distance > 200) {
    Serial.print(" [OUT OF RANGE]");
  }
  
  Serial.println();
  
  delay(200); // Display update rate
}

// ===== EASY-TO-USE LIBRARY FUNCTIONS =====

// Call this once in setup() - that's it!
void setupUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
  Serial.println("Ultrasonic sensor initialized on pins 2&3 with interrupts");
}

// Call this anytime to get the latest distance in cm
// Returns the most recent measurement, never blocks!
float nbPulseIn() {
  // Auto-trigger new measurements
  if (millis() - lastTrigger >= TRIGGER_INTERVAL) {
    triggerMeasurement();
    lastTrigger = millis();
  }
  
  // Process any new readings
  if (newDistanceReady) {
    calculateDistance();
    newDistanceReady = false;
  }
  
  // Handle timeouts
  if (measurementInProgress && (millis() - echoStart > 30)) {
    measurementInProgress = false;
  }
  
  return lastDistance; // Always returns latest available distance
}

// ===== INTERNAL FUNCTIONS (you don't need to call these) =====

void triggerMeasurement() {
  if (!measurementInProgress) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    measurementInProgress = true;
  }
}

void echoISR() {
  if (digitalRead(echoPin) == HIGH) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    newDistanceReady = true;
    measurementInProgress = false;
  }
}

void calculateDistance() {
  if (echoEnd > echoStart) {
    unsigned long duration = echoEnd - echoStart;
    lastDistance = duration * 0.034 / 2; // Speed of sound calculation
  } else {
    // Keep previous distance on error
  }
}

// ===== DEMO FUNCTIONS =====

void demoEasyUsage() {
  Serial.println("Demo: How easy it is to use nbPulseIn():");
  Serial.println();
  Serial.println("// In setup():");
  Serial.println("setupUltrasonic();");
  Serial.println();
  Serial.println("// Anytime you want distance:");
  Serial.println("float dist = nbPulseIn();");
  Serial.println("if (dist < 20) { /* obstacle detected */ }");
  Serial.println();
  Serial.println("That's it! No delays, no blocking, always up-to-date!");
  Serial.println("========================================\n");
  
  delay(2000); // Let user read the demo
}
