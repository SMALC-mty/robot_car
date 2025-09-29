// --- PIN DEFINITIONS ---
const int leftSensorPin = A3;
const int rightSensorPin = A2;
const int trigPin = 2;
const int echoPin = 3;
int ENA = 11;
int IN1 = 12;
int IN2 = 10;
int ENB = 9;
int IN3 = 8;
int IN4 = 7;

void setup() {
  Serial.begin(9600);
  
  // Configure motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 127);
  analogWrite(ENB, 127);
  
  // Configure sensor pins
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Starting motor test in 5 seconds...");
  delay(5000);

  Serial.println("Both wheels forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);

  // Left back for 2s
  Serial.println("Left wheel back");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(2000);

  // Right back for 2s
  Serial.println("Right wheel back");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000);

  // Stop all motors
  Serial.println("Motors stopped");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Read IR sensors
  int leftSensor = digitalRead(leftSensorPin);
  int rightSensor = digitalRead(rightSensorPin);
  
  // Read ultrasonic sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  long distance = duration * 0.034 / 2; // Convert to cm
  
  // Print sensor readings
  Serial.print("Left IR: ");
  Serial.print(leftSensor);
  Serial.print(" | Right IR: ");
  Serial.print(rightSensor);
  Serial.print(" | Ultrasonic: ");
  if (duration == 0) {
    Serial.println("No echo");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  delay(500); // Update every 500ms
}