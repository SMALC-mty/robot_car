// Motor driver pin assignments
int ENA = 11; // Left PWM
int IN1 = 12; // Left IN1
int IN2 = 10; // Left IN2
int ENB = 9;  // Right PWM
int IN3 = 8;  // Right IN3
int IN4 = 7;  // Right IN4

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set both enables to 50% PWM (127 out of 255)
  analogWrite(ENA, 127);
  analogWrite(ENB, 127);

  delay(5000); // Startup delay

  // Both wheels forward for 2s
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);

  // Left back for 2s
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(2000);

  // Right back for 2s
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000);

  // Stop all motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  // Optionally, set ENA/ENB to 0 if you want to fully stop
  // analogWrite(ENA, 0);
  // analogWrite(ENB, 0);
}

void loop() {
  // Nothing here. Press reset to rerun the test.
}