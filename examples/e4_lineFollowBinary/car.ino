// Car control functions - reusable for all examples

// Motor pins
int ENA = 11, IN1 = 12, IN2 = 10;
int ENB = 9, IN3 = 8, IN4 = 7;

// Trim adjustment (-1.0 to 1.0)
float trim = 0.0;
bool carInitialized = false;
float currentThrottle = 0.0;
float currentSteering = 0.0;

void setupCar() {
  // Configure PWM frequencies for better low-speed control (31Hz)
  TCCR1B = (TCCR1B & 0xF8) | 0x05; // Prescaler 1024
  TCCR2B = (TCCR2B & 0xF8) | 0x07; // Prescaler 1024
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  carInitialized = true;
}

void setCar(float throttle, float steering) {
  if (!carInitialized) setupCar();
  float left = throttle + steering;
  float right = throttle - steering;
  
  left = constrain(left, -1.0, 1.0);
  right = constrain(right, -1.0, 1.0);
  
  // Apply trim compensation
  if (trim < 0) left *= (1.0 + trim);   // Slow down left motor
  if (trim > 0) right *= (1.0 - trim);  // Slow down right motor
  
  // Left motor
  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left <= 0 ? HIGH : LOW);
  analogWrite(ENA, abs(left) * 255);
  
  // Right motor
  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right <= 0 ? HIGH : LOW);
  analogWrite(ENB, abs(right) * 255);

  currentThrottle = throttle;
  currentSteering = steering;
}

void setTrim(float t) {
  trim = constrain(t, -1.0, 1.0);
}

void setThrottle(float throttle) {
  setCar(throttle, currentSteering);
}

void setSteering(float steering) {
  setCar(currentThrottle, steering);
}