// Example 2: Throttle + Steering control

void setup() {
  
  delay(2000);
  
  setCar(0.3, 0);    // Forward
  delay(500);

  setCar(0, 0);    // Stop
  delay(500);

  setCar(-0.3, 0);    // Back
  delay(500);

  setCar(0, 0);    // Stop
  delay(500);

  setSteering(0.3);
  delay(500);

  setCar(0, 0);    // Stop
  delay(500);
}

void loop() {}
