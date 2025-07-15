// --- DUAL SERIAL WRAPPER FUNCTIONS ---
// These functions automatically send output to both USB Serial and Bluetooth Serial
// and read input from whichever source has data available

void println(const String& msg) {
  Serial.println(msg);
  BTSerial.println(msg);
}

void print(const String& msg) {
  Serial.print(msg);
  BTSerial.print(msg);
}

bool available() {
  return Serial.available() || BTSerial.available();
}

char read() {
  if (Serial.available()) {
    return Serial.read();
  } else if (BTSerial.available()) {
    return BTSerial.read();
  }
  return 0; // No data available
}
