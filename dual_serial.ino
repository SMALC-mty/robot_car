// --- DUAL SERIAL WRAPPER FUNCTIONS ---
// These functions automatically send output to both USB Serial and Bluetooth Serial
// and read input from whichever source has data available

template<typename T>
void println(const T& msg) {
  Serial.println(msg);
  BTSerial.println(msg);
}

template<typename T>
void print(const T& msg) {
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
