#include <Arduino.h>

// Differential‑drive mix/unmix functions (classic, unclamped mix)
inline void diffMix(float t, float s, float &L, float &R) {
  L = t - s;
  R = t + s;
}

inline void diffUnmix(float L, float R, float &t, float &s) {
  t = (L + R) * 0.5f;
  s = (R - L) * 0.5f;
}


class DiffCar {
public:
  // Constructor: ENA, IN1, IN2, ENB, IN3, IN4
  DiffCar(uint8_t ena, uint8_t in1, uint8_t in2,
          uint8_t enb, uint8_t in3, uint8_t in4)
  {
    // store pins
    _ENA  = ena;  _IN1 = in1;  _IN2 = in2;
    _ENB  = enb;  _IN3 = in3;  _IN4 = in4;
    // initialize raw wheel speeds
    _left  = 0.0f;
    _right = 0.0f;
    // initialize trim (0 = no adjustment)
    _trim = -0.1f;
    // configure pins
    pinMode(_ENA, OUTPUT);
    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);
    pinMode(_ENB, OUTPUT);
    pinMode(_IN3, OUTPUT);
    pinMode(_IN4, OUTPUT);
    // stop motors
    _applyLR();
  }

  // — Left wheel property
  void  setLeft(float L)   { _left = L; _applyLR(); }
  float getLeft() const    { return _left; }

  // — Right wheel property
  void  setRight(float R)  { _right = R; _applyLR(); }
  float getRight() const   { return _right; }

  // — Throttle property (recomputes left/right via unclamped mix)
  void  setThrottle(float t) {
    t = constrain(t, -1.0f, 1.0f);
    float s = getSteering();
    diffMix(t, s, _left, _right);
    _applyLR();
  }
  float getThrottle() const {
    float t, s;
    diffUnmix(_left, _right, t, s);
    return t;
  }

  // — Steering property (recomputes left/right via unclamped mix)
  void  setSteering(float s) {
    s = constrain(s, -1.0f, 1.0f);
    float t = getThrottle();
    diffMix(t, s, _left, _right);
    _applyLR();
  }
  float getSteering() const {
    float t, s;
    diffUnmix(_left, _right, t, s);
    return s;
  }

  // — Convenience method to stop both wheels
  void stop() {
    _left = 0.0f;
    _right = 0.0f;
    _applyLR();
  }

  // — Trim property (motor speed compensation)
  void  setTrim(float trim) {
    _trim = constrain(trim, -1.0f, 1.0f);
    _applyLR();  // Re-apply with new trim
  }
  float getTrim() const { return _trim; }

private:
  uint8_t _ENA, _IN1, _IN2, _ENB, _IN3, _IN4;
  float   _left, _right;
  float   _trim;  // Motor speed compensation: -1.0 (slow left) to +1.0 (slow right)

  // apply _left/_right to the hardware, clamping only at write time
  void _applyLR() {
    // Apply trim compensation before sending to motors
    float leftAdjusted = _left * (_trim < 0 ? (1.0f + _trim) : 1.0f);
    float rightAdjusted = _right * (_trim > 0 ? (1.0f - _trim) : 1.0f);
    
    _driveMotor(_ENA, _IN1, _IN2, leftAdjusted);
    _driveMotor(_ENB, _IN3, _IN4, rightAdjusted);
  }

  // drive one motor: clamp v∈[-1,1] here, then map to PWM
  static void _driveMotor(uint8_t pwmPin, uint8_t inA, uint8_t inB, float v) {
    float vc = constrain(v, -1.0f, 1.0f);
    int   duty = (int)(fabs(vc) * 255.0f);
    digitalWrite(inA, vc >= 0 ? HIGH : LOW);
    digitalWrite(inB, vc <= 0 ? HIGH : LOW);
    analogWrite(pwmPin, duty);
  }
};
