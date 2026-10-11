// motors.h - differential-drive hardware layer for the rover (ESP32 Arduino core 3.x).
//
//   * two H-bridge inputs per motor with LEDC PWM (works with DRV8833, DRV8871, TB6612 wired
//     as "PWM on one input, other input low"; adapt setPwm() if your driver differs)
//   * quadrature encoders read by interrupt
//   * per-wheel speed PID with a feed-forward term
//   * hardware-timer failsafe: if drive()/stop() is not called for FAILSAFE_MS, PWM goes to zero
//
// Every pin, size and gain below is a PLACEHOLDER: set them for your chassis, then tune the PID
// with the wheels off the ground.
#pragma once
#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "follower.h"
#include "pins.h"

namespace motors {

// ---- pins (set in include/pins.h)
constexpr int PIN_L_FWD = pins::MOTOR_L_FWD, PIN_L_REV = pins::MOTOR_L_REV;
constexpr int PIN_R_FWD = pins::MOTOR_R_FWD, PIN_R_REV = pins::MOTOR_R_REV;
constexpr int PIN_L_ENC = pins::ENC_L, PIN_R_ENC = pins::ENC_R;   // single channel each

// ---- mechanics (TODO: measure)
constexpr float WHEEL_DIAMETER_M = 0.065f;
constexpr float TICKS_PER_WHEEL_REV = 660.0f;       // edges (rising and falling) of the one channel per wheel turn
constexpr float METRES_PER_TICK = 3.14159265f * WHEEL_DIAMETER_M / TICKS_PER_WHEEL_REV;
constexpr float V_AT_FULL_PWM = 1.2f;               // m/s at 100% duty, for feed-forward
constexpr float V_WHEEL_MAX = 1.1f;                 // m/s cap on any wheel
constexpr bool INVERT_L = false, INVERT_R = false;  // flip if a wheel drives backwards

// ---- PWM and failsafe
constexpr int PWM_FREQ = 20000;
constexpr int PWM_BITS = 10;
constexpr uint32_t FAILSAFE_MS = 300;

// One pin per wheel, so the encoder cannot see direction. The sign of the last motor command
// (+1 forward, -1 reverse) is used instead.
struct Encoder {
  volatile int8_t dir = 1;
  volatile int32_t count = 0;
  int32_t last = 0;
};

static void IRAM_ATTR encoderIsr(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  e->count += e->dir;
}

class Drive {
 public:
  void begin() {
    pinMode(PIN_L_ENC, INPUT_PULLUP);
    pinMode(PIN_R_ENC, INPUT_PULLUP);
    attachInterruptArg(PIN_L_ENC, encoderIsr, &encL_, CHANGE);
    attachInterruptArg(PIN_R_ENC, encoderIsr, &encR_, CHANGE);

    ledcAttach(PIN_L_FWD, PWM_FREQ, PWM_BITS); ledcAttach(PIN_L_REV, PWM_FREQ, PWM_BITS);
    ledcAttach(PIN_R_FWD, PWM_FREQ, PWM_BITS); ledcAttach(PIN_R_REV, PWM_FREQ, PWM_BITS);
    writeZero();

    pidL_ = makePid(); pidR_ = makePid();
    touch();
    esp_timer_create_args_t args = {};
    args.callback = &Drive::failsafeCb;
    args.arg = this;
    args.name = "motor_failsafe";
    esp_timer_handle_t h;
    if (esp_timer_create(&args, &h) == ESP_OK) esp_timer_start_periodic(h, 50 * 1000);
  }

  // Metres each wheel has moved since the last call (positive = forward). Also updates the
  // measured wheel speeds used by the PID. Call once per control cycle.
  void readEncoders(float& dL, float& dR, float dt) {
    int32_t cl = encL_.count, cr = encR_.count;
    dL = (cl - encL_.last) * METRES_PER_TICK;
    dR = (cr - encR_.last) * METRES_PER_TICK;
    encL_.last = cl; encR_.last = cr;
    vL_ = dL / dt; vR_ = dR / dt;
  }

  // Body command -> wheel targets -> PID + feed-forward -> PWM.
  void drive(float v, float w, float dt) {
    touch();
    if (v == 0.0f && w == 0.0f) { halt(); return; }
    float tl, tr;
    rover::wheelSpeeds(v, w, TRACK_M, V_WHEEL_MAX, tl, tr);
    float pl = tl / V_AT_FULL_PWM + pidL_.step(tl - vL_, dt);
    float pr = tr / V_AT_FULL_PWM + pidR_.step(tr - vR_, dt);
    if (INVERT_L) pl = -pl;
    if (INVERT_R) pr = -pr;
    // Direction as the wheel really turns (before any wiring inversion): follows the target.
    if (tl != 0.0f) encL_.dir = tl > 0 ? 1 : -1;
    if (tr != 0.0f) encR_.dir = tr > 0 ? 1 : -1;
    setPwm(PIN_L_FWD, PIN_L_REV, pl);
    setPwm(PIN_R_FWD, PIN_R_REV, pr);
  }

  void stop() { touch(); halt(); }

  static constexpr float TRACK_M = 0.30f;   // keep equal to rover::Params::trackWidth

 private:
  static rover::PID makePid() {
    rover::PID p;
    p.kp = 0.8f; p.ki = 2.5f; p.iLimit = 0.4f; p.outMin = -0.5f; p.outMax = 0.5f;   // TODO tune
    return p;
  }
  void halt() { pidL_.reset(); pidR_.reset(); writeZero(); }
  void touch() { lastCmdMs_ = millis(); }

  static void setPwm(int fwdPin, int revPin, float x) {
    x = rover::clampf(x, -1.0f, 1.0f);
    uint32_t duty = (uint32_t)(fabsf(x) * ((1u << PWM_BITS) - 1));
    if (x >= 0) { ledcWrite(fwdPin, duty); ledcWrite(revPin, 0); }
    else        { ledcWrite(fwdPin, 0);    ledcWrite(revPin, duty); }
  }
  static void writeZero() {
    ledcWrite(PIN_L_FWD, 0); ledcWrite(PIN_L_REV, 0);
    ledcWrite(PIN_R_FWD, 0); ledcWrite(PIN_R_REV, 0);
  }
  static void failsafeCb(void* arg) {
    Drive* d = static_cast<Drive*>(arg);
    if (millis() - d->lastCmdMs_ > FAILSAFE_MS) writeZero();
  }

  Encoder encL_{}, encR_{};
  rover::PID pidL_, pidR_;
  float vL_ = 0, vR_ = 0;
  volatile uint32_t lastCmdMs_ = 0;
};

}  // namespace motors
