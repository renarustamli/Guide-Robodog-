// pins.h - every GPIO the project uses, in one place, by Genesis Mini port name.
// Change a pin here and both builds follow. The static_assert at the bottom stops two functions
// being given the same GPIO.
#pragma once
#include "genesis_pins.h"

namespace pins {

// ---- Wired for the control-test hardware (taken from your control_test sketch) --------------
constexpr uint8_t CTRL_BTN = P1_IO0;   // encoder module push button (reads HIGH when pressed)
constexpr uint8_t ENC_CLK  = P1_IO1;   // encoder module CLK
constexpr uint8_t ENC_DT   = P1_IO2;   // encoder module DT
constexpr uint8_t STOP_BTN = P2_IO1;   // stop button (reads LOW when pressed)
constexpr uint8_t STOP_LED = P2_IO2;   // light in the stop button (HIGH = stopped)
constexpr uint8_t BUZZER   = P3_IO1;   // AX22-0018 buzzer

// ---- Rover firmware (src/main.cpp, src/motors.h) ---------------------------------------------
// PROPOSED pins, using what the control-test wiring leaves free. Change them to match your wiring.
// Microphone: taken from your voice_capture.h (P4 port).
constexpr uint8_t MIC_SELECT = P4_IO0;  // driven LOW: T3902 SELECT (LOW = left slot)
constexpr uint8_t MIC_DATA   = P4_IO1;  // T3902 data
constexpr uint8_t MIC_CLK    = P4_IO2;  // T3902 clock

// Motors and sensors: PROPOSED, using what is left. Change them to match your wiring.
constexpr uint8_t MOTOR_L_FWD = P2_IO0;
constexpr uint8_t MOTOR_L_REV = P3_IO0;
constexpr uint8_t MOTOR_R_FWD = P3_IO2;
constexpr uint8_t MOTOR_R_REV = genesis::SPI_SCK;

// Single-channel wheel encoders (one pin each), because the board runs out of pins.
// Direction is taken from the sign of the motor command, so a wheel pushed backwards by hand
// or coasting through a reversal is counted wrongly. Fine for a first test.
constexpr uint8_t ENC_L = genesis::I2C_SDA;
constexpr uint8_t ENC_R = genesis::I2C_SCL;

constexpr uint8_t DEADMAN = genesis::SPI_MOSI;   // handle switch to GND (LOW = held)
// genesis::SPI_MISO (13) is still free.

// The rover firmware reuses the control-test buttons: STOP_BTN is the e-stop and CTRL_BTN is the
// "safe to cross" confirm button.

// ---- Duplicate check ---------------------------------------------------------------------------
constexpr uint8_t kAll[] = {CTRL_BTN, ENC_CLK, ENC_DT, STOP_BTN, STOP_LED, BUZZER,
                            MIC_SELECT, MIC_CLK, MIC_DATA,
                            MOTOR_L_FWD, MOTOR_L_REV, MOTOR_R_FWD, MOTOR_R_REV,
                            ENC_L, ENC_R, DEADMAN};
constexpr bool allDistinct() {
  for (unsigned i = 0; i < sizeof(kAll); i++)
    for (unsigned j = i + 1; j < sizeof(kAll); j++)
      if (kAll[i] == kAll[j]) return false;
  return true;
}
static_assert(allDistinct(), "pins.h: two functions share the same GPIO");

}  // namespace pins
