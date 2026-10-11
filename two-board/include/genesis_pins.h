// genesis_pins.h - GPIO numbers for the Axiometa Genesis Mini.
//
// Copied from Arduino-ESP32: variants/axiometa_genesis_mini/pins_arduino.h
// PlatformIO has no Genesis Mini board, so the build uses a generic ESP32-S3 board and the port
// names (P1_IO0 ...) are defined here. If you build with the real Genesis variant (Arduino IDE
// or a future PlatformIO board), that variant already defines them and this block is skipped.
#pragma once
#include <Arduino.h>

#ifndef USER_BUTTON   // only the real Genesis variant defines this

// Port 1
static const uint8_t P1_IO0 = 4;
static const uint8_t P1_IO1 = 3;
static const uint8_t P1_IO2 = 2;
// Port 2
static const uint8_t P2_IO0 = 7;
static const uint8_t P2_IO1 = 6;
static const uint8_t P2_IO2 = 5;
// Port 3
static const uint8_t P3_IO0 = 9;
static const uint8_t P3_IO1 = 16;
static const uint8_t P3_IO2 = 15;
// Port 4
static const uint8_t P4_IO0 = 1;
static const uint8_t P4_IO1 = 17;
static const uint8_t P4_IO2 = 18;

#endif

// Other Genesis Mini pins. Named differently from the Arduino SDA/SCL/MOSI/... constants because a
// generic ESP32-S3 board defines those with different numbers.
namespace genesis {
constexpr uint8_t I2C_SDA = 10;
constexpr uint8_t I2C_SCL = 11;
constexpr uint8_t SPI_MOSI = 12;
constexpr uint8_t SPI_MISO = 13;
constexpr uint8_t SPI_SCK = 14;
constexpr uint8_t RGB_LED_PIN = 21;
constexpr uint8_t USER_BUTTON_PIN = 45;
}  // namespace genesis
