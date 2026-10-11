// Rover receiver (motor board): your Studio Axiometa sketch.ino, logic unchanged.
// Motors: two AX22-0032 modules, left on P2, right on P3. Commands arrive over ESP-NOW from the
// handheld (see include/rover_radio.h) or as "SET:left,right" over USB until the radio takes over.
//
// Changes for PlatformIO: this file is .cpp, so it includes <Arduino.h> and "genesis_pins.h"
// (the P2_IO1 ... names Studio Axiometa's board package provides).
#include <Arduino.h>
#include "genesis_pins.h"
#include "rover_radio.h"

struct Motor {
  uint8_t a1, a2;
  int requested, applied;
  unsigned long offAt;
};
Motor leftMotor = {P2_IO1, P2_IO2, 0, 0, 0};
Motor rightMotor = {P3_IO1, P3_IO2, 0, 0, 0};
// Stop if no command arrives for this long. The handheld must send at least every ~100-200 ms.
const unsigned long HEARTBEAT_TIMEOUT_MS = 400;
unsigned long lastCommand = 0;
uint32_t lastRejectedReported = 0;
unsigned long lastReport = 0;
unsigned long lastRadioIdentity = 0;
String input;
bool overflow = false;
bool wirelessControl = false;

void report() {
  if (wirelessControl) Serial.println("RADIO:Handheld owns control until rover restarts; USB driving disabled.");
  Serial.printf("ROVER:%d,%d\n", leftMotor.applied, rightMotor.applied);
}

void coast(Motor &motor) {
  analogWrite(motor.a1, 0);
  analogWrite(motor.a2, 0);
  motor.applied = 0;
  motor.offAt = millis();
}

void requestMotor(Motor &motor, int value) {
  motor.requested = value;
  if (value == 0 || (motor.applied != 0 && ((value > 0) != (motor.applied > 0)))) coast(motor);
}

void updateMotor(Motor &motor) {
  if (motor.requested == motor.applied || motor.requested == 0) return;
  if (motor.applied == 0 && millis() - motor.offAt < 350) return;
  if (motor.requested > 0) {
    analogWrite(motor.a2, 0);
    analogWrite(motor.a1, motor.requested);
  } else {
    analogWrite(motor.a1, 0);
    analogWrite(motor.a2, -motor.requested);
  }
  motor.applied = motor.requested;
  report();
}

void command(const String &line) {
  if (!line.startsWith("SET:") || wirelessControl) return;
  const char *text = line.c_str() + 4;
  char *end;
  long left = strtol(text, &end, 10);
  if (end == text || *end != ',' || left < -255 || left > 255) {
    Serial.println("ERROR:Invalid rover command");
    return;
  }
  text = end + 1;
  long right = strtol(text, &end, 10);
  if (end == text || *end != '\0' || right < -255 || right > 255) {
    Serial.println("ERROR:Invalid motor command");
    return;
  }
  requestMotor(leftMotor, (int)left);
  requestMotor(rightMotor, (int)right);
  lastCommand = millis();
  report();
}

void setup() {
  pinMode(leftMotor.a1, OUTPUT);
  pinMode(leftMotor.a2, OUTPUT);
  pinMode(rightMotor.a1, OUTPUT);
  pinMode(rightMotor.a2, OUTPUT);
  coast(leftMotor);
  coast(rightMotor);
  Serial.begin(115200);
  input.reserve(40);
  Serial.println("Rover ready: AX22-0032 left P2, right P3. Stopped.");
  startRoverRadio();
  report();
}

void loop() {
  RoverPacket packet;
  if (readRoverRadio(packet)) {
    if (!wirelessControl) {
      wirelessControl = true;
      Serial.println("RADIO:Handheld owns control until rover restarts; USB driving disabled.");
    }
    requestMotor(leftMotor, packet.left);
    requestMotor(rightMotor, packet.right);
    lastCommand = millis();
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (!overflow) command(input);
      input = "";
      overflow = false;
    } else if (c != '\r') {
      if (input.length() < 40) input += c;
      else overflow = true;
    }
  }
  if ((leftMotor.requested != 0 || rightMotor.requested != 0) && millis() - lastCommand > HEARTBEAT_TIMEOUT_MS) {
    requestMotor(leftMotor, 0);
    requestMotor(rightMotor, 0);
    Serial.println("Rover stopped: control heartbeat lost.");
    report();
  }
  updateMotor(leftMotor);
  updateMotor(rightMotor);
  if (millis() - lastReport >= 500) {
    lastReport = millis();
    report();
  }
  if (millis() - lastRadioIdentity >= 5000) {
    lastRadioIdentity = millis();
    Serial.printf("RADIO:Station MAC %s\n", WiFi.macAddress().c_str());
    if (!senderAllowed(ROVER_ALLOWED_SENDER)) Serial.println("RADIO:No allowed sender set in rover_radio.h - radio commands are ignored.");
    if (radioRejected != lastRejectedReported) {
      lastRejectedReported = radioRejected;
      Serial.printf("RADIO:%lu packets ignored (wrong sender)\n", (unsigned long)radioRejected);
    }
  }
  delay(1);
}
