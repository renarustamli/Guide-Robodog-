// Rover receiver (motor board): your Studio Axiometa sketch.ino, logic unchanged.
// Motors: two AX22-0032 motor pairs, left on P4/P3, right on P1/P2. Commands arrive over ESP-NOW from the
// handheld (see include/rover_radio.h) or as "SET:left,right" over USB until the radio takes over.
//
// Changes for PlatformIO: this file is .cpp, so it includes <Arduino.h> and "genesis_pins.h"
// (the P2_IO1 ... names Studio Axiometa's board package provides).
#include <Arduino.h>
#include "genesis_pins.h"
#include "rover_radio.h"

struct Motor {
  uint8_t a1, a2, frontA1, frontA2;
  int requested, applied;
  unsigned long offAt;
};
// P3 and P4 pins swapped (IO2 before IO1) to reverse both left motors.
Motor leftMotor = {P3_IO2, P3_IO1, P4_IO2, P4_IO1, 0, 0, 0};
Motor rightMotor = {P2_IO2, P2_IO1, P1_IO2, P1_IO1, 0, 0, 0};
unsigned long lastCommand = 0;
unsigned long lastReport = 0;
unsigned long lastRadioIdentity = 0;
String input;
bool overflow = false;
bool wirelessControl = false;
bool calibration = false;
bool calibrationStarted = false;
unsigned long calibrationDuration = 0, calibrationStart = 0;

void report() {
  if (wirelessControl) Serial.println("RADIO:Handheld owns control until rover restarts; USB driving disabled.");
  Serial.printf("ROVER:%d,%d\n", leftMotor.applied, rightMotor.applied);
}

void coast(Motor &motor) {
  analogWrite(motor.a1, 0);
  analogWrite(motor.a2, 0);
  analogWrite(motor.frontA1, 0);
  analogWrite(motor.frontA2, 0);
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
    analogWrite(motor.frontA2, 0);
    analogWrite(motor.a1, motor.requested);
    analogWrite(motor.frontA1, motor.requested);
  } else {
    analogWrite(motor.a1, 0);
    analogWrite(motor.frontA1, 0);
    analogWrite(motor.a2, -motor.requested);
    analogWrite(motor.frontA2, -motor.requested);
  }
  motor.applied = motor.requested;
  report();
}

void command(const String &line) {
  if (wirelessControl) return;
  if (line == "CALPING" && calibration) { lastCommand = millis(); return; }
  if (line.startsWith("CAL:")) {
    int left, right;
    unsigned long duration;
    char extra;
    if (sscanf(line.c_str(), "CAL:%d,%d,%lu%c", &left, &right, &duration, &extra) != 3 ||
        abs(left) > 255 || abs(right) > 255 || duration < 100 || duration > 10000 ||
        (left == 0 && right == 0)) {
      Serial.println("ERROR:Invalid calibration command"); return;
    }
    if (calibration || leftMotor.applied != 0 || rightMotor.applied != 0) {
      Serial.println("ERROR:Stop before starting calibration"); return;
    }
    calibration = true;
    calibrationStarted = false;
    calibrationDuration = duration;
    requestMotor(leftMotor, left);
    requestMotor(rightMotor, right);
    lastCommand = millis();
    Serial.println("CAL:Accepted");
    return;
  }
  if (!line.startsWith("SET:")) return;
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
  if (calibration) {
    if (left != 0 || right != 0) return;
    calibration = false;
    Serial.println("CAL:Cancelled");
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
  pinMode(leftMotor.frontA1, OUTPUT);
  pinMode(leftMotor.frontA2, OUTPUT);
  pinMode(rightMotor.frontA1, OUTPUT);
  pinMode(rightMotor.frontA2, OUTPUT);
  coast(leftMotor);
  coast(rightMotor);
  Serial.begin(115200);
  input.reserve(40);
  Serial.println("Rover ready: AX22-0032 left P4/P3, right P1/P2. Stopped.");
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
    if (calibration) { calibration = false; Serial.println("CAL:Cancelled by handheld"); }
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
  if ((leftMotor.requested != 0 || rightMotor.requested != 0) && millis() - lastCommand > 1500) {
    requestMotor(leftMotor, 0);
    requestMotor(rightMotor, 0);
    if (calibration) { calibration = false; Serial.println("CAL:Heartbeat lost"); }
    Serial.println("Rover stopped: control heartbeat lost.");
    report();
  }
  updateMotor(leftMotor);
  updateMotor(rightMotor);
  if (calibration) {
    if (!calibrationStarted && leftMotor.applied == leftMotor.requested && rightMotor.applied == rightMotor.requested) {
      calibrationStarted = true;
      calibrationStart = millis();
      Serial.println("CAL:Running");
    }
    if (calibrationStarted && millis() - calibrationStart >= calibrationDuration) {
      requestMotor(leftMotor, 0);
      requestMotor(rightMotor, 0);
      calibration = false;
      Serial.println("CAL:Done");
      report();
    }
  }
  if (millis() - lastReport >= 500) {
    lastReport = millis();
    report();
  }
  if (millis() - lastRadioIdentity >= 5000) {
    lastRadioIdentity = millis();
    Serial.printf("RADIO:Station MAC %s\n", WiFi.macAddress().c_str());
  }
  delay(1);
}
