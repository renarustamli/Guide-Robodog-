// Handheld controller: your Studio Axiometa sketch.ino, logic unchanged.
// Modules: P1 encoder (AX22-0003), P2 stop button (AX22-0050), P3 buzzer (AX22-0018), P4 microphone (AX22-0044).
// Sends motor commands to the receiver over ESP-NOW every 100 ms; the PC "Studio" app sends
// PING / RECORD / VOICE:<epoch>:<word> / STOP over USB serial.
//
// Changes for PlatformIO: this file is .cpp, so it includes <Arduino.h> and "genesis_pins.h"
// (the P1_IO0 ... names Studio Axiometa's board package provides).
#include <Arduino.h>
#include "genesis_pins.h"
#include <RotaryEncoder.h>
#include "rover_radio.h"
#include "voice_capture.h"

enum Direction { STOPPED, FORWARD, REVERSE, LEFT, RIGHT };
RotaryEncoder encoder(P1_IO1, P1_IO2, RotaryEncoder::LatchMode::TWO03);
Direction direction = STOPPED;
int speedValue = 160;
long lastPosition = 0;
uint32_t epoch = 1;
unsigned long lastHost = 0, lastHeartbeat = 0, lastStatus = 0;
bool hostAlive = false, previousStop = false;
bool lastBtnState = LOW;
unsigned long lastBtnTime = 0;
bool ramping = false;
unsigned long rampStarted = 0;
String commandLine;
bool calibrating = false;
unsigned long calibrationStarted = 0, calibrationDuration = 0;

void reportStatus() {
  const char *names[] = {"stop", "forward", "reverse", "left", "right"};
  Serial.printf("STATE:%lu,%d,%s,%d\n", (unsigned long)epoch, speedValue, names[direction], digitalRead(P2_IO1) == LOW);
}
void drivePacket() {
  int16_t left = 0, right = 0;
  if (direction == FORWARD) left = right = speedValue;
  if (direction == REVERSE) left = right = -speedValue;
  if (direction == LEFT) { left = -speedValue; right = speedValue; }
  if (direction == RIGHT) { left = speedValue; right = -speedValue; }
  sendRover(left, right);
}
void stopRover(const char *reason) {
  if (calibrating) {
    Serial.printf("CAL:Ended after %lu ms: %s\n", millis() - calibrationStarted, reason);
    calibrating = false;
  }
  ramping = false;
  direction = STOPPED; ++epoch;
  digitalWrite(P2_IO2, HIGH);
  drivePacket();
  tone(P3_IO1, 600, 120);
  Serial.printf("STOP:%s\n", reason);
  reportStatus();
}
void onButtonPress() {
  if (!hostAlive || millis() - lastHost > 1200 || digitalRead(P2_IO1) == LOW) {
    Serial.println("ERROR:Release stop and connect voice panel before starting");
    return;
  }
  if (calibrating) return;
  Serial.println("LISTEN:Encoder pressed");
}
void processCommand(const String &line) {
  if (line == "PING") { lastHost = millis(); hostAlive = true; return; }
  if (line == "STATUS") { reportStatus(); return; }
  if (line == "STOP") { stopRover("Studio stop"); return; }
  if (line.startsWith("CAL:")) {
    if (!hostAlive || millis()-lastHost > 1200 || digitalRead(P2_IO1) == LOW || direction != STOPPED) {
      Serial.println("ERROR:Calibration requires connected Studio, released P2, and stopped rover"); return;
    }
    int split = line.indexOf(':', 4);
    if (split < 0) return;
    String move = line.substring(4, split);
    const char *text = line.c_str() + split + 1;
    char *end;
    long duration = strtol(text, &end, 10);
    if (end == text || *end || duration < 100 || duration > 10000 ||
        (move != "forward" && move != "right" && move != "left")) {
      Serial.println("ERROR:Invalid calibration (100 to 10000 ms, forward/left/right)"); return;
    }
    ++epoch;
    direction = move == "forward" ? FORWARD : (move == "right" ? RIGHT : LEFT);
    speedValue = 80; rampStarted = millis(); ramping = true;
    calibrating = true; calibrationStarted = rampStarted; calibrationDuration = duration;
    digitalWrite(P2_IO2, LOW);
    Serial.printf("CAL:Started %s for %ld ms, ramp 80 to 160 over 4 s\n", move.c_str(), duration);
    drivePacket(); reportStatus(); return;
  }
  if (calibrating && (line == "RECORD" || line.startsWith("VOICE:"))) {
    Serial.println("ERROR:Stop calibration before using voice"); return;
  }
  if (line == "RECORD") {
    if (!hostAlive || digitalRead(P2_IO1) == LOW) { Serial.println("ERROR:Release stop and connect voice panel"); return; }
    requestVoice(epoch); return;
  }
  if (!line.startsWith("VOICE:")) return;
  int separator = line.indexOf(':', 6);
  if (separator < 0) return;
  uint32_t requestedEpoch = strtoul(line.substring(6, separator).c_str(), nullptr, 10);
  String word = line.substring(separator+1);
  if (word == "stop") { stopRover("Voice stop"); return; }
  if (requestedEpoch != epoch || !hostAlive || digitalRead(P2_IO1) == LOW) {
    Serial.println("ERROR:Stale voice command or stop held"); return;
  }
  if (word == "forward") direction = FORWARD;
  else if (word == "reverse") direction = REVERSE;
  else if (word == "left") direction = LEFT;
  else if (word == "right") direction = RIGHT;
  else { Serial.println("ERROR:Unknown voice command"); return; }
  speedValue = 80;
  rampStarted = millis();
  ramping = true;
  digitalWrite(P2_IO2, LOW);
  tone(P3_IO1, 1400, 80);
  drivePacket(); reportStatus();
}
void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  pinMode(P1_IO0, INPUT);
  pinMode(P2_IO1, INPUT);
  pinMode(P2_IO2, OUTPUT); digitalWrite(P2_IO2, HIGH);
  pinMode(P3_IO1, OUTPUT);
  commandLine.reserve(100);
  beginRadio(); beginVoice();
  Serial.println("READY:Rover controller; encoder=speed/click to listen, P2=stop, Studio=voice capture");
  reportStatus(); drivePacket();
}
void loop() {
  encoder.tick();
  long position = encoder.getPosition();
  if (position != lastPosition) {
    if (calibrating) stopRover("Encoder adjusted during calibration");
    ramping = false;
    speedValue = constrain(speedValue + (position-lastPosition)*5, 0L, 255L);
    lastPosition = position;
    tone(P3_IO1, 1000, 15); reportStatus();
  }
  bool stopPressed = digitalRead(P2_IO1) == LOW;
  if (stopPressed && !previousStop) stopRover("Physical stop");
  previousStop = stopPressed;
  unsigned long now = millis();
  if (hostAlive && now-lastHost > 1200) { hostAlive=false; stopRover("Studio heartbeat lost"); }
  for (int n=0; n<128 && Serial.available(); ++n) {
    char c=Serial.read();
    if (c=='\n') { processCommand(commandLine); commandLine=""; }
    else if (c!='\r') {
      if (commandLine.length()<100) commandLine+=c;
      else commandLine="";
    }
  }
  if (calibrating && millis()-calibrationStarted >= calibrationDuration) stopRover("Calibration complete");
  if (ramping) {
    unsigned long elapsed = millis() - rampStarted;
    if (elapsed >= 4000) { speedValue = 160; ramping = false; }
    else speedValue = 80 + (elapsed * 80UL) / 4000UL;
  }
  if (now-lastHeartbeat >= 100) { lastHeartbeat=now; drivePacket(); }
  if (now-lastStatus >= 500) { lastStatus=now; reportStatus(); }
  bool btn = digitalRead(P1_IO0);
  if (btn == HIGH && lastBtnState == LOW && (millis() - lastBtnTime > 150)) {
    onButtonPress();
    lastBtnTime = millis();
  }
  lastBtnState = btn;
  pollVoice();
  delay(1);
}
