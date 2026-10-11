// Handheld controller: your Studio Axiometa sketch.ino, logic unchanged.
// Modules: P1 encoder (AX22-0003), P2 stop button (AX22-0050), P3 buzzer (AX22-0018), P4 microphone (AX22-0044).
// Sends motor commands to the receiver over ESP-NOW every 100 ms. The host (the iPhone app over
// Bluetooth, or the PC "Studio" panel over USB serial) sends PING / RECORD / VOICE:<epoch>:<word> / STOP,
// plus CAM:CLEAR / CAM:BLOCK / CAM:OFF from a camera and NAV:<cue> from a route guide.
//
// Changes for PlatformIO: this file is .cpp, so it includes <Arduino.h> and "genesis_pins.h"
// (the P1_IO0 ... names Studio Axiometa's board package provides).
#include <Arduino.h>
#include "genesis_pins.h"
#include <RotaryEncoder.h>
#include "rover_radio.h"
#include "ble_link.h"
#include "voice_capture.h"
#include "nav_cues.h"

// Intelligent disobedience: refuse to drive forward unless a camera reported a clear path in the
// last 2 s, and stop if it sees an obstacle or goes quiet. Build with -DREQUIRE_CAMERA=0 to bench-test
// without a camera.
#ifndef REQUIRE_CAMERA
#define REQUIRE_CAMERA 1
#endif
const unsigned long CAMERA_TIMEOUT_MS = 2000;
bool cameraReady = false, cameraBlocked = true;
unsigned long lastCameraFrame = 0;

enum Direction { STOPPED, FORWARD, REVERSE, LEFT, RIGHT };
RotaryEncoder encoder(P1_IO1, P1_IO2, RotaryEncoder::LatchMode::TWO03);
Direction direction = STOPPED;
// Motor power is PWM 0..255. Drive commands ramp from RAMP_START up to topSpeed, which the encoder sets.
const int RAMP_START = 120;
const unsigned long RAMP_MS = 2000;
int topSpeed = 220;
int speedValue = topSpeed;
int rampFrom = RAMP_START, rampTo = 220;
unsigned long rampDuration = RAMP_MS;
// Going around obstacles and rejoining the path. When the camera reports the centre blocked but one
// side free (CAM:AVOID:LEFT / CAM:AVOID:RIGHT) while driving forward:
//   TURN_OUT  turn toward the free side until the centre is clear (+AVOID_EXTRA_MS); its length is T
//   PASS      forward for AVOID_PASS_MS, alongside the obstacle
//   TURN_BACK turn the other way for 2T, now pointing back toward the original line
//   RETURN    forward for AVOID_PASS_MS, back onto the line
//   ALIGN     turn the first way for T, facing the original heading again, then carry on forward
// The two sideways legs mirror each other, so the rover ends up on the same line. Timing based
// (no wheel sensors), so approximate: tune AVOID_PASS_MS and the speed on the real floor.
enum AvoidPhase { AVOID_NONE, TURN_OUT, PASS, TURN_BACK, RETURN_TO_LINE, ALIGN };
const unsigned long AVOID_EXTRA_MS = 300;   // keep turning this long after the centre clears
const unsigned long AVOID_MAX_MS = 4000;    // about half a turn; give up and stop after this
const unsigned long AVOID_PASS_MS = 1500;   // each forward leg of the detour
const int AVOID_SPEED = 170;                // forward legs of the detour
// Turning on the spot makes the wheels skid sideways, which needs much more power than driving
// forward: below full power the motors strain and the rover turns too little. All turns use this.
const int TURN_SPEED = 255;
AvoidPhase avoidPhase = AVOID_NONE;
Direction avoidSide = LEFT;
unsigned long phaseStarted = 0, avoidClearSince = 0, turnOutMs = 0;
bool avoiding = false;  // avoidPhase != AVOID_NONE

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
// Obstacle stops slow the rover down over this time (with three warning beeps) instead of cutting power.
// Every other stop (button, voice, heartbeat, camera off) is still immediate.
const unsigned long OBSTACLE_DECEL_MS = 900;
bool decelerating = false;
unsigned long decelStarted = 0;
int decelFromSpeed = 0;
const char *decelReason = "Camera obstacle";

void reportStatus() {
  const char *names[] = {"stop", "forward", "reverse", "left", "right"};
  reportf("STATE:%lu,%d,%s,%d", (unsigned long)epoch, speedValue, names[direction], digitalRead(P2_IO1) == LOW);
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
    reportf("CAL:Ended after %lu ms: %s", millis() - calibrationStarted, reason);
    calibrating = false;
  }
  ramping = false;
  avoiding = false;
  avoidPhase = AVOID_NONE;
  decelerating = false;
  speedValue = topSpeed;
  direction = STOPPED; ++epoch;
  cancelPattern();
  digitalWrite(P2_IO2, HIGH);
  drivePacket();
  tone(P3_IO1, 600, 120);
  reportf("STOP:%s", reason);
  reportStatus();
}
void obstacleStop(const char *reason) {
  if (decelerating) return;
  decelReason = reason;
  avoidPhase = AVOID_NONE;  // an obstacle ends any detour in progress
  avoiding = false;
  ramping = false;
  decelerating = true; decelStarted = millis(); decelFromSpeed = speedValue;
  playObstacleAlert();
  report("STOP:Camera obstacle, slowing down");
}
void onButtonPress() {
  if (!hostAlive || millis() - lastHost > 1200 || digitalRead(P2_IO1) == LOW) {
    report("ERROR:Release stop and connect the iPhone app (or Studio) before starting");
    return;
  }
  if (calibrating) return;
  if (captureBusy) return;  // a clip is still being recorded or sent
  report("LISTEN:Encoder pressed");
}
void startRamp() {
  rampFrom = min(RAMP_START, topSpeed); rampTo = topSpeed; rampDuration = RAMP_MS;
  speedValue = rampFrom;
  rampStarted = millis();
  ramping = true;
}
const char *sideName(Direction side) { return side == LEFT ? "LEFT" : "RIGHT"; }
Direction opposite(Direction side) { return side == LEFT ? RIGHT : LEFT; }
void enterPhase(AvoidPhase phase, Direction move) {
  avoidPhase = phase;
  avoiding = true;
  phaseStarted = millis();
  direction = move;
  ramping = false;
  speedValue = move == FORWARD ? min(AVOID_SPEED, topSpeed) : TURN_SPEED;
  drivePacket();
  reportStatus();
}
void startAvoid(Direction side) {
  avoidSide = side;
  avoidClearSince = 0;
  playNavCue(sideName(side));
  reportf("AVOID:%s", sideName(side));
  enterPhase(TURN_OUT, side);
}
void finishAvoid() {
  avoidPhase = AVOID_NONE;
  avoiding = false;
  direction = FORWARD;
  startRamp();
  playNavCue("STRAIGHT");
  drivePacket();
  report("AVOID:Done");
  reportStatus();
}
bool cameraClear() {
  return !REQUIRE_CAMERA || (cameraReady && !cameraBlocked);
}
void refuse(const char *reason) {
  cancelPattern();
  tone(P3_IO1, 600, 300);
  reportf("REFUSED:%s", reason);
}
void processCommand(const String &line, bool fromBle = false) {
  if (line == "PING") { lastHost = millis(); hostAlive = true; return; }
  if (line == "STATUS") { reportStatus(); return; }
  if (line == "STOP") { stopRover("Host stop"); return; }
  if (line == "CAM:CLEAR" || line == "CAM:BLOCK" || line.startsWith("CAM:AVOID:")) {
    cameraReady = true;
    cameraBlocked = line != "CAM:CLEAR";
    lastCameraFrame = millis();
    bool clear = line == "CAM:CLEAR";
    if (decelerating) {
      // Already slowing down for an obstacle: nothing to decide until it has stopped.
    } else if (avoidPhase == TURN_OUT) {
      // Turning away: wait for a clear centre. Both sides blocked means there's no way around.
      if (clear) { if (!avoidClearSince) avoidClearSince = millis(); }
      else if (line == "CAM:BLOCK") { if (REQUIRE_CAMERA) stopRover("No way around the obstacle"); }
      else avoidClearSince = 0;
    } else if (avoidPhase == PASS || avoidPhase == RETURN_TO_LINE) {
      // Driving a detour leg: anything ahead now is a second obstacle, so slow down and stop.
      if (!clear && REQUIRE_CAMERA) obstacleStop("Obstacle while going around");
    } else if (avoidPhase == TURN_BACK || avoidPhase == ALIGN) {
      // Turning back toward the line sweeps the camera across the obstacle we are passing: ignore.
    } else if (direction == FORWARD && !clear) {
      if (line == "CAM:BLOCK") { if (REQUIRE_CAMERA) obstacleStop("Camera obstacle"); }
      else startAvoid(line == "CAM:AVOID:LEFT" ? LEFT : RIGHT);
    }
    return;
  }
  if (line == "CAM:OFF") {
    cameraReady = false;
    if (REQUIRE_CAMERA && (direction == FORWARD || avoiding)) stopRover("Camera off");
    return;
  }
  if (line.startsWith("NAV:")) {
    // Guide only while moving; a stopped rover must not suggest walking.
    if (direction == STOPPED) { report("NAV:Ignored, stopped"); return; }
    if (playNavCue(line.substring(4))) reportf("NAV:%s", line.c_str() + 4);
    else report("ERROR:Unknown navigation cue");
    return;
  }
  if (line.startsWith("CAL:")) {
    if (!hostAlive || millis()-lastHost > 1200 || digitalRead(P2_IO1) == LOW || direction != STOPPED) {
      report("ERROR:Calibration requires connected Studio, released P2, and stopped rover"); return;
    }
    int split = line.indexOf(':', 4);
    if (split < 0) return;
    String move = line.substring(4, split);
    const char *text = line.c_str() + split + 1;
    char *end;
    long duration = strtol(text, &end, 10);
    if (end == text || *end || duration < 100 || duration > 10000 ||
        (move != "forward" && move != "right" && move != "left")) {
      report("ERROR:Invalid calibration (100 to 10000 ms, forward/left/right)"); return;
    }
    // Turns run 50% longer than requested (capped at the 10 s limit); forward runs are unchanged.
    if (move != "forward") duration = min(duration * 3 / 2, 10000L);
    ++epoch;
    direction = move == "forward" ? FORWARD : (move == "right" ? RIGHT : LEFT);
    speedValue = 80; rampFrom = 80; rampTo = 160; rampDuration = 4000;
    rampStarted = millis(); ramping = true;
    calibrating = true; calibrationStarted = rampStarted; calibrationDuration = duration;
    digitalWrite(P2_IO2, LOW);
    reportf("CAL:Started %s for %ld ms, ramp 80 to 160 over 4 s", move.c_str(), duration);
    drivePacket(); reportStatus(); return;
  }
  if (calibrating && (line == "RECORD" || line.startsWith("VOICE:"))) {
    report("ERROR:Stop calibration before using voice"); return;
  }
  if (line == "RECORD") {
    if (!hostAlive || digitalRead(P2_IO1) == LOW) { report("ERROR:Release stop and connect the iPhone app (or Studio)"); return; }
    requestVoice(epoch, fromBle); return;
  }
  if (!line.startsWith("VOICE:")) return;
  int separator = line.indexOf(':', 6);
  if (separator < 0) return;
  uint32_t requestedEpoch = strtoul(line.substring(6, separator).c_str(), nullptr, 10);
  String word = line.substring(separator+1);
  if (word == "stop") { stopRover("Voice stop"); return; }
  if (requestedEpoch != epoch || !hostAlive || digitalRead(P2_IO1) == LOW) {
    report("ERROR:Stale voice command or stop held"); return;
  }
  if (word == "forward" && !cameraClear()) { refuse("Camera has not reported a clear path"); return; }
  avoiding = false;  // a spoken command takes over from going around
  avoidPhase = AVOID_NONE;
  if (word == "forward") direction = FORWARD;
  else if (word == "reverse") direction = REVERSE;
  else if (word == "left") direction = LEFT;
  else if (word == "right") direction = RIGHT;
  else { report("ERROR:Unknown voice command"); return; }
  decelerating = false;
  if (direction == LEFT || direction == RIGHT) {
    ramping = false;
    speedValue = TURN_SPEED;  // turns start at full power, no ramp
  } else {
    startRamp();
  }
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
  navBuzzerPin = P3_IO1;
  beginRadio(); initBle(); beginVoice();
  report("READY:Rover controller; encoder=speed/click to listen, P2=stop, host=iPhone (Bluetooth \"Robodog\") or Studio (USB)");
  reportStatus(); drivePacket();
}
void loop() {
  encoder.tick();
  long position = encoder.getPosition();
  if (position != lastPosition && decelerating) lastPosition = position;  // ignore the dial while stopping
  if (position != lastPosition) {
    if (calibrating) stopRover("Encoder adjusted during calibration");
    ramping = false;
    topSpeed = constrain(topSpeed + (position-lastPosition)*5, 0L, 255L);
    speedValue = topSpeed;
    lastPosition = position;
    tone(P3_IO1, 1000, 15); reportStatus();
  }
  bool stopPressed = digitalRead(P2_IO1) == LOW;
  if (stopPressed && !previousStop) stopRover("Physical stop");
  previousStop = stopPressed;
  unsigned long now = millis();
  if (hostAlive && now-lastHost > 1200) {
    hostAlive = false;
    // Only a moving rover needs stopping. Stopping an already stopped one would bump the epoch
    // and reject a voice command that is still on its way (seen when PINGs lag during audio).
    if (direction != STOPPED) stopRover("Host heartbeat lost");
    else report("HOST:Heartbeat lost");
  }
  if (cameraReady && now-lastCameraFrame > CAMERA_TIMEOUT_MS) {
    cameraReady = false;
    if (REQUIRE_CAMERA && (direction == FORWARD || avoiding)) stopRover("Camera timeout");
  }
  unsigned long inPhase = now - phaseStarted;
  switch (avoidPhase) {
    case TURN_OUT:
      if (avoidClearSince && now - avoidClearSince >= AVOID_EXTRA_MS) {
        turnOutMs = inPhase;
        report("AVOID:Passing");
        enterPhase(PASS, FORWARD);
      } else if (inPhase > AVOID_MAX_MS) {
        stopRover("Could not find a way around");
      }
      break;
    case PASS:
      if (inPhase >= AVOID_PASS_MS) {
        playNavCue(sideName(opposite(avoidSide)));
        report("AVOID:Rejoining");
        enterPhase(TURN_BACK, opposite(avoidSide));
      }
      break;
    case TURN_BACK:
      if (inPhase >= 2 * turnOutMs) enterPhase(RETURN_TO_LINE, FORWARD);
      break;
    case RETURN_TO_LINE:
      if (inPhase >= AVOID_PASS_MS) enterPhase(ALIGN, avoidSide);
      break;
    case ALIGN:
      if (inPhase >= turnOutMs) finishAvoid();
      break;
    case AVOID_NONE:
      break;
  }
  String bleCommand;
  while (nextBleCommand(bleCommand)) processCommand(bleCommand, true);
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
    if (elapsed >= rampDuration) { speedValue = rampTo; ramping = false; }
    else speedValue = rampFrom + (long)(elapsed * (rampTo - rampFrom)) / (long)rampDuration;
  }
  if (decelerating) {
    unsigned long elapsed = millis() - decelStarted;
    if (elapsed >= OBSTACLE_DECEL_MS) stopRover(decelReason);
    else speedValue = decelFromSpeed - (int)((unsigned long)decelFromSpeed * elapsed / OBSTACLE_DECEL_MS);
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
  servicePattern();
  delay(1);
}
