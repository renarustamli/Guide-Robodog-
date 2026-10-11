#include <RotaryEncoder.h>
#include "ble.h"

#define BTN_PIN P1_IO0
#define CLK_PIN P1_IO1
#define DT_PIN P1_IO2
#define STOP_PIN P2_IO1
#define STOP_LED_PIN P2_IO2
#define BUZZER_PIN P3_IO1

RotaryEncoder encoder(CLK_PIN, DT_PIN, RotaryEncoder::LatchMode::TWO03);
bool lastBtnState = LOW;
unsigned long lastBtnTime = 0;
long previousPosition = 0;
int speedPercent = 20;
bool running = false;
bool lastStopState = HIGH;
bool cameraReady = false;
bool cameraBlocked = true;
unsigned long lastCameraFrame = 0;
String command;

// Status goes to both USB serial (Axiometa Studio) and Bluetooth (iPhone app).
void report(const String& message) {
  Serial.println(message);
  bleSend(message);
}

// Navigation cues: each step is a tone, then silence. Stop beeps (2000 Hz, 300 ms) always override these.
struct Beep { int freq; int onMs; int offMs; };
const Beep NAV_LEFT[] = {{1500, 150, 120}, {1500, 150, 0}};        // two short
const Beep NAV_RIGHT[] = {{1500, 600, 0}};                         // one long
const Beep NAV_STRAIGHT[] = {{1000, 100, 0}};                      // one short, low
const Beep NAV_ARRIVED[] = {{1000, 150, 50}, {1300, 150, 50}, {1600, 300, 0}};  // rising
#define PATTERN(p) p, sizeof(p) / sizeof(p[0])

const Beep* pattern = nullptr;
int patternLength = 0;
int patternStep = 0;
unsigned long stepStarted = 0;
bool stepSounding = false;

void startStep() {
  tone(BUZZER_PIN, pattern[patternStep].freq);
  stepStarted = millis();
  stepSounding = true;
}

void playPattern(const Beep* p, int length) {
  pattern = p;
  patternLength = length;
  patternStep = 0;
  startStep();
}

void cancelPattern() {
  pattern = nullptr;
  noTone(BUZZER_PIN);
}

void servicePattern() {
  if (!pattern) return;
  const Beep& b = pattern[patternStep];
  unsigned long elapsed = millis() - stepStarted;
  if (stepSounding && elapsed >= (unsigned long)b.onMs) {
    noTone(BUZZER_PIN);
    stepSounding = false;
  }
  if (!stepSounding && elapsed >= (unsigned long)(b.onMs + b.offMs)) {
    if (++patternStep >= patternLength) { pattern = nullptr; return; }
    startStep();
  }
}

void stopBeep() {
  cancelPattern();
  tone(BUZZER_PIN, 2000, 300);
}

void cameraStop() {
  if (running) {
    running = false;
    digitalWrite(STOP_LED_PIN, HIGH);
    stopBeep();
    report("Camera stop: object detected or camera unavailable.");
    printStatus();
  }
}

void navCue(const String& cue) {
  // Guide only while walking; a stopped rover must not suggest moving.
  if (!running) { report("Nav ignored: stopped"); return; }
  if (cue == "LEFT") playPattern(PATTERN(NAV_LEFT));
  else if (cue == "RIGHT") playPattern(PATTERN(NAV_RIGHT));
  else if (cue == "STRAIGHT") playPattern(PATTERN(NAV_STRAIGHT));
  else if (cue == "ARRIVED") playPattern(PATTERN(NAV_ARRIVED));
  else { report("Nav ignored: unknown cue " + cue); return; }
  report("Nav cue: " + cue);
}

void handleCommand(const String& command) {
  if (command == "CAM:CLEAR" || command == "CAM:BLOCK") {
    cameraReady = true;
    cameraBlocked = command == "CAM:BLOCK";
    lastCameraFrame = millis();
    if (cameraBlocked) cameraStop();
  } else if (command == "CAM:OFF") {
    cameraReady = false;
    cameraStop();
  } else if (command.startsWith("NAV:")) {
    navCue(command.substring(4));
  }
}

void receiveCommands() {
  String bleCommand;
  while (nextBleCommand(bleCommand)) handleCommand(bleCommand);
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(command);
      command = "";
    } else if (c != '\r') {
      command += c;
      if (command.length() > 40) command = "";
    }
  }
  if (cameraReady && millis() - lastCameraFrame > 2000) {
    cameraReady = false;
    cameraStop();
  }
}

void printStatus() {
  report(String(running ? "Running" : "Stopped") + " | Selected speed: " + speedPercent +
      "% | Control test only: no motors connected");
}

void onButtonPress() {
  if (digitalRead(STOP_PIN) == LOW || running) return;
  if (!cameraReady || cameraBlocked) {
    report("Start blocked: start camera and remove recognized objects from the center first.");
    return;
  }
  running = true;
  digitalWrite(STOP_LED_PIN, LOW);
  printStatus();
}

void setup() {
  pinMode(BTN_PIN, INPUT);
  pinMode(STOP_PIN, INPUT);
  pinMode(STOP_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STOP_LED_PIN, HIGH);
  Serial.begin(115200);
  initBle();
  report("Control test: encoder on P1 starts and sets speed; LED button on P2 stops. Light on = stopped.");
  report("Buzzer test: AX22-0018 on P3. Press the stop button for a short beep.");
  report("Camera interlock enabled: waiting for Camera Test (USB) or the iPhone app (Bluetooth \"Robodog\"). Press encoder to start after a clear detection frame.");
  report("Navigation cues: left = two short, right = one long, straight = one low, arrived = rising.");
  printStatus();
}

void loop() {
  receiveCommands();
  servicePattern();
  encoder.tick();
  long position = encoder.getPosition();
  long delta = position - previousPosition;
  previousPosition = position;
  if (delta != 0) {
    speedPercent = constrain(speedPercent + delta * 5, 0L, 100L);
    printStatus();
  }

  bool btn = digitalRead(BTN_PIN);
  if (btn == HIGH && lastBtnState == LOW && (millis() - lastBtnTime > 150)) {
    onButtonPress();
    lastBtnTime = millis();
  }
  lastBtnState = btn;

  bool stopState = digitalRead(STOP_PIN);
  if (stopState == LOW && lastStopState == HIGH) {
    running = false;
    digitalWrite(STOP_LED_PIN, HIGH);
    stopBeep();
    report("Stop button pressed: 300 ms warning beep.");
    printStatus();
  }
  lastStopState = stopState;
}
