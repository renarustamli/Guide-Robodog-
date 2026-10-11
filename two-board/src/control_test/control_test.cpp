// Control test for the Axiometa Genesis Mini. No motors are driven here.
//   Encoder on P1: turn = set speed, press = start.   LED button on P2: stop.   Buzzer on P3.
//   A computer-side camera test sends CAM:CLEAR / CAM:BLOCK / CAM:OFF (and MIC:START / MIC:STOP)
//   over the serial port; starting is blocked unless the camera reports a clear frame.
//
// This is your sketch. Changes for PlatformIO: #include <Arduino.h> and "pins.h", a forward
// declaration of printStatus() (the Arduino IDE adds those for you, PlatformIO does not), and
// the pin #defines now point at pins.h. Microphone calls follow voice_capture.h:
// beginVoice() once in setup, MIC:START -> requestVoice(), pollVoice() each loop.
#include <Arduino.h>
#include <RotaryEncoder.h>
#include "pins.h"
#include "voice_capture.h"

#define BTN_PIN pins::CTRL_BTN
#define CLK_PIN pins::ENC_CLK
#define DT_PIN pins::ENC_DT
#define STOP_PIN pins::STOP_BTN
#define STOP_LED_PIN pins::STOP_LED
#define BUZZER_PIN pins::BUZZER

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

void printStatus();

void cameraStop() {
 if (running) {
   running = false;
   digitalWrite(STOP_LED_PIN, HIGH);
   noTone(BUZZER_PIN);
   tone(BUZZER_PIN, 2000, 300);
   Serial.println("Camera stop: object detected or camera unavailable.");
   printStatus();
 }
}

void receiveCamera() {
 while (Serial.available()) {
   char c = Serial.read();
   if (c == '\n') {
     if (command == "MIC:START" || command.startsWith("MIC:START:")) {
       // "MIC:START:<n>" uses n as the clip number; plain "MIC:START" counts up by itself.
       cameraStop();
       static uint32_t nextEpoch = 1;
       uint32_t epoch = command.length() > 10 ? (uint32_t)command.substring(10).toInt() : nextEpoch;
       nextEpoch = epoch + 1;
       requestVoice(epoch);
     } else if (command == "MIC:STOP") {
       // voice_capture.h records a fixed 2 s clip, so there is nothing to stop.
       Serial.println("MIC:STOP ignored: clips are a fixed 2 s.");
     } else if (command == "CAM:CLEAR" || command == "CAM:BLOCK") {
       cameraReady = true;
       cameraBlocked = command == "CAM:BLOCK";
       lastCameraFrame = millis();
       if (cameraBlocked) cameraStop();
     } else if (command == "CAM:OFF") {
       cameraReady = false;
       cameraStop();
     }
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
 Serial.print(running ? "Running" : "Stopped");
 Serial.print(" | Selected speed: ");
 Serial.print(speedPercent);
 Serial.println("% | Control test only: no motors connected");
}

void onButtonPress() {
 if (digitalRead(STOP_PIN) == LOW || running) return;
 if (captureBusy) { Serial.println("Start blocked: finish recording first."); return; }
 if (!cameraReady || cameraBlocked) {
   Serial.println("Start blocked: start camera and remove recognized objects from the center first.");
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
 beginVoice();
 Serial.println("Control test: encoder on P1 starts and sets speed; LED button on P2 stops. Light on = stopped.");
 Serial.println("Buzzer test: AX22-0018 on P3. Press the stop button for a short beep.");
 Serial.println("Camera interlock enabled: waiting for computer Camera Test. Press encoder to start after a clear detection frame.");
 printStatus();
}

void loop() {
 receiveCamera();
 pollVoice();
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
   noTone(BUZZER_PIN);
   tone(BUZZER_PIN, 2000, 300);
   Serial.println("Stop button pressed: 300 ms warning beep.");
   printStatus();
 }
 lastStopState = stopState;
}
