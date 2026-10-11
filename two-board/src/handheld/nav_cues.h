// Navigation cues on the handle buzzer, played without blocking loop().
// left = two short, right = one long, straight = one low, arrived = rising.
// Stop tones call cancelPattern() first, so they always win.
#pragma once
#include <Arduino.h>

struct Beep { int freq; int onMs; int offMs; };
static const Beep NAV_LEFT[] = {{1500, 150, 120}, {1500, 150, 0}};
static const Beep NAV_RIGHT[] = {{1500, 600, 0}};
static const Beep NAV_STRAIGHT[] = {{1000, 100, 0}};
static const Beep NAV_ARRIVED[] = {{1000, 150, 50}, {1300, 150, 50}, {1600, 300, 0}};
#define PATTERN(p) p, sizeof(p) / sizeof(p[0])

static uint8_t navBuzzerPin = 0;
static const Beep *pattern = nullptr;
static int patternLength = 0, patternStep = 0;
static unsigned long stepStarted = 0;
static bool stepSounding = false;

static void startStep() {
  tone(navBuzzerPin, pattern[patternStep].freq);
  stepStarted = millis();
  stepSounding = true;
}

static void playPattern(const Beep *p, int length) {
  pattern = p;
  patternLength = length;
  patternStep = 0;
  startStep();
}

static void cancelPattern() {
  if (!pattern) return;
  pattern = nullptr;
  noTone(navBuzzerPin);
}

static void servicePattern() {
  if (!pattern) return;
  const Beep &b = pattern[patternStep];
  unsigned long elapsed = millis() - stepStarted;
  if (stepSounding && elapsed >= (unsigned long)b.onMs) {
    noTone(navBuzzerPin);
    stepSounding = false;
  }
  if (!stepSounding && elapsed >= (unsigned long)(b.onMs + b.offMs)) {
    if (++patternStep >= patternLength) { pattern = nullptr; return; }
    startStep();
  }
}

/// Plays the cue named by NAV:<cue>. Returns false for an unknown cue.
static bool playNavCue(const String &cue) {
  if (cue == "LEFT") playPattern(PATTERN(NAV_LEFT));
  else if (cue == "RIGHT") playPattern(PATTERN(NAV_RIGHT));
  else if (cue == "STRAIGHT") playPattern(PATTERN(NAV_STRAIGHT));
  else if (cue == "ARRIVED") playPattern(PATTERN(NAV_ARRIVED));
  else return false;
  return true;
}
