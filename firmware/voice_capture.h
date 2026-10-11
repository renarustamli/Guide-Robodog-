#pragma once
#include <Arduino.h>
#include <ESP_I2S.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static I2SClass i2s;
static uint8_t recording[64000];
static QueueHandle_t captureRequests, captureResults;
static bool micReady = false, captureBusy = false, transferring = false;
static size_t audioOffset = 0;
static uint32_t captureEpoch = 0;

static void captureTask(void*) {
  uint32_t epoch;
  for (;;) {
    if (xQueueReceive(captureRequests, &epoch, portMAX_DELAY) != pdTRUE) continue;
    // Discard buffered audio so each clip starts with fresh speech.
    char discard[2048];
    i2s.readBytes(discard, sizeof(discard));
    size_t count = i2s.readBytes(reinterpret_cast<char*>(recording), sizeof(recording));
    uint32_t result = count == sizeof(recording) ? epoch : UINT32_MAX;
    xQueueSend(captureResults, &result, portMAX_DELAY);
  }
}
static void beginVoice() {
  pinMode(P4_IO0, OUTPUT); digitalWrite(P4_IO0, LOW);
  pinMode(P4_IO1, INPUT);
  pinMode(P4_IO2, OUTPUT); digitalWrite(P4_IO2, LOW);
  i2s.setPinsPdmRx(P4_IO2, P4_IO1);
  if (!i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("ERROR:PDM mic init failed"); return;
  }
  captureRequests = xQueueCreate(1, sizeof(uint32_t));
  captureResults = xQueueCreate(1, sizeof(uint32_t));
  if (!captureRequests || !captureResults || xTaskCreate(captureTask, "voice", 4096, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("ERROR:Capture task allocation failed"); return;
  }
  micReady = true;
}
static void requestVoice(uint32_t epoch) {
  if (!micReady || captureBusy) { Serial.println("ERROR:Microphone unavailable or busy"); return; }
  captureEpoch = epoch; captureBusy = true;
  xQueueSend(captureRequests, &epoch, 0);
  Serial.printf("AUDIO_BEGIN:%lu\n", (unsigned long)epoch);
}
static void pollVoice() {
  uint32_t result;
  if (!transferring && captureBusy && xQueueReceive(captureResults, &result, 0) == pdTRUE) {
    if (result == UINT32_MAX) { captureBusy = false; Serial.println("ERROR:Incomplete microphone capture"); return; }
    transferring = true; audioOffset = 0;
  }
  // Small serial writes leave physical controls and ESP-NOW heartbeat responsive.
  if (transferring && Serial.availableForWrite() >= 140) {
    char line[140];
    memcpy(line, "PCM:", 4);
    const char hex[] = "0123456789abcdef";
    size_t n = min((size_t)64, sizeof(recording) - audioOffset);
    for (size_t j=0; j<n; ++j) {
      uint8_t b = recording[audioOffset+j];
      line[4+2*j]=hex[b>>4]; line[5+2*j]=hex[b&15];
    }
    line[4+2*n]='\n'; Serial.write((uint8_t*)line, 5+2*n);
    audioOffset += n;
    if (audioOffset == sizeof(recording)) {
      Serial.printf("AUDIO_END:%lu\n", (unsigned long)captureEpoch);
      transferring = false; captureBusy = false;
    }
  }
}
