// voice_capture.h - microphone clips (16 kHz, 16-bit mono) for voice commands.
// A clip is recorded first, then sent to whoever asked for it:
//   USB (Studio panel): AUDIO_BEGIN, PCM:<hex> lines, AUDIO_END
//   Bluetooth (iPhone): AUDIO_BEGIN, raw PCM on the audio characteristic, AUDIO_END:<epoch>:<bytes>
// Needs ble_link.h included first.
#pragma once
#include <Arduino.h>
#include <ESP_I2S.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static const size_t VOICE_CLIP_BYTES = 96000;      // 3 s, long enough for "guide me to Tesco"
static const unsigned long BLE_AUDIO_PACE_MS = 6;  // gap between notifications so none are dropped

static I2SClass i2s;
static uint8_t *recording = nullptr;
static QueueHandle_t captureRequests, captureResults;
static bool micReady = false, captureBusy = false, transferring = false, audioToBle = false;
static size_t audioOffset = 0;
static uint32_t captureEpoch = 0;
static unsigned long lastAudioPacket = 0;

static void captureTask(void*) {
  uint32_t epoch;
  for (;;) {
    if (xQueueReceive(captureRequests, &epoch, portMAX_DELAY) != pdTRUE) continue;
    // Discard buffered audio so each clip starts with fresh speech.
    char discard[2048];
    i2s.readBytes(discard, sizeof(discard));
    size_t count = i2s.readBytes(reinterpret_cast<char*>(recording), VOICE_CLIP_BYTES);
    uint32_t result = count == VOICE_CLIP_BYTES ? epoch : UINT32_MAX;
    xQueueSend(captureResults, &result, portMAX_DELAY);
  }
}
static void beginVoice() {
  pinMode(P4_IO0, OUTPUT); digitalWrite(P4_IO0, LOW);
  pinMode(P4_IO1, INPUT);
  pinMode(P4_IO2, OUTPUT); digitalWrite(P4_IO2, LOW);
  i2s.setPinsPdmRx(P4_IO2, P4_IO1);
  if (!i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    report("ERROR:PDM mic init failed"); return;
  }
  // Keep the clip in PSRAM so Wi-Fi (ESP-NOW) and Bluetooth keep their internal RAM.
  recording = static_cast<uint8_t*>(psramFound() ? ps_malloc(VOICE_CLIP_BYTES) : malloc(VOICE_CLIP_BYTES));
  captureRequests = xQueueCreate(1, sizeof(uint32_t));
  captureResults = xQueueCreate(1, sizeof(uint32_t));
  if (!recording || !captureRequests || !captureResults ||
      xTaskCreate(captureTask, "voice", 4096, nullptr, 1, nullptr) != pdPASS) {
    report("ERROR:Capture allocation failed"); return;
  }
  micReady = true;
}
static void requestVoice(uint32_t epoch, bool toBle) {
  if (!micReady || captureBusy) { report("ERROR:Microphone unavailable or busy"); return; }
  captureEpoch = epoch; captureBusy = true; audioToBle = toBle;
  xQueueSend(captureRequests, &epoch, 0);
  reportf("AUDIO_BEGIN:%lu", (unsigned long)epoch);
}
static void finishTransfer() {
  reportf("AUDIO_END:%lu:%u", (unsigned long)captureEpoch, (unsigned)audioOffset);
  transferring = false; captureBusy = false;
}
static void pollVoice() {
  uint32_t result;
  if (!transferring && captureBusy && xQueueReceive(captureResults, &result, 0) == pdTRUE) {
    if (result == UINT32_MAX) { captureBusy = false; report("ERROR:Incomplete microphone capture"); return; }
    transferring = true; audioOffset = 0;
  }
  if (!transferring) return;

  if (audioToBle) {
    if (!bleConnected) { transferring = false; captureBusy = false; report("ERROR:Bluetooth lost during audio"); return; }
    if (millis() - lastAudioPacket < BLE_AUDIO_PACE_MS) return;
    lastAudioPacket = millis();
    size_t n = min(bleAudioChunk(), VOICE_CLIP_BYTES - audioOffset);
    bleSendAudio(recording + audioOffset, n);
    audioOffset += n;
    if (audioOffset == VOICE_CLIP_BYTES) finishTransfer();
    return;
  }

  // Small serial writes leave physical controls and ESP-NOW heartbeat responsive.
  if (Serial.availableForWrite() >= 140) {
    char line[140];
    memcpy(line, "PCM:", 4);
    const char hex[] = "0123456789abcdef";
    size_t n = min((size_t)64, VOICE_CLIP_BYTES - audioOffset);
    for (size_t j=0; j<n; ++j) {
      uint8_t b = recording[audioOffset+j];
      line[4+2*j]=hex[b>>4]; line[5+2*j]=hex[b&15];
    }
    line[4+2*n]='\n'; Serial.write((uint8_t*)line, 5+2*n);
    audioOffset += n;
    if (audioOffset == VOICE_CLIP_BYTES) finishTransfer();
  }
}
