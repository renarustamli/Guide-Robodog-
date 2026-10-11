// Receiver side of the ESP-NOW protocol (the handheld has its own rover_radio.h in src/handheld/).
// Your file plus the sender filter (ROVER_ALLOWED_SENDER).
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Shared protocol for this rover and its handheld controller.
// Channel 1, version 1, signed PWM values: -255 through 255.
struct RoverPacket {
  uint32_t magic;
  uint32_t sequence;
  int16_t left;
  int16_t right;
};
static_assert(sizeof(RoverPacket) == 12, "Rover packet layout changed");
const uint32_t ROVER_MAGIC = 0x72A19C43;
QueueHandle_t radioInbox = nullptr;

// Only packets from the configured handheld are obeyed. Set this to all zeros to disable radio input;
// update it if the handheld is replaced.
const uint8_t ROVER_ALLOWED_SENDER[6] = {0xA0, 0xF2, 0x62, 0xEB, 0x92, 0xD8};
volatile uint32_t radioRejected = 0;   // packets dropped because the sender was not the handheld

inline bool senderAllowed(const uint8_t *mac) {
  static const uint8_t none[6] = {0, 0, 0, 0, 0, 0};
  return memcmp(ROVER_ALLOWED_SENDER, none, 6) != 0 && memcmp(mac, ROVER_ALLOWED_SENDER, 6) == 0;
}

void receiveRoverPacket(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (!radioInbox || length != sizeof(RoverPacket)) return;
  if (!info || !senderAllowed(info->src_addr)) { radioRejected = radioRejected + 1; return; }
  RoverPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != ROVER_MAGIC || packet.left < -255 || packet.left > 255 ||
      packet.right < -255 || packet.right > 255) return;
  xQueueOverwrite(radioInbox, &packet);
}

bool startRoverRadio() {
  radioInbox = xQueueCreate(1, sizeof(RoverPacket));
  if (!radioInbox) { Serial.println("ERROR:Radio queue allocation failed"); return false; }
  if (!WiFi.mode(WIFI_STA) || !WiFi.setSleep(false)) {
    Serial.println("ERROR:Radio startup failed"); return false;
  }
  esp_err_t result = esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (result == ESP_OK) result = esp_now_init();
  if (result == ESP_OK) result = esp_now_register_recv_cb(receiveRoverPacket);
  if (result != ESP_OK) {
    Serial.printf("ERROR:ESP-NOW startup failed (%d)\n", (int)result);
    return false;
  }
  Serial.printf("RADIO:Receiver ready, channel 1, MAC %s\n", WiFi.macAddress().c_str());
  return true;
}

bool readRoverRadio(RoverPacket &packet) {
  return radioInbox && xQueueReceive(radioInbox, &packet, 0) == pdTRUE;
}
