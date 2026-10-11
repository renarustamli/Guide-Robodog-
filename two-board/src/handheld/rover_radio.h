// Handheld side of the ESP-NOW protocol: your Studio Axiometa file.
// The only addition is one line in beginRadio() that prints the handheld's own MAC address:
// paste it into ROVER_ALLOWED_SENDER in src/receiver/rover_radio.h.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

struct RoverPacket {
  uint32_t magic;
  uint32_t sequence;
  int16_t left;
  int16_t right;
};
static_assert(sizeof(RoverPacket) == 12, "Receiver requires 12 bytes");
static const uint8_t roverMac[] = {0xA0,0xF2,0x62,0xEB,0x98,0xBC};
static bool radioReady = false;
static uint32_t sequence = 0;

static bool radioCheck(esp_err_t result, const char *operation) {
  if (result == ESP_OK) return true;
  Serial.printf("ERROR:Radio %s: %s\n", operation, esp_err_to_name(result));
  return false;
}
static void beginRadio() {
  WiFi.setSleep(false);
  if (!WiFi.mode(WIFI_STA)) { Serial.println("ERROR:Station start failed"); return; }
  if (!radioCheck(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE), "channel")) return;
  if (!radioCheck(esp_now_init(), "init")) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, roverMac, 6);
  peer.channel = 1;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (!radioCheck(esp_now_add_peer(&peer), "peer")) return;
  radioReady = true;
  Serial.println("RADIO:Unicast A0:F2:62:EB:98:BC channel 1, 12 bytes; send acceptance is not rover confirmation");
  Serial.printf("RADIO:Handheld MAC %s (put this in ROVER_ALLOWED_SENDER on the receiver)\n", WiFi.macAddress().c_str());
}
static void sendRover(int16_t left, int16_t right) {
  if (!radioReady) return;
  RoverPacket packet = {0x72A19C43, sequence++, left, right};
  static unsigned long lastError = 0;
  esp_err_t result = esp_now_send(roverMac, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  if (result != ESP_OK && millis() - lastError > 1000) {
    radioCheck(result, "send"); lastError = millis();
  }
}
