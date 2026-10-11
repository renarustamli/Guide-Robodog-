// Bluetooth link to the iPhone app (advertises as "Robodog").
// The phone sends the same text commands as the Studio panel over USB (PING, RECORD, VOICE:..., STOP)
// plus CAM:* from its camera and NAV:* from its route guide. Status lines go to both USB and Bluetooth.
#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <stdarg.h>

// Nordic UART Service, so generic apps like nRF Connect can talk to the board too.
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"     // phone -> board, text commands
#define NUS_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"     // board -> phone, text status lines
#define ROBODOG_AUDIO "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"  // board -> phone, raw 16-bit PCM

#define BLE_COMMAND_LENGTH 100

static BLEServer *bleServer = nullptr;
static BLECharacteristic *bleTx = nullptr, *bleAudio = nullptr;
static volatile bool bleConnected = false;
// BLE callbacks run on another task, so commands are queued and handled in loop().
static QueueHandle_t bleCommands;

class RobodogServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

class RobodogRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String value = c->getValue().c_str();
    // A write may hold one command without a newline (nRF Connect) or several newline-separated ones.
    int start = 0;
    while (start <= (int)value.length()) {
      int end = value.indexOf('\n', start);
      if (end < 0) end = value.length();
      String line = value.substring(start, end);
      line.trim();
      if (line.length() > 0 && line.length() < BLE_COMMAND_LENGTH) {
        char buffer[BLE_COMMAND_LENGTH] = {0};
        line.toCharArray(buffer, BLE_COMMAND_LENGTH);
        xQueueSend(bleCommands, buffer, 0);
      }
      start = end + 1;
    }
  }
};

static void initBle() {
  bleCommands = xQueueCreate(8, BLE_COMMAND_LENGTH);
  BLEDevice::init("Robodog");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new RobodogServerCallbacks());
  BLEService *service = bleServer->createService(NUS_SERVICE);
  bleTx = service->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  bleAudio = service->createCharacteristic(ROBODOG_AUDIO, BLECharacteristic::PROPERTY_NOTIFY);
  bleAudio->addDescriptor(new BLE2902());
  BLECharacteristic *rx = service->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RobodogRxCallbacks());
  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

static bool nextBleCommand(String &command) {
  char buffer[BLE_COMMAND_LENGTH];
  if (xQueueReceive(bleCommands, buffer, 0) != pdTRUE) return false;
  command = buffer;
  return true;
}

/// Largest audio chunk that fits one notification at the negotiated MTU.
static size_t bleAudioChunk() {
  uint16_t mtu = bleServer ? bleServer->getPeerMTU(bleServer->getConnId()) : 0;
  if (mtu <= 23) return 20;
  return min((size_t)(mtu - 3), (size_t)240);
}

static void bleSendAudio(const uint8_t *data, size_t length) {
  if (!bleConnected || !bleAudio) return;
  bleAudio->setValue(const_cast<uint8_t *>(data), length);
  bleAudio->notify();
}

/// A status line to USB serial (Studio) and Bluetooth (iPhone app).
static void report(const String &message) {
  Serial.println(message);
  if (bleConnected && bleTx) {
    bleTx->setValue(message.c_str());
    bleTx->notify();
  }
}

static void reportf(const char *format, ...) {
  char line[160];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  report(line);
}
