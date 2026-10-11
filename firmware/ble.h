#pragma once
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Nordic UART Service UUIDs, so generic apps like nRF Connect can talk to the board.
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> board
#define NUS_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // board -> phone

#define BLE_COMMAND_LENGTH 48

BLECharacteristic* bleTx = nullptr;
bool bleConnected = false;
// BLE callbacks run on another task, so commands are queued and handled in loop().
QueueHandle_t bleCommands;

class RobodogServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

class RobodogRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
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

void initBle() {
  bleCommands = xQueueCreate(8, BLE_COMMAND_LENGTH);
  BLEDevice::init("Robodog");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new RobodogServerCallbacks());
  BLEService* service = server->createService(NUS_SERVICE);
  bleTx = service->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic* rx = service->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RobodogRxCallbacks());
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool nextBleCommand(String& command) {
  char buffer[BLE_COMMAND_LENGTH];
  if (xQueueReceive(bleCommands, buffer, 0) != pdTRUE) return false;
  command = buffer;
  return true;
}

void bleSend(const String& message) {
  if (!bleConnected || !bleTx) return;
  bleTx->setValue(message.c_str());
  bleTx->notify();
}
