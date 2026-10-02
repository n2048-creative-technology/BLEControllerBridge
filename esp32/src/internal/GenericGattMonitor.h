#pragma once

// Internal: subscribes to every notify/indicate characteristic on every
// service of an already-connected BLEClient and forwards raw bytes.
// This is the "works on literally any BLE peripheral" fallback module --
// independently reusable in any project that just wants raw GATT
// notification dumping (sensor loggers, reverse-engineering unknown
// peripherals, etc), decoupled from BLEControllerBridge's HID parsing.

#include <Arduino.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <functional>
#include <vector>

namespace bcb {

struct GattNotification {
  const uint8_t* data;
  size_t length;
  std::string serviceUUID;
  std::string characteristicUUID;
};

struct GattCharacteristicInfo {
  std::string serviceUUID;
  std::string characteristicUUID;
  bool canRead;
  bool canWrite;
  bool canNotify;
  bool canIndicate;
};

using OnGattNotification = std::function<void(const GattNotification&)>;
using OnGattEnumerated   = std::function<void(const GattCharacteristicInfo&)>;

class GenericGattMonitor {
 public:
  // Walks every service/characteristic on the client, logs each one via
  // the (optional) enumeration callback, and subscribes to every
  // notify/indicate-capable characteristic found. Call once right after
  // a connection is established (and service discovery has happened,
  // which BLEClient::connect() triggers implicitly in this library).
  // Returns the number of characteristics successfully subscribed.
  int subscribeToAll(BLEClient* client);

  void setNotificationCallback(OnGattNotification cb) { _onNotification = cb; }
  void setEnumeratedCallback(OnGattEnumerated cb) { _onEnumerated = cb; }

 private:
  OnGattNotification _onNotification;
  OnGattEnumerated _onEnumerated;
};

}  // namespace bcb
