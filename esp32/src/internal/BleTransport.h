#pragma once

// Internal: low-level BLE central (scan/connect/bond) wrapper.
// Zero dependencies beyond the ESP32 Arduino core's built-in BLE library.
// Not part of the public API -- BLEControllerBridge is the facade clients use.
//
// Separated from BLEControllerBridge so it can be reused standalone in any
// project that just needs "find and connect to a BLE peripheral with
// persistent bonding" without the HID/raw-data event plumbing above it.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include <functional>
#include <string>

namespace bcb {

enum class TransportState {
  IDLE,
  SCANNING,
  CONNECTING,
  CONNECTED
};

using OnDeviceFound   = std::function<bool(BLEAdvertisedDevice* device)>;  // return true to connect
using OnScanResultRaw = std::function<void(const std::string& name, const std::string& address, int rssi)>;
using OnConnected     = std::function<void(BLEClient* client)>;
using OnDisconnected  = std::function<void()>;

// Thin RAII-ish wrapper around BLEScan + BLEClient lifecycle. One instance
// manages exactly one active/target connection at a time (sufficient for
// single-controller robotics/animatronics use -- see README for
// multi-controller notes).
class BleTransport {
 public:
  void begin(const std::string& deviceName);

  // onDeviceFound is consulted for every advertisement; return true to stop
  // scanning and connect to that device.
  void setDeviceFoundFilter(OnDeviceFound cb) { _onDeviceFound = cb; }
  void setScanResultCallback(OnScanResultRaw cb) { _onScanResultRaw = cb; }
  void setConnectedCallback(OnConnected cb) { _onConnected = cb; }
  void setDisconnectedCallback(OnDisconnected cb) { _onDisconnected = cb; }

  void startScan(uint32_t durationMs);
  void stopScan();
  void disconnect();
  void forgetAllBonds();

  // Call every loop() -- processes the "need to connect now" flag set by
  // the scan callback (BLE callbacks run on a separate task; connecting
  // directly from that task is unsafe, so we defer to the Arduino task).
  void loop();

  TransportState state() const { return _state; }
  BLEClient* client() const { return _client; }

 private:
  class ScanCb;
  class ClientCb;

  BLEClient* _client = nullptr;
  ScanCb* _scanCb = nullptr;
  ClientCb* _clientCb = nullptr;

  TransportState _state = TransportState::IDLE;
  volatile bool _pendingConnect = false;
  BLEAdvertisedDevice* _pendingDevice = nullptr;
  uint32_t _rescanDurationMs = 0;

  OnDeviceFound _onDeviceFound;
  OnScanResultRaw _onScanResultRaw;
  OnConnected _onConnected;
  OnDisconnected _onDisconnected;

  bool connectTo(BLEAdvertisedDevice* device);

  friend class ScanCb;
  friend class ClientCb;
};

}  // namespace bcb
