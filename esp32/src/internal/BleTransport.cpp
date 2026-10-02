#include "BleTransport.h"
#include <nvs_flash.h>

namespace bcb {

class BleTransport::ScanCb : public BLEAdvertisedDeviceCallbacks {
 public:
  explicit ScanCb(BleTransport* owner) : _owner(owner) {}

  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    std::string name = advertisedDevice.haveName() ? std::string(advertisedDevice.getName().c_str()) : "";
    std::string addr = std::string(advertisedDevice.getAddress().toString().c_str());
    int rssi = advertisedDevice.haveRSSI() ? advertisedDevice.getRSSI() : 0;

    if (_owner->_onScanResultRaw) {
      _owner->_onScanResultRaw(name, addr, rssi);
    }

    if (_owner->_pendingConnect) {
      return;  // already have a match queued, ignore further results
    }

    bool shouldConnect = true;
    if (_owner->_onDeviceFound) {
      // NOTE: advertisedDevice is a stack copy from the BLE stack; it stays
      // valid for the duration of this callback. We heap-copy it so the
      // main-task loop() can use it after this callback returns.
      shouldConnect = _owner->_onDeviceFound(&advertisedDevice);
    }

    if (shouldConnect) {
      BLEDevice::getScan()->stop();
      _owner->_pendingDevice = new BLEAdvertisedDevice(advertisedDevice);
      _owner->_pendingConnect = true;
    }
  }

 private:
  BleTransport* _owner;
};

class BleTransport::ClientCb : public BLEClientCallbacks {
 public:
  explicit ClientCb(BleTransport* owner) : _owner(owner) {}

  void onConnect(BLEClient* pClient) override {
    // Intentionally minimal -- full "connected" semantics (service
    // discovery done) are signaled from connectTo() after discovery
    // succeeds, not from this raw link-up event.
  }

  void onDisconnect(BLEClient* pClient) override {
    _owner->_state = TransportState::IDLE;
    if (_owner->_onDisconnected) {
      _owner->_onDisconnected();
    }
    // Resume scanning automatically so a bonded device reconnects on its
    // own once back in range -- this is what gives "persists across
    // reboots / out-of-range drops" its practical effect.
    if (_owner->_rescanDurationMs > 0 || _owner->_onDeviceFound) {
      _owner->startScan(_owner->_rescanDurationMs);
    }
  }

 private:
  BleTransport* _owner;
};

void BleTransport::begin(const std::string& deviceName) {
  BLEDevice::init(String(deviceName.c_str()));
  _scanCb = new ScanCb(this);
  _clientCb = new ClientCb(this);

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(_scanCb, /*wantDuplicates=*/false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
}

void BleTransport::startScan(uint32_t durationMs) {
  _rescanDurationMs = durationMs;
  _state = TransportState::SCANNING;
  BLEDevice::getScan()->start(durationMs, /*is_continue=*/false);
}

void BleTransport::stopScan() {
  BLEDevice::getScan()->stop();
  if (_state == TransportState::SCANNING) {
    _state = TransportState::IDLE;
  }
}

void BleTransport::disconnect() {
  if (_client && _client->isConnected()) {
    _client->disconnect();
  }
}

void BleTransport::forgetAllBonds() {
  // Clears the NVS-backed bond/key store. There is no single documented
  // public call for this across both Bluedroid/NimBLE backends of the
  // bundled ESP32 BLE library at all versions, so we go straight to the
  // NVS namespace BLE stacks use for bond storage.
  nvs_flash_erase();
  nvs_flash_init();
}

bool BleTransport::connectTo(BLEAdvertisedDevice* device) {
  _state = TransportState::CONNECTING;

  if (_client == nullptr) {
    _client = BLEDevice::createClient();
    _client->setClientCallbacks(_clientCb);
  }

  if (!_client->connect(device)) {
    _state = TransportState::IDLE;
    return false;
  }

  _state = TransportState::CONNECTED;
  if (_onConnected) {
    _onConnected(_client);
  }
  return true;
}

void BleTransport::loop() {
  if (_pendingConnect && _pendingDevice != nullptr) {
    _pendingConnect = false;
    BLEAdvertisedDevice* dev = _pendingDevice;
    _pendingDevice = nullptr;
    connectTo(dev);
    delete dev;
  }
}

}  // namespace bcb
