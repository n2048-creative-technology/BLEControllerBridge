#include "BLEControllerBridge.h"
#include <algorithm>

namespace {
std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

void BLEControllerBridge::begin(const std::string& deviceName) {
  _transport.begin(deviceName);

  _transport.setScanResultCallback([this](const std::string& name, const std::string& address, int rssi) {
    if (_onScanResult) {
      _onScanResult(name, address, rssi);
    }
  });

  _transport.setDeviceFoundFilter([this](BLEAdvertisedDevice* device) {
    std::string name = device->haveName() ? std::string(device->getName().c_str()) : "";
    std::string address = std::string(device->getAddress().toString().c_str());
    int rssi = device->haveRSSI() ? device->getRSSI() : 0;
    return evaluateFilter(name, address, rssi);
  });

  _transport.setConnectedCallback([this](BLEClient* client) {
    _state = BCBState::CONNECTED;
    _connectedAddress = std::string(client->getPeerAddress().toString().c_str());
    // BLE name isn't always available post-connect via BLEClient directly;
    // we keep whatever name the scan filter last observed, if any.

    _gattMonitor.setEnumeratedCallback([](const bcb::GattCharacteristicInfo&) {
      // Enumeration is logged by the raw-data consumer's own tooling if
      // desired; no-op here by default to keep the facade lean. Advanced
      // users can bypass the facade and use GenericGattMonitor directly
      // for enumeration logging (see README "Advanced: bypassing the
      // facade").
    });

    _gattMonitor.setNotificationCallback([this](const bcb::GattNotification& evt) {
      if (_onRawData) {
        BCBRawDataEvent out;
        out.data = evt.data;
        out.length = evt.length;
        out.serviceUUID = evt.serviceUUID;
        out.characteristicUUID = evt.characteristicUUID;
        _onRawData(out);
      }
      _buttonHeuristic.feed(evt.data, evt.length);
      _axisExtractor.feed(evt.data, evt.length);
    });

    _gattMonitor.subscribeToAll(client);

    if (_onConnect) {
      _onConnect(_connectedName, _connectedAddress);
    }
  });

  _transport.setDisconnectedCallback([this]() {
    _state = BCBState::DISCONNECTED;
    if (_onDisconnect) {
      _onDisconnect();
    }
  });

  _buttonHeuristic.setCallback([this](const bcb::HeuristicButtonEvent& evt) {
    if (_onButton) {
      BCBButtonEvent out;
      out.buttonMask = evt.buttonMask;
      out.changedMask = evt.changedMask;
      _onButton(out);
    }
  });

  _axisExtractor.setCallback([this](const bcb::AnalogAxisEvent& evt) {
    if (_onAxis) {
      BCBAxisEvent out;
      out.axisIndex = evt.axisIndex;
      out.value = evt.value;
      _onAxis(out);
    }
  });
}

void BLEControllerBridge::setTargetAddress(const std::string& macAddress) {
  _targetAddress = toLower(macAddress);
  _targetName.clear();
  _customFilter = nullptr;
}

void BLEControllerBridge::setTargetName(const std::string& nameSubstring) {
  _targetName = nameSubstring;
  _targetAddress.clear();
  _customFilter = nullptr;
}

void BLEControllerBridge::clearTargetFilter() {
  _targetAddress.clear();
  _targetName.clear();
  _customFilter = nullptr;
}

bool BLEControllerBridge::evaluateFilter(const std::string& name, const std::string& address, int rssi) {
  if (_customFilter) {
    return _customFilter(name, address, rssi);
  }
  if (!_targetAddress.empty()) {
    return toLower(address) == _targetAddress;
  }
  if (!_targetName.empty()) {
    return name.find(_targetName) != std::string::npos;
  }
  return true;  // no filter -> accept first device seen
}

void BLEControllerBridge::scanAndConnect(uint32_t durationMs) {
  _state = BCBState::SCANNING;
  _transport.startScan(durationMs);
}

void BLEControllerBridge::stopScan() {
  _transport.stopScan();
}

void BLEControllerBridge::disconnect() {
  _transport.disconnect();
}

void BLEControllerBridge::forgetBond() {
  _transport.forgetAllBonds();
}

void BLEControllerBridge::configureAxis(const bcb::AxisFieldConfig& config) {
  _axisExtractor.configureAxis(config);
}

void BLEControllerBridge::clearAxes() {
  _axisExtractor.clearAxes();
}

void BLEControllerBridge::loop() {
  _transport.loop();
}
