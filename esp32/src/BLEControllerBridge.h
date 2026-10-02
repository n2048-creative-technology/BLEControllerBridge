#pragma once

// BLEControllerBridge -- zero-dependency BLE controller input bridge for
// ESP32 (Arduino core). Discovers, connects, bonds (persists across
// reboots), and streams input from ANY BLE controller/remote: commercial
// HID gamepads AND generic/no-name BLE peripherals with fully custom GATT
// services.
//
// DEPENDENCIES: NONE beyond the ESP32 Arduino core's built-in BLE library
// (BLEDevice.h et al., bundled with every esp32:esp32 board install --
// no Library Manager installs, no Bluepad32, no NimBLE-Arduino required).
//
// MODULAR DESIGN (see src/internal/):
//   BleTransport        -- scan/connect/bond lifecycle only. Reusable
//                           standalone for any "find + connect + persist
//                           bonding" need, no controller semantics at all.
//   GenericGattMonitor   -- subscribes to every notify/indicate
//                           characteristic on a connected client, forwards
//                           raw bytes. Reusable standalone for logging any
//                           BLE peripheral's GATT traffic.
//   HidButtonHeuristic   -- best-effort button-bitmask diffing over raw
//                           notification bytes. Optional convenience layer,
//                           not a full USB HID report-descriptor parser.
// BLEControllerBridge composes all three behind one event-driven facade so
// typical use touches only THIS header. Advanced use can include the
// internal/ headers directly and skip the facade.
//
// See README.md for the full guide: wiring a new ESP32 project, the full
// API reference, a Raspberry Pi integration path, and how to write a
// custom per-device parser once you've identified your controller's exact
// report format via onRawData().

#include <Arduino.h>
#include <functional>
#include <string>

#include "internal/BleTransport.h"
#include "internal/GenericGattMonitor.h"
#include "internal/HidButtonHeuristic.h"
#include "internal/AnalogAxisExtractor.h"

// --- Public event payload types -----------------------------------------

/// Fired whenever the heuristic button-bitmask diff detects a change.
/// See internal/HidButtonHeuristic.h for exactly what this does and does
/// not guarantee -- it is a convenience, not a certified HID parser.
struct BCBButtonEvent {
  uint32_t buttonMask;
  uint32_t changedMask;
};

/// Fired for EVERY notification/indication from EVERY subscribed
/// characteristic, parsed or not. This is the guaranteed-to-work path for
/// devices with no recognized/heuristically-matched report structure --
/// always wire this up at least once while bringing up a new controller.
struct BCBRawDataEvent {
  const uint8_t* data;
  size_t length;
  std::string serviceUUID;
  std::string characteristicUUID;
};

/// Fired when a configured analog axis field changes beyond its deadzone.
/// See AnalogAxisExtractor -- this requires explicit per-axis configuration
/// via configureAxis() (byte offset/width identified from onRawData()
/// inspection), it is NOT auto-detected from the device.
struct BCBAxisEvent {
  uint8_t axisIndex;
  int32_t value;
};

enum class BCBState {
  DISCONNECTED,
  SCANNING,
  CONNECTING,
  CONNECTED
};

using BCBConnectCallback    = std::function<void(const std::string& name, const std::string& address)>;
using BCBDisconnectCallback = std::function<void()>;
using BCBButtonCallback     = std::function<void(const BCBButtonEvent&)>;
using BCBAxisCallback       = std::function<void(const BCBAxisEvent&)>;
using BCBRawDataCallback    = std::function<void(const BCBRawDataEvent&)>;
using BCBScanResultCallback = std::function<void(const std::string& name, const std::string& address, int rssi)>;
/// Return true to accept this device and stop scanning/connect to it.
using BCBDeviceFilter       = std::function<bool(const std::string& name, const std::string& address, int rssi)>;

class BLEControllerBridge {
 public:
  // --- Setup -------------------------------------------------------------

  /// Initializes the BLE stack. Call once from setup().
  void begin(const std::string& deviceName = "BLEControllerBridge");

  /// Restrict connection to one device by BLE MAC ("aa:bb:cc:dd:ee:ff",
  /// case-insensitive). Overrides any name filter or custom filter.
  void setTargetAddress(const std::string& macAddress);

  /// Restrict connection to devices whose advertised name CONTAINS this
  /// substring (case-sensitive). Overrides any address filter.
  void setTargetName(const std::string& nameSubstring);

  /// Full custom accept/reject logic, called for every advertisement.
  /// Takes precedence over address/name filters if set.
  void setDeviceFilter(BCBDeviceFilter filter) { _customFilter = filter; }

  /// Clears all filters -- connects to the first connectable device found.
  /// Use this for initial discovery of an unknown controller (pair
  /// onScanResult() to log every candidate's name/address/RSSI first).
  void clearTargetFilter();

  // --- Lifecycle -----------------------------------------------------------

  /// Starts scanning for the configured target and connects automatically
  /// on match. Non-blocking -- call loop() continuously afterwards.
  /// durationMs = 0 scans indefinitely until a match or stopScan().
  void scanAndConnect(uint32_t durationMs = 0);

  void stopScan();
  void disconnect();

  /// Erases ALL stored BLE bond keys (NVS). Use for a "forget device"
  /// action; this defeats reboot-persistence until re-paired.
  void forgetBond();

  /// Registers an analog axis field (byte offset/width/deadzone) to
  /// extract from every raw notification. Identify the offset/width by
  /// inspecting onRawData() output while moving the stick/trigger on your
  /// specific controller -- see README "Handling analog axes (joysticks,
  /// triggers)". Call any time; re-registering the same axisIndex replaces
  /// its config and resets its change-detection baseline.
  void configureAxis(const bcb::AxisFieldConfig& config);

  /// Clears all configured analog axis fields.
  void clearAxes();

  /// Call every loop() iteration.
  void loop();

  // --- Status --------------------------------------------------------------

  BCBState state() const { return _state; }
  bool isConnected() const { return _state == BCBState::CONNECTED; }
  std::string connectedName() const { return _connectedName; }
  std::string connectedAddress() const { return _connectedAddress; }

  // --- Event subscriptions ---------------------------------------------

  void onConnect(BCBConnectCallback cb) { _onConnect = cb; }
  void onDisconnect(BCBDisconnectCallback cb) { _onDisconnect = cb; }
  void onScanResult(BCBScanResultCallback cb) { _onScanResult = cb; }

  /// Heuristic button-bitmask change (see HidButtonHeuristic). Convenience
  /// only -- verify against onRawData() for your specific device.
  void onButton(BCBButtonCallback cb) { _onButton = cb; }

  /// Fires when a configured analog axis field changes beyond its
  /// deadzone (see configureAxis()). No axes fire until configured.
  void onAxis(BCBAxisCallback cb) { _onAxis = cb; }

  /// Every raw notification from every subscribed characteristic.
  /// Always wire this up when bringing up a new/unknown controller.
  void onRawData(BCBRawDataCallback cb) { _onRawData = cb; }

 private:
  bcb::BleTransport _transport;
  bcb::GenericGattMonitor _gattMonitor;
  bcb::HidButtonHeuristic _buttonHeuristic;
  bcb::AnalogAxisExtractor _axisExtractor;

  BCBState _state = BCBState::DISCONNECTED;
  std::string _connectedName;
  std::string _connectedAddress;
  std::string _targetAddress;   // lowercased
  std::string _targetName;
  BCBDeviceFilter _customFilter;

  BCBConnectCallback _onConnect;
  BCBDisconnectCallback _onDisconnect;
  BCBScanResultCallback _onScanResult;
  BCBButtonCallback _onButton;
  BCBAxisCallback _onAxis;
  BCBRawDataCallback _onRawData;

  bool evaluateFilter(const std::string& name, const std::string& address, int rssi);
};
