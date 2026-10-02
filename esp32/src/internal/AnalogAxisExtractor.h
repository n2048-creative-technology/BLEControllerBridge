#pragma once

// Internal: configurable analog axis extraction from raw notification
// bytes. Zero dependencies, same reusable-module pattern as
// HidButtonHeuristic -- you register WHERE each axis lives in the report
// (byte offset, width, signed-ness) once you've identified the layout via
// onRawData(), and this module does the byte-level decode + change
// detection on every subsequent packet.
//
// This is NOT a HID Report Descriptor parser (it doesn't auto-detect axis
// layout from the device's self-described descriptor) -- it's a thin,
// explicit "here's where the bytes are" decoder. Pair it with onRawData()
// during bring-up of a new controller: dump raw bytes, move the stick/
// trigger, see which byte(s) change, then register the field here.

#include <Arduino.h>
#include <functional>
#include <vector>
#include <cstdint>

namespace bcb {

enum class AxisWidth {
  UINT8,
  INT8,
  UINT16_LE,
  INT16_LE,
  UINT16_BE,
  INT16_BE,
};

struct AxisFieldConfig {
  uint8_t index;        // your chosen logical axis index (0, 1, 2, ...)
  size_t byteOffset;     // offset into the notification payload
  AxisWidth width;       // field size/signedness/endianness
  int32_t deadzone = 0;  // ignore changes smaller than this (noise filter)
};

struct AnalogAxisEvent {
  uint8_t axisIndex;
  int32_t value;       // sign-extended/widened to int32_t regardless of source width
  int32_t rawValue;    // same as value, kept distinct in case of future scaling
};

using OnAnalogAxis = std::function<void(const AnalogAxisEvent&)>;

class AnalogAxisExtractor {
 public:
  /// Registers one axis field. Call once per axis during setup(), after
  /// you've identified byte offsets via onRawData(). Re-registering the
  /// same axisIndex replaces the previous config.
  void configureAxis(const AxisFieldConfig& config);

  /// Removes all configured axes (e.g. when switching target device type).
  void clearAxes();

  void setCallback(OnAnalogAxis cb) { _onAxis = cb; }

  /// Feed every raw notification through this. Silently ignores payloads
  /// too short for a configured field's offset+width (defensive -- avoids
  /// out-of-bounds reads if a device sends a shorter report than expected).
  void feed(const uint8_t* data, size_t length);

 private:
  struct AxisState {
    AxisFieldConfig config;
    int32_t lastValue = 0;
    bool haveLast = false;
  };

  std::vector<AxisState> _axes;
  OnAnalogAxis _onAxis;

  static int32_t decode(const uint8_t* data, size_t length, const AxisFieldConfig& cfg, bool* ok);
};

}  // namespace bcb
