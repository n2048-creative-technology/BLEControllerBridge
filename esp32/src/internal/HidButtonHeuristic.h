#pragma once

// Internal: best-effort heuristic button/axis extraction from HID Report
// notifications, with ZERO dependency on a full USB HID Report Descriptor
// parser (writing a complete, correct HID report-map parser is a large
// undertaking -- out of scope for a zero-dependency embedded library).
//
// Strategy: many simple BLE HID gamepads/remotes send a small, fixed-size
// report where the first 1-4 bytes are a plain button bitmask (one bit per
// button) and, if present, 2-4 subsequent signed bytes are analog axes.
// This heuristic treats the FIRST byte of every HID Report notification as
// a button bitmask and diffs it against the previous report to produce
// BCBButtonEvent. It intentionally does NOT attempt to parse axes.
//
// This is a convenience, not a guarantee -- always wire onRawData() too
// and inspect the actual bytes for your specific controller; override this
// heuristic with your own parsing once you know the exact report layout
// (see README.md "Writing a custom parser for your controller" section).

#include <Arduino.h>
#include <functional>

namespace bcb {

struct HeuristicButtonEvent {
  uint32_t buttonMask;
  uint32_t changedMask;
};

using OnHeuristicButton = std::function<void(const HeuristicButtonEvent&)>;

class HidButtonHeuristic {
 public:
  void setCallback(OnHeuristicButton cb) { _onButton = cb; }

  // Feed every raw notification through this; it only acts on payloads
  // whose characteristic UUID matches the standard HID Report
  // characteristic (0x2A4D) OR, since many clones mislabel/omit that, on
  // ANY payload shorter than 8 bytes (typical simple-remote report size) --
  // this casts a deliberately wide net since being a heuristic fallback is
  // its entire purpose.
  void feed(const uint8_t* data, size_t length);

 private:
  uint32_t _lastMask = 0;
  bool _haveLast = false;
  OnHeuristicButton _onButton;
};

}  // namespace bcb
