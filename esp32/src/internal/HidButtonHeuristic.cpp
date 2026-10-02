#include "HidButtonHeuristic.h"

namespace bcb {

void HidButtonHeuristic::feed(const uint8_t* data, size_t length) {
  if (data == nullptr || length == 0 || length > 8) {
    return;  // not a plausible simple-report payload; skip
  }

  // Treat up to the first 4 bytes, little-endian, as a button bitmask.
  uint32_t mask = 0;
  size_t n = length < 4 ? length : 4;
  for (size_t i = 0; i < n; i++) {
    mask |= (uint32_t)data[i] << (8 * i);
  }

  if (!_haveLast) {
    _lastMask = mask;
    _haveLast = true;
    // Still report the initial state so consumers see the first press.
  }

  uint32_t changed = mask ^ _lastMask;
  _lastMask = mask;

  if (changed != 0 && _onButton) {
    HeuristicButtonEvent evt;
    evt.buttonMask = mask;
    evt.changedMask = changed;
    _onButton(evt);
  }
}

}  // namespace bcb
