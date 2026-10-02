#include "AnalogAxisExtractor.h"

namespace bcb {

void AnalogAxisExtractor::configureAxis(const AxisFieldConfig& config) {
  for (auto& axis : _axes) {
    if (axis.config.index == config.index) {
      axis.config = config;
      axis.haveLast = false;
      return;
    }
  }
  AxisState state;
  state.config = config;
  _axes.push_back(state);
}

void AnalogAxisExtractor::clearAxes() {
  _axes.clear();
}

int32_t AnalogAxisExtractor::decode(const uint8_t* data, size_t length, const AxisFieldConfig& cfg, bool* ok) {
  size_t needed;
  switch (cfg.width) {
    case AxisWidth::UINT8:
    case AxisWidth::INT8:
      needed = 1;
      break;
    default:
      needed = 2;
      break;
  }

  if (cfg.byteOffset + needed > length) {
    *ok = false;
    return 0;
  }
  *ok = true;

  const uint8_t* p = data + cfg.byteOffset;
  switch (cfg.width) {
    case AxisWidth::UINT8:
      return (int32_t)p[0];
    case AxisWidth::INT8:
      return (int32_t)(int8_t)p[0];
    case AxisWidth::UINT16_LE:
      return (int32_t)(uint16_t)(p[0] | (p[1] << 8));
    case AxisWidth::INT16_LE:
      return (int32_t)(int16_t)(p[0] | (p[1] << 8));
    case AxisWidth::UINT16_BE:
      return (int32_t)(uint16_t)((p[0] << 8) | p[1]);
    case AxisWidth::INT16_BE:
      return (int32_t)(int16_t)((p[0] << 8) | p[1]);
  }
  *ok = false;
  return 0;
}

void AnalogAxisExtractor::feed(const uint8_t* data, size_t length) {
  if (data == nullptr) return;

  for (auto& axis : _axes) {
    bool ok = false;
    int32_t value = decode(data, length, axis.config, &ok);
    if (!ok) continue;  // payload too short for this field, skip silently

    if (!axis.haveLast) {
      axis.lastValue = value;
      axis.haveLast = true;
      if (_onAxis) {
        AnalogAxisEvent evt;
        evt.axisIndex = axis.config.index;
        evt.value = value;
        evt.rawValue = value;
        _onAxis(evt);  // report initial state too
      }
      continue;
    }

    int32_t delta = value - axis.lastValue;
    if (delta < 0) delta = -delta;

    if (delta > axis.config.deadzone) {
      axis.lastValue = value;
      if (_onAxis) {
        AnalogAxisEvent evt;
        evt.axisIndex = axis.config.index;
        evt.value = value;
        evt.rawValue = value;
        _onAxis(evt);
      }
    }
  }
}

}  // namespace bcb
